#!/usr/bin/env python3
import os
import csv
from enum import Enum
from typing import Dict, List, Optional

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from control_msgs.msg import SingleDOFStateStamped
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from utils import (
    clamp_float,
    axes_to_pwm_raw,
    vertical_mix_pwm_us,
    clamp,
    normalize_xy,
    trigger_to_01,
)



def _load_yaml(path: str) -> dict:
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data if isinstance(data, dict) else {}


class ControlMode(str, Enum):
    GAMEPAD = "gamepad"
    AUV_CONTROLLER = "auv_controller"
    GAMEPAD_TWIST = "gamepad_twist"


class Ll_controller(Node):
    def __init__(self) -> None:
        super().__init__("ll_controller")

        self.declare_parameter("config_path", "")
        self.CONFIG_PATH = str(self.get_parameter("config_path").value)

        self.get_logger().info(f"config file: {self.CONFIG_PATH}")
        config = _load_yaml(self.CONFIG_PATH)

        self.GAMEPAD_SPAN = float(config.get("gamepad_span", 100.0))
        self.NEUTRAL_US = float(config.get("neutral_us", 1500.0))
        self.MIN_US = float(config.get("min_pwm_us", 1100.0))
        self.MAX_US = float(config.get("max_pwm_us", 1900.0))
        self.UPDATE_RATE_HZ = float(config.get("update_rate_hz", 200.0))
        self.TOPIC_TIMEOUT_S = float(config.get("topic_timeout_s", 0.3))

        self.TWIST_VEL_XY = float(config.get("twist_vel_xy", 0.4))
        self.TWIST_VEL_Z = float(config.get("twist_vel_z", 0.3))
        self.TWIST_YAW_RATE = float(config.get("twist_yaw_rate", 0.6))
        self.TWIST_PITCH_RATE = float(config.get("twist_pitch_rate", 0.6))

        self.ENABLE_PITCH_CONTROL = bool(config.get("enable_pitch_control", False))
        self.kgf_to_N = float(config.get("Kgf_to_N", 0.980665))
        self.isArmed = True # TODO default to False and require gamepad arming for safety

        self.num_thrusters = 6
        self.channels = list(range(self.num_thrusters))

        self.topics: List[str] = [
            "/thruster_1_controller/status",
            "/thruster_2_controller/status",
            "/thruster_3_controller/status",
            "/thruster_4_controller/status",
            "/thruster_5_controller/status",
            "/thruster_6_controller/status",
        ]

        self.declare_parameter("joy_button_a", 0)
        self.declare_parameter("joy_button_b", 1)
        self.declare_parameter("joy_button_x", 2)
        self.declare_parameter("joy_button_power", 8)
        self._btn_a: int = int(self.get_parameter("joy_button_a").value)
        self._btn_b: int = int(self.get_parameter("joy_button_b").value)
        self._btn_x: int = int(self.get_parameter("joy_button_x").value)
        self._btn_power: int = int(self.get_parameter("joy_button_power").value)

        self._mode: ControlMode = ControlMode.AUV_CONTROLLER
        self._prev_joy_buttons: Optional[List[int]] = None

        # CSV path
        csv_default = "/home/ubuntu/ws_blue/src/bubble_controller/bubble_ll_controller/t200_measured_data/pwm_thrust_measurement.csv"
        self.csv_path = str(config.get("pwm_thrust_csv_pth", csv_default))

        # Load measured PWM -> thrust data
        self.pwm_samples_us, self.thrust_samples_n = self._load_pwm_thrust_csv(self.csv_path)
        self.get_logger().info(
            f"Loaded {len(self.pwm_samples_us)} PWM->thrust samples from {self.csv_path}"
        )

        # Debug pubs
        self.pub_pwm_us = self.create_publisher(
            Float64MultiArray,
            "/thrusters/pwm_us",
            10,
        )

        self.pub_thrust_n = self.create_publisher(
            Float64MultiArray,
            "/thrusters/thrust_n",
            10,
        )

        self.pub_twist_ref = self.create_publisher(
            Twist,
            "/adaptive_integral_terminal_sliding_mode_controller/reference",
            10,
        )

        now = self.get_clock().now()
        self.last_out: Dict[int, float] = {i: self.NEUTRAL_US for i in range(self.num_thrusters)}
        self.last_time: Dict[int, Time] = {i: now for i in range(self.num_thrusters)}

        # AUV controller subscribers
        self.sub1 = self.create_subscription(
            SingleDOFStateStamped, self.topics[0], lambda m: self._cb_thruster(0, m), 10
        )
        self.sub2 = self.create_subscription(
            SingleDOFStateStamped, self.topics[1], lambda m: self._cb_thruster(1, m), 10
        )
        self.sub3 = self.create_subscription(
            SingleDOFStateStamped, self.topics[2], lambda m: self._cb_thruster(2, m), 10
        )
        self.sub4 = self.create_subscription(
            SingleDOFStateStamped, self.topics[3], lambda m: self._cb_thruster(3, m), 10
        )
        self.sub5 = self.create_subscription(
            SingleDOFStateStamped, self.topics[4], lambda m: self._cb_thruster(4, m), 10
        )
        self.sub6 = self.create_subscription(
            SingleDOFStateStamped, self.topics[5], lambda m: self._cb_thruster(5, m), 10
        )

        # ROS topics bridged to Gazebo cmd_thrust
        self.pub1 = self.create_publisher(Float64, "/bluerov2/thruster1_cmd", 10)
        self.pub2 = self.create_publisher(Float64, "/bluerov2/thruster2_cmd", 10)
        self.pub3 = self.create_publisher(Float64, "/bluerov2/thruster3_cmd", 10)
        self.pub4 = self.create_publisher(Float64, "/bluerov2/thruster4_cmd", 10)
        self.pub5 = self.create_publisher(Float64, "/bluerov2/thruster5_cmd", 10)
        self.pub6 = self.create_publisher(Float64, "/bluerov2/thruster6_cmd", 10)
        self.thrust_publishers = [
            self.pub1, self.pub2, self.pub3, self.pub4, self.pub5, self.pub6
        ]

        self.subjoy = self.create_subscription(Joy, "/joy", self._cb_joy, 10)

        self.timer = self.create_timer(1.0 / self.UPDATE_RATE_HZ, self._on_timer)

        self.get_logger().info(
            "Started. "
            f"mode={self._mode.value}, update={self.UPDATE_RATE_HZ}Hz, "
            f"joy A={self._btn_a}, B={self._btn_b}, X={self._btn_x}, "
            f"TwistXY={self.TWIST_VEL_XY} m/s TwistZ={self.TWIST_VEL_Z} m/s "
            f"YawRate={self.TWIST_YAW_RATE} rad/s PitchRate={self.TWIST_PITCH_RATE} rad/s"
        )

    def _load_pwm_thrust_csv(self, path: str):
        if not os.path.exists(path):
            raise FileNotFoundError(f"CSV not found: {path}")

        pwm_us: List[float] = []
        thrust_n: List[float] = []

        with open(path, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                # support either µ or u
                pwm_key = None
                for k in row.keys():
                    ks = k.strip()
                    if ks in ("PWM (µs)", "PWM (us)", "PWM"):
                        pwm_key = k
                        break

                thrust_key = None
                for k in row.keys():
                    ks = k.strip()
                    if ks in ("Force (Kg f)", "Force (kgf)", "Force"):
                        thrust_key = k
                        break

                if pwm_key is None or thrust_key is None:
                    raise ValueError(
                        "CSV must contain columns like 'PWM (µs)' and 'Force (Kg f)'"
                    )

                pwm = float(row[pwm_key])
                kgf = float(row[thrust_key])

                pwm_us.append(pwm)
                thrust_n.append(kgf * self.kgf_to_N)

        pwm_arr = np.asarray(pwm_us, dtype=float)
        thrust_arr = np.asarray(thrust_n, dtype=float)

        idx = np.argsort(pwm_arr)
        return pwm_arr[idx], thrust_arr[idx]

    def _pwm_us_to_thrust_n(self, pwm_us: float) -> float:
        pwm_clamped = float(np.clip(pwm_us, self.pwm_samples_us[0], self.pwm_samples_us[-1]))
        # self.get_logger().info(f"Mapping PWM {pwm_us} -> {pwm_clamped} us to thrust (N)..")
        # self.get_logger().info(f"Given clamping range [{self.pwm_samples_us[0]}, {self.pwm_samples_us[-1]}] us based on loaded CSV samples..")
        # self.get_logger().info(f"Which is {float(np.interp(pwm_clamped, self.pwm_samples_us, self.thrust_samples_n))} (N)..")
        return float(np.interp(pwm_clamped, self.pwm_samples_us, self.thrust_samples_n))

    def _set_arm(self, arm: bool) -> None:
        self.isArmed = arm
        self.get_logger().info(f"Armed -> {self.isArmed}")
        if not self.isArmed:
            self._send_neutral_all()

    def _set_mode(self, mode: ControlMode) -> None:
        if mode == self._mode:
            return
        self._mode = mode
        if self._mode in (ControlMode.GAMEPAD, ControlMode.GAMEPAD_TWIST):
            self._send_neutral_all()
        self.get_logger().info(f"Switched mode -> {self._mode.value}")

    def _cb_thruster(self, idx: int, msg: SingleDOFStateStamped) -> None:
        value = float(msg.dof_state.output)

        self.last_out[idx] = value
        self.last_time[idx] = self.get_clock().now()

    def _cb_joy(self, msg: Joy) -> None:
        buttons = list(msg.buttons) if msg.buttons is not None else []
        axes = list(msg.axes) if msg.axes is not None else []
        axes_4 = axes[4] if (self.ENABLE_PITCH_CONTROL and len(axes) > 4) else 0.0

        if self._prev_joy_buttons is None:
            self._prev_joy_buttons = buttons
            return

        def rising_edge(btn_idx: int) -> bool:
            if btn_idx < 0:
                return False
            if btn_idx >= len(buttons) or btn_idx >= len(self._prev_joy_buttons):
                return False
            return (self._prev_joy_buttons[btn_idx] == 0) and (buttons[btn_idx] == 1)

        if rising_edge(self._btn_a):
            self._set_mode(ControlMode.GAMEPAD)

        if rising_edge(self._btn_b):
            self._set_mode(ControlMode.AUV_CONTROLLER)

        if rising_edge(self._btn_x):
            self._set_mode(ControlMode.GAMEPAD_TWIST)

        if rising_edge(self._btn_power):
            self._set_arm(not self.isArmed)

        self._prev_joy_buttons = buttons

        if self._mode == ControlMode.GAMEPAD:
            if len(axes) < 6:
                return

            t1 = axes_to_pwm_raw(axes[0], axes[1], axes[3], span=self.GAMEPAD_SPAN)
            t2 = vertical_mix_pwm_us(axes[5], axes[2], axes_4, span=self.GAMEPAD_SPAN)

            pwm_us = t1 + t2
            self.set_pwm_channels_values(self.channels, pwm_us)

            msg_dbg = Float64MultiArray()
            msg_dbg.data = [float(v) for v in pwm_us]
            self.pub_pwm_us.publish(msg_dbg)

        elif self._mode == ControlMode.GAMEPAD_TWIST:
            if len(axes) < 6:
                return

            x_cmd = clamp(-axes[0], -1.0, 1.0)
            y_cmd = clamp(axes[1], -1.0, 1.0)
            yaw_cmd = clamp(-axes[3], -1.0, 1.0)

            x_cmd, y_cmd = normalize_xy(x_cmd, y_cmd)

            t_up = trigger_to_01(axes[5])
            t_down = trigger_to_01(axes[2])
            u_z_up = clamp(t_up - t_down, -1.0, 1.0)
            u_pitch = clamp(axes_4, -1.0, 1.0)

            tw = Twist()
            tw.linear.x = float(y_cmd * self.TWIST_VEL_XY)
            tw.linear.y = float(x_cmd * self.TWIST_VEL_XY)
            tw.linear.z = float((-u_z_up) * self.TWIST_VEL_Z)
            tw.angular.x = 0.0
            tw.angular.y = float(u_pitch * self.TWIST_PITCH_RATE)
            tw.angular.z = float(yaw_cmd * self.TWIST_YAW_RATE)

            self.pub_twist_ref.publish(tw)

    def _send_neutral_all(self) -> None:
        self.set_pwm_channels_values(self.channels, [self.NEUTRAL_US] * self.num_thrusters)

    def set_pwm_channels_values(self, channels: List[int], pwm_values_us: List[float]) -> None:
        if not self.isArmed:
            pwm_values_us = [self.NEUTRAL_US] * self.num_thrusters

        thrust_values_n: List[float] = []

        for i, pwm_us in zip(channels, pwm_values_us):
            if i >= self.num_thrusters:
                continue
            
            # self.get_logger().info(f"Setting channel {i} -> {pwm_us} us PWM..")
            pwm_us_clamped = clamp_float(float(pwm_us), self.MIN_US, self.MAX_US)
            # self.get_logger().info(f"Clamped to {pwm_us_clamped} us PWM..")
            thrust_n = self._pwm_us_to_thrust_n(pwm_us_clamped)
            thrust_values_n.append(thrust_n)

            msg = Float64()
            msg.data = thrust_n
            self.thrust_publishers[i].publish(msg)

        # self.get_logger().info(f"Which is {thrust_values_n} N")

        msg_pwm = Float64MultiArray()
        msg_pwm.data = [float(clamp_float(v, self.MIN_US, self.MAX_US)) for v in pwm_values_us[:self.num_thrusters]]
        self.pub_pwm_us.publish(msg_pwm)

        msg_thrust = Float64MultiArray()
        msg_thrust.data = thrust_values_n
        self.pub_thrust_n.publish(msg_thrust)

    def _on_timer(self) -> None:
        if self._mode == ControlMode.GAMEPAD:
            return

        now = self.get_clock().now()
        pwm_us_values: List[float] = []

        for i in range(self.num_thrusters):
            age_s = (now - self.last_time[i]).nanoseconds * 1e-9
            if age_s > self.TOPIC_TIMEOUT_S:
                pwm_us = self.NEUTRAL_US
                # self.get_logger().info("Time issues")
            else:
                pwm_us = clamp_float(self.last_out[i], self.MIN_US, self.MAX_US)
                # self.get_logger().info(f"Got pwm {pwm_us}..")

            pwm_us_values.append(float(pwm_us))

        self.set_pwm_channels_values(self.channels, pwm_us_values)

    def deinit(self) -> None:
        try:
            self._send_neutral_all()
        except Exception:
            pass


def main() -> None:
    rclpy.init()
    node = Ll_controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected. Shutting down...")
        node.deinit()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()