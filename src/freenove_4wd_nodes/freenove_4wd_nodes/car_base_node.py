import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import UInt8

import freenove_4wd_hw
from freenove_4wd_hw.buzzer import Buzzer
from freenove_4wd_hw.car import Car
from freenove_4wd_hw.led import Led


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class CarBaseNode(Node):
    def __init__(self) -> None:
        super().__init__("freenove_car_base")

        self.declare_parameter("max_duty", 2000)
        self.declare_parameter("duty_scale", 2000.0)
        self.declare_parameter("cmd_vel_timeout_s", 0.5)
        self.declare_parameter("servo_channel", "0")
        self.declare_parameter("sensor_frame_id", "base_link")
        self.declare_parameter("ultrasonic.max_range_m", 3.0)
        self.declare_parameter("ultrasonic.min_range_m", 0.02)
        self.declare_parameter("ultrasonic.field_of_view", 0.5)
        self.declare_parameter("publish.ultrasonic_period_s", 0.2)
        self.declare_parameter("publish.light_period_s", 0.3)
        self.declare_parameter("publish.line_period_s", 0.3)
        self.declare_parameter("publish.battery_period_s", 1.0)
        self.declare_parameter("publish.led_effect_period_s", 0.05)

        self._max_duty = int(self.get_parameter("max_duty").value)
        self._duty_scale = float(self.get_parameter("duty_scale").value)
        self._cmd_vel_timeout_s = float(self.get_parameter("cmd_vel_timeout_s").value)
        self._servo_channel = str(self.get_parameter("servo_channel").value)
        self._sensor_frame_id = str(self.get_parameter("sensor_frame_id").value)
        self._ultra_max_range_m = float(self.get_parameter("ultrasonic.max_range_m").value)
        self._ultra_min_range_m = float(self.get_parameter("ultrasonic.min_range_m").value)
        self._ultra_fov = float(self.get_parameter("ultrasonic.field_of_view").value)

        self._car = Car()
        self._buzzer = Buzzer()
        self._led = Led()

        self._cmd_vel_lock = threading.Lock()
        self._last_cmd_vel_time = 0.0
        self._last_linear_x = 0.0
        self._last_linear_y = 0.0
        self._last_angular_z = 0.0

        self._led_mode_lock = threading.Lock()
        self._led_mode = 0

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Bool, "/buzzer", self._on_buzzer, 10)
        self.create_subscription(Int32, "/servo_angle", self._on_servo_angle, 10)
        self.create_subscription(Int32, "/led_mode", self._on_led_mode, 10)

        self._pub_ultrasonic = self.create_publisher(Range, "/ultrasonic/range", 10)
        self._pub_light_left = self.create_publisher(Float32, "/light/left_voltage", 10)
        self._pub_light_right = self.create_publisher(Float32, "/light/right_voltage", 10)
        self._pub_line = self.create_publisher(UInt8, "/line_sensor", 10)
        self._pub_battery = self.create_publisher(BatteryState, "/battery", 10)

        self.create_timer(float(self.get_parameter("publish.ultrasonic_period_s").value), self._tick_ultrasonic)
        self.create_timer(float(self.get_parameter("publish.light_period_s").value), self._tick_light)
        self.create_timer(float(self.get_parameter("publish.line_period_s").value), self._tick_line)
        self.create_timer(float(self.get_parameter("publish.battery_period_s").value), self._tick_battery)
        self.create_timer(float(self.get_parameter("publish.led_effect_period_s").value), self._tick_led_effect)
        self.create_timer(0.05, self._tick_cmd_vel_watchdog)

    def destroy_node(self) -> bool:
        try:
            self._stop_motors()
        except Exception:
            pass
        try:
            self._buzzer.set_state(False)
        except Exception:
            pass
        try:
            self._car.close()
        except Exception:
            pass
        try:
            self._buzzer.close()
        except Exception:
            pass
        return super().destroy_node()

    def _stop_motors(self) -> None:
        self._car.motor.set_motor_model(0, 0, 0, 0)

    def _on_cmd_vel(self, msg: Twist) -> None:
        now = time.time()
        linear_x = float(msg.linear.x)
        linear_y = float(msg.linear.y)
        angular_z = float(msg.angular.z)

        with self._cmd_vel_lock:
            self._last_cmd_vel_time = now
            self._last_linear_x = linear_x
            self._last_linear_y = linear_y
            self._last_angular_z = angular_z

        self._apply_cmd_vel(linear_x, linear_y, angular_z)

    def _apply_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        front_left = _clamp(linear_x - linear_y - angular_z, -1.0, 1.0)
        front_right = _clamp(linear_x + linear_y + angular_z, -1.0, 1.0)
        rear_left = _clamp(linear_x + linear_y - angular_z, -1.0, 1.0)
        rear_right = _clamp(linear_x - linear_y + angular_z, -1.0, 1.0)

        duty_fl = int(round(_clamp(front_left * self._duty_scale, -self._max_duty, self._max_duty)))
        duty_fr = int(round(_clamp(front_right * self._duty_scale, -self._max_duty, self._max_duty)))
        duty_rl = int(round(_clamp(rear_left * self._duty_scale, -self._max_duty, self._max_duty)))
        duty_rr = int(round(_clamp(rear_right * self._duty_scale, -self._max_duty, self._max_duty)))

        self._car.motor.set_motor_model(duty_fl, duty_rl, duty_fr, duty_rr)

    def _tick_cmd_vel_watchdog(self) -> None:
        now = time.time()
        with self._cmd_vel_lock:
            age = now - self._last_cmd_vel_time
            linear_x = self._last_linear_x
            linear_y = self._last_linear_y
            angular_z = self._last_angular_z

        if self._last_cmd_vel_time == 0.0 or age > self._cmd_vel_timeout_s:
            self._stop_motors()
            return

        self._apply_cmd_vel(linear_x, linear_y, angular_z)

    def _on_buzzer(self, msg: Bool) -> None:
        try:
            self._buzzer.set_state(bool(msg.data))
        except Exception:
            pass

    def _on_servo_angle(self, msg: Int32) -> None:
        try:
            angle = int(msg.data)
            angle = int(_clamp(angle, 0, 180))
            self._car.servo.set_servo_pwm(self._servo_channel, angle)
        except Exception:
            pass

    def _on_led_mode(self, msg: Int32) -> None:
        with self._led_mode_lock:
            self._led_mode = int(msg.data)

    def _tick_ultrasonic(self) -> None:
        distance_cm = None
        try:
            distance_cm = self._car.sonic.get_distance()
        except Exception:
            distance_cm = None

        if distance_cm is None:
            return

        distance_m = float(distance_cm) / 100.0

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._sensor_frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = float(self._ultra_fov)
        msg.min_range = float(self._ultra_min_range_m)
        msg.max_range = float(self._ultra_max_range_m)
        msg.range = float(_clamp(distance_m, msg.min_range, msg.max_range))
        self._pub_ultrasonic.publish(msg)

    def _tick_light(self) -> None:
        try:
            left_v = float(self._car.adc.read_adc(0))
            right_v = float(self._car.adc.read_adc(1))
        except Exception:
            return

        msg_l = Float32()
        msg_l.data = left_v
        self._pub_light_left.publish(msg_l)

        msg_r = Float32()
        msg_r.data = right_v
        self._pub_light_right.publish(msg_r)

    def _tick_line(self) -> None:
        try:
            v = int(self._car.infrared.read_all_infrared())
        except Exception:
            return

        msg = UInt8()
        msg.data = int(_clamp(v, 0, 255))
        self._pub_line.publish(msg)

    def _tick_battery(self) -> None:
        try:
            raw = float(self._car.adc.read_adc(2))
            factor = 3.0 if int(getattr(self._car.adc, "pcb_version", 1)) == 1 else 2.0
            voltage = raw * factor
        except Exception:
            return

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float(voltage)
        self._pub_battery.publish(msg)

    def _tick_led_effect(self) -> None:
        with self._led_mode_lock:
            mode = self._led_mode

        try:
            if mode == 0:
                self._led.colorBlink(0)
            elif mode == 2:
                self._led.following()
            elif mode == 3:
                self._led.colorBlink(1)
            elif mode == 4:
                self._led.rainbowbreathing()
            elif mode == 5:
                self._led.rainbowCycle()
        except Exception:
            pass


def main() -> None:
    rclpy.init()
    node = CarBaseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
