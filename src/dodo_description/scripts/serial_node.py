#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial, time

class HoverSerial(Node):
    def __init__(self):
        super().__init__('hover_serial')

        # ---- parameters ----
        self.declare_parameter('port', '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_14235333431351619082-if00')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('linear_gain', 250.0)   # m/s -> speed ticks
        self.declare_parameter('angular_gain', 200.0)  # rad/s -> steer ticks
        self.declare_parameter('invert_turn', True)
        # match Arduino parser EXACTLY:
        self.declare_parameter('line_template', 'speed {speed} turn {turn}')
        # Arduino uses readStringUntil('\n'); LF is fine:
        self.declare_parameter('newline', 'lf')
        self.declare_parameter('topic', '/cmd_vel')

        self.port        = self.get_parameter('port').get_parameter_value().string_value
        self.baud        = int(self.get_parameter('baud').value)
        self.lin_gain    = float(self.get_parameter('linear_gain').value)
        self.ang_gain    = float(self.get_parameter('angular_gain').value)
        self.invert_turn = bool(self.get_parameter('invert_turn').value)
        self.template    = str(self.get_parameter('line_template').value)
        self.newline     = str(self.get_parameter('newline').value).lower()
        self.topic_name  = str(self.get_parameter('topic').value)

        self.ser = None
        self._open_serial()

        self.raw_pub = self.create_publisher(String, '/hover/raw_line', 10)
        self.sub = self.create_subscription(Twist, self.topic_name, self.on_cmd, 10)

        self.cached_speed = 0
        self.cached_turn  = 0
        # send at 10 Hz instead of 20 to reduce stress
        self.timer = self.create_timer(0.1, self.keepalive)

        self.send_count = 0

    def _open_serial(self):
        # try to (re)open serial cleanly
        while True:
            try:
                s = serial.Serial(
                    port=self.port,
                    baudrate=self.baud,
                    timeout=0.02,
                    write_timeout=0.2,
                    rtscts=False,
                    dsrdtr=False,
                    xonxoff=False,
                    exclusive=True
                )
                # kill DTR/RTS so Uno doesn't keep resetting
                try:
                    s.dtr = False
                    s.rts = False
                except Exception:
                    pass

                # give Arduino time to boot after port grab
                time.sleep(1.5)
                s.reset_input_buffer()
                s.reset_output_buffer()

                self.ser = s
                self.get_logger().info(f"[hover_serial] Serial OPEN {self.port} @ {self.baud}")
                break
            except Exception as e:
                self.get_logger().error(f"[hover_serial] Serial open failed: {e}")
                time.sleep(1.0)

    def _write_serial(self, line: str):
        if not self.ser:
            self._open_serial()
        payload = (line + ('\n' if self.newline == 'lf' else '\r\n')).encode('ascii', errors='ignore')
        try:
            self.ser.write(payload)
        except Exception as e:
            # EIO / timeout / disconnect -> reopen and retry once
            self.get_logger().warn(f"[hover_serial] serial write failed: {e}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            # reopen
            self._open_serial()
            # retry once
            try:
                self.ser.write(payload)
                self.get_logger().info("[hover_serial] write ok after reopen")
            except Exception as e2:
                self.get_logger().error(f"[hover_serial] retry failed: {e2}")

    def on_cmd(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        speed = int(self.lin_gain * v)
        turn  = int(self.ang_gain * w)
        if self.invert_turn:
            turn = -turn

        self.cached_speed = speed
        self.cached_turn  = turn
        self._send_current()

    def keepalive(self):
        self._send_current()

    def _send_current(self):
        line = self.template.format(speed=self.cached_speed, turn=self.cached_turn)
        self._write_serial(line)

        # Publish debug topic so we can echo in ROS
        self.raw_pub.publish(String(data=line))

        # Log only first ~10 messages to avoid spam
        if self.send_count < 10:
            self.get_logger().info(f'[hover_serial] TX "{line}"')
        self.send_count += 1

def main():
    rclpy.init()
    node = HoverSerial()
    try:
        rclpy.spin(node)
    finally:
        try:
            if node.ser and node.ser.is_open:
                node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
