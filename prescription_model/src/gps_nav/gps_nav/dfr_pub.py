#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

# ---- Change this to your presets ----
PRESETS = [
    [130, 130, 130, 130],
    [100, 100, 160, 160],
    [90, 110, 150, 180],
    [170, 140, 130, 100],
]

class DFRTimePublisher(Node):
    def __init__(self):
        super().__init__('dfr_time_publisher')

        # Parameters (override with --ros-args -p name:=value)
        self.declare_parameter('robot', 'robot')
        self.declare_parameter('topic', '')               # if empty, use /{robot}/in/dfr
        self.declare_parameter('period_s', 30.0)          # seconds per preset
        self.declare_parameter('publish_hz', 5.0)         # publish rate
        self.declare_parameter('order', '')               # e.g. "0,1,2,3"

        robot       = self.get_parameter('robot').get_parameter_value().string_value
        topic_param = self.get_parameter('topic').get_parameter_value().string_value
        self.period = max(0.01, float(self.get_parameter('period_s').value))
        pub_hz      = max(0.1, float(self.get_parameter('publish_hz').value))
        order_str   = self.get_parameter('order').get_parameter_value().string_value

        if topic_param:
            topic = topic_param
        else:
            topic = f'/{robot}/in/dfr'

        if order_str.strip():
            try:
                self.order = [int(x) for x in order_str.replace(' ', '').split(',') if x != '']
            except Exception:
                self.get_logger().warn("Bad 'order' param; using default order.")
                self.order = list(range(len(PRESETS)))
        else:
            self.order = list(range(len(PRESETS)))

        if not self.order:
            self.order = [0]

        self.pub = self.create_publisher(Float32MultiArray, topic, 10)
        self.t0 = time.monotonic()
        self.last_idx = None

        self.get_logger().info(
            f"Publishing DFR presets on {topic} | period={self.period}s | order={self.order} | rate={pub_hz} Hz"
        )
        self.timer = self.create_timer(1.0 / pub_hz, self._tick)



    def _make_f32_array(self, data, label):
        msg = Float32MultiArray()
        msg.data = [float(x) for x in data]
        dim = MultiArrayDimension()
        dim.label = label
        dim.size = len(msg.data)
        dim.stride = len(msg.data)
        msg.layout.dim = [dim]
        msg.layout.data_offset = 0
        return msg

    def _tick(self):
        elapsed = time.monotonic() - self.t0
        slot = int(elapsed // self.period) % len(self.order)
        idx = self.order[slot] % len(PRESETS)

        if idx != self.last_idx:
            self.get_logger().info(f"Switching to preset #{idx}: {PRESETS[idx]}")
            self.last_idx = idx

        self.pub.publish(self._make_f32_array(PRESETS[idx],"dfr_in"))


def main():
    rclpy.init()
    node = DFRTimePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
