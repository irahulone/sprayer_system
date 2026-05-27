#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import gps_nav.path as gpath  # uses _xy_to_latlon_m, _xy_to_bearing_deg

class SimpleGpsHeadingSim(Node):
    def __init__(self):
        super().__init__("simple_gps_heading_sim")

        # --- Declare params (defaults are sane) ---
        self.declare_parameter('robot_name', 'Kanda')
        self.declare_parameter('agg_topic', 'agg')
        self.declare_parameter('heading_topic', 'heading')
        self.declare_parameter('lat0', 37.26022904999999)
        self.declare_parameter('lon0', -121.8426278)

        self.declare_parameter('dt', 0.1)             # s
        self.declare_parameter('speed_mps', 0.8)      # m/s

        self.declare_parameter('motion', 'circle')    # circle | line | eight
        self.declare_parameter('radius_m', 10.0)      # for circle
        self.declare_parameter('line_length_m', 20.0) # for line
        self.declare_parameter('loop', True)          # for line
        self.declare_parameter('eight_width_m', 24.0) # for figure-8 (overall width)

        # optional noise knobs
        self.declare_parameter('gps_noise_m', 0.0)
        self.declare_parameter('yaw_noise_deg', 0.0)

        # --- Read params (do this BEFORE motion-specific math!) ---
        self.robot_name = str(self.get_parameter('robot_name').value)
        self.agg_topic = str(self.get_parameter('agg_topic').value)
        self.heading_topic = str(self.get_parameter('heading_topic').value)
        self.lat0 = float(self.get_parameter('lat0').value)
        self.lon0 = float(self.get_parameter('lon0').value)

        self.dt = float(self.get_parameter('dt').value)
        self.speed = float(self.get_parameter('speed_mps').value)

        self.motion = str(self.get_parameter('motion').value).lower()
        self.radius = float(self.get_parameter('radius_m').value)
        self.line_len = float(self.get_parameter('line_length_m').value)
        self.loop = bool(self.get_parameter('loop').value)
        self.eight_width = float(self.get_parameter('eight_width_m').value)

        self.gps_noise = float(self.get_parameter('gps_noise_m').value)
        self.yaw_noise_deg = float(self.get_parameter('yaw_noise_deg').value)

        # --- Publishers ---
        self.gps_pub = self.create_publisher(
            NavSatFix, f"/{self.robot_name}/{self.agg_topic}", 10)
        self.hdg_pub = self.create_publisher(
            Float32, f"/{self.robot_name}/{self.heading_topic}", 10)

        # --- Initialize trajectory state ---
        if self.motion == 'circle':
            self.radius = max(self.radius, 1e-6)
            self.omega = self.speed / self.radius
            self.t = 0.0
            self.x, self.y = self.radius, 0.0

        elif self.motion == 'line':
            self.x = -self.line_len / 2.0
            self.y = 0.0
            self.dir = 1.0  # forward along +x

        elif self.motion == 'eight':
            # Gerono lemniscate: x=a*sin(t), y=(a/2)*sin(2t)
            self.a = max(self.eight_width * 0.5, 1e-6)
            # crude mapping so tangential speed ~ speed
            self.omega = self.speed / self.a
            self.t = 0.0
            self.x = self.a * np.sin(self.t)
            self.y = 0.5 * self.a * np.sin(2.0 * self.t)

        else:
            self.get_logger().warn(f"Unknown motion '{self.motion}', defaulting to circle.")
            self.motion = 'circle'
            self.radius = max(10.0, 1e-6)
            self.omega = self.speed / self.radius
            self.t = 0.0
            self.x, self.y = self.radius, 0.0

        self.prev_x, self.prev_y = self.x, self.y

        # --- Timer ---
        self.timer = self.create_timer(self.dt, self.step)

    def step(self):
        self.prev_x, self.prev_y = self.x, self.y

        if self.motion == 'circle':
            self.t += self.omega * self.dt
            self.x = self.radius * np.cos(self.t)
            self.y = self.radius * np.sin(self.t)

        elif self.motion == 'line':
            self.x += self.dir * self.speed * self.dt
            half = self.line_len / 2.0
            if self.loop:
                if self.x >  half: self.x = -half
                if self.x < -half: self.x =  half
            else:
                if self.x >  half: self.x, self.dir =  half, -1.0
                if self.x < -half: self.x, self.dir = -half,  1.0

        else:  # 'eight'
            self.t += self.omega * self.dt
            self.x = self.a * np.sin(self.t)
            self.y = 0.5 * self.a * np.sin(2.0 * self.t)

        # heading from finite difference
        dx = self.x - self.prev_x
        dy = self.y - self.prev_y
        if abs(dx) + abs(dy) < 1e-12:
            dy = 1e-12
        heading_deg = gpath._xy_to_bearing_deg(dx, dy)

        # noise (optional)
        if self.yaw_noise_deg != 0.0:
            heading_deg = float(heading_deg + np.random.normal(0.0, self.yaw_noise_deg))
        gx, gy = self.x, self.y
        if self.gps_noise != 0.0:
            gx += np.random.normal(0.0, self.gps_noise)
            gy += np.random.normal(0.0, self.gps_noise)

        # ENU -> lat/lon around (lat0,lon0)
        lat, lon = gpath._xy_to_latlon_m(gx, gy, self.lat0, self.lon0)

        # publish GPS
        gps = NavSatFix()
        gps.header.stamp = self.get_clock().now().to_msg()
        gps.header.frame_id = 'map'
        gps.latitude = float(lat)
        gps.longitude = float(lon)
        gps.altitude = 0.0
        gps.position_covariance = [0.0] * 9
        gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.gps_pub.publish(gps)

        # publish heading
        h = Float32(); h.data = float(heading_deg)
        self.hdg_pub.publish(h)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleGpsHeadingSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
