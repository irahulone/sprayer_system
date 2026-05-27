import os
import json
import configparser
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from serial import Serial
from cobs import cobs
import rclpy
from rclpy.node import Node
import threading, struct
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import time
import numpy as np
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import gps_nav.path as gpath
from gps_nav.path import SpinePath, MultiSegmentPath  # for isinstance checks
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker

CONFIG_FILE = "gps_nav.cfg"


def _quat_from_yaw_np(yaw_rad: float):
    half = 0.5 * float(yaw_rad)
    # yaw-only quaternion: (x,y,z,w) = (0,0,sin(ψ/2),cos(ψ/2))
    return 0.0, 0.0, float(np.sin(half)), float(np.cos(half))
def _yaw_from_north_ccw(b_ccw_deg: float) -> float:
    # 0° (N) → +90° yaw, CCW+; 90° (W) → 180° yaw; 270° (E) → 0° yaw.
    return np.deg2rad((float(b_ccw_deg) + 90.0) % 360.0)
class LowPassFilter:
    """Simple exponential moving average (EMA) filter."""
    def __init__(self, alpha: float, initial_value: float = None):
        """
        :param alpha: smoothing factor in [0,1] (higher = more responsive)
        :param initial_value: if provided, seeds the filter; otherwise
                              first update() call will seed it.
        """
        if not 0.0 <= alpha <= 1.0:
            raise ValueError("alpha must be in [0,1]")
        self.alpha = alpha
        self.value = initial_value

    def update(self, measurement: float) -> float:
        """Ingest a new measurement and return the filtered value."""
        if self.value is None:
            self.value = measurement
        else:
            self.value = self.alpha * measurement + (1 - self.alpha) * self.value
        return self.value

class line_nav(Node):

    def __init__(self):

        config_file_path = os.getcwd()
        config_file_path = os.path.abspath(os.path.join(config_file_path, CONFIG_FILE))  

        nav_cfg = configparser.ConfigParser()
        if not nav_cfg.read(config_file_path):
            print(f"Config file {CONFIG_FILE} not found or invalid.")
            try:
                rclpy.shutdown()
            except Exception:
                pass
            return

        robot_name = nav_cfg['Robot']['name']
        cmd_name = nav_cfg['Robot']['cmd_name']

        gps_topic_name = nav_cfg['Gps']['gps_topic']
        agg_topic = nav_cfg['Gps']['agg_topic']
        heading_topic = nav_cfg['Gps']['heading_topic']

        node_name = nav_cfg['Node']['name']
        queue = int(nav_cfg['Node']['queue_length'])
        self.timer_period = float(nav_cfg['Node']['timer'])

        path_path = os.path.abspath(os.path.join(os.path.dirname(config_file_path),
                                                 nav_cfg['Nav']['path_name']))
        with open(path_path, 'r') as f:
            spec = json.load(f)
        self.path = gpath.SpinePath.from_spec(spec)

        # start & goal


        self.speed_enabled = nav_cfg['Nav']['speed_enabled'].lower() == "true"
        self.linear_fallback = float(nav_cfg['Nav']['speed_fallback'])
        self.speed = float(nav_cfg['Nav']['target_speed'])
        self.max_speed = float(nav_cfg['Nav']['max_speed'])
        self.max_cmd = float(nav_cfg['Nav']['max_cmd'])
        self.z_max = float(nav_cfg['Nav']['z_max'])
        speed_alpha = float(nav_cfg['Nav']['speed_filter'])
        lat_alpha = float(nav_cfg['Nav']['gps_filter'])
        lon_alpha = float(nav_cfg['Nav']['gps_filter'])
        bearing_alpha = float(nav_cfg['Nav']['bearing_filter'])

        # gains & tolerances
        self.k_lin_p     = float(nav_cfg['Gains']['linear_p'])
        self.k_lin_i     = float(nav_cfg['Gains']['linear_i'])
        self.k_ang       = float(nav_cfg['Gains']['angular'])
        self.k_cross     = -float(nav_cfg['Gains']['cross_track'])
        
        self.dist_tol    = float(nav_cfg['Tolerance']['distance'])
        self.ang_tol     = np.deg2rad(float(nav_cfg['Tolerance']['angle']))



        print(f"distance = {self.path.length_m}")
        super().__init__(f'{robot_name}_{node_name}')

        self.speed_filter = LowPassFilter(speed_alpha)
        self.lat_filter = LowPassFilter(lat_alpha)
        self.lon_filter = LowPassFilter(lon_alpha)
        self.bearing_filter = LowPassFilter(bearing_alpha)
        self.curr_speed   = 0.0      # m/s
        self.curr_heading = 0        # degrees
        self.integral    = 0.0
        self.cmd_val = Twist()
        self.lat = 0.0
        self.lon = 0.0

        self.zone_palette = [
            (0.93, 0.27, 0.27),  # red-ish
            (0.26, 0.82, 0.44),  # green
            (0.26, 0.59, 0.98),  # blue
            (0.93, 0.69, 0.22),  # orange
            (0.70, 0.33, 0.83),  # purple
        ]


        self.sprayer_vals = [[130,130,130,130],[100,100,160,160],[90,110,150,180],[170,140,130,100]]
        self.zone_map = None
        self._init_zone_mapping(nav_cfg)

        # --- ROS interfaces ---
        self.cmd_pub = self.create_publisher(Twist, f"/{robot_name}/{cmd_name}", queue)
        self.gps_sub = self.create_subscription(NavSatFix, f'/{robot_name}/{agg_topic}', self.gps_cb, queue)
        self.vel_sub = self.create_subscription(String, f'/{robot_name}/{agg_topic}/Diagnostic', self.vel_cb, queue)
        self.heading_sub = self.create_subscription(Float32, f'/{robot_name}/{heading_topic}', self.heading_cb, queue)
        self.dfr_pub  = self.create_publisher(Float32MultiArray, f"/{robot_name}/in/dfr", queue)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.setup_viz_publishers()

        self.publish_static_path()

    def setup_viz_publishers(self):
        latched = QoSProfile(depth=1)
        latched.history = HistoryPolicy.KEEP_LAST
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        volatile = QoSProfile(depth=1)
        volatile.history = HistoryPolicy.KEEP_LAST
        volatile.reliability = ReliabilityPolicy.RELIABLE
        volatile.durability = DurabilityPolicy.VOLATILE
        self.marker_pub = self.create_publisher(Marker, "/viz/path_marker", volatile)

        self.path_pub   = self.create_publisher(Path,  "/viz/path",        latched)
        self.pose_pub   = self.create_publisher(PoseStamped, "/viz/pose",  10)
        self.traj_pub   = self.create_publisher(Path, "/viz/traj_path", latched)
        self.traj_path = Path(); self.traj_path.header.frame_id = "map"
        self.traj_max_points = 1200     # cap history
        self.traj_step_m = 0.05         # add a point every ~25 cm


    def _traj_maybe_add(self, x, y):
        # only add if moved far enough from last point
        if not self.traj_path.poses:
            add = True
        else:
            last = self.traj_path.poses[-1].pose.position
            dx = float(x) - last.x; dy = float(y) - last.y
            add = (dx*dx + dy*dy) >= (self.traj_step_m * self.traj_step_m)

        if add:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = float(x); ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            self.traj_path.poses.append(ps)
            if len(self.traj_path.poses) > self.traj_max_points:
                self.traj_path.poses = self.traj_path.poses[-self.traj_max_points:]
            self.traj_path.header.stamp = self.get_clock().now().to_msg()
            self.traj_pub.publish(self.traj_path)

    def _zone_runs_from_spine(self):
        """
        Returns list of (zone_idx, XY_chunk) where XY_chunk is an (N,2) array
        of contiguous points that share the same zone.
        """
        if not isinstance(self.path, SpinePath):
            return []

        if not hasattr(self, "pts_xy") or self.pts_xy is None or len(self.pts_xy) == 0:
            return []

        ids = self.path.seg_ids_per_point
        zones_per_point = [int(self.zone_map.get(self._norm_id(pid), 0)) for pid in ids]

        runs = []
        start = 0
        for i in range(1, len(zones_per_point) + 1):
            if i == len(zones_per_point) or zones_per_point[i] != zones_per_point[start]:
                z = zones_per_point[start]
                chunk = self.pts_xy[start:i].copy()
                if len(chunk) >= 2:                          # need at least two points for a strip
                    runs.append((z, chunk))
                start = i
        return runs

    def publish_static_path(self):
        # build XY (meters) along your path
        if hasattr(self.path, "xy"):
            self.pts_xy = self.path.xy
            lat0, lon0 = self.path.lat0, self.path.lon0
        else:
            gj = path_to_geojson(self.path, step_m=2.0, split_by_segments=False)
            coords = gj["features"][0]["geometry"]["coordinates"]  # [lon,lat]
            lat0, lon0 = coords[0][1], coords[0][0]
            self.pts_xy = np.array([gpath._latlon_to_xy_m(lat, lon, lat0, lon0)
                               for lon, lat in coords], dtype=float)

        # nav_msgs/Path
        pmsg = Path(); pmsg.header.frame_id = "map"
        for x, y in self.pts_xy:
            ps = PoseStamped(); ps.header.frame_id = "map"
            ps.pose.position.x = float(x); ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            pmsg.poses.append(ps)
        self.path_pub.publish(pmsg)
        self.path_marker()
        self.segmented_path_markers()


    def segmented_path_markers(self):
        runs = self._zone_runs_from_spine()
        if not runs:
            return

        for i, (zone_idx, chunk) in enumerate(runs):
            m = Marker()
            m.header.frame_id = "map"; m.ns = "zones"; m.id = i
            m.type = Marker.LINE_STRIP; m.action = Marker.ADD
            m.scale.x = 0.05
            m.color.a = 1.0
            r, g, b = self.zone_palette[zone_idx % len(self.zone_palette)]
            m.color.r, m.color.g, m.color.b = float(r), float(g), float(b)
            for x, y in chunk:
                p = Point(); p.x = float(x); p.y = float(y); p.z = 0.1
                m.points.append(p)
            self.marker_pub.publish(m)

    def path_marker(self):
        m = Marker()
        m.header.frame_id = "map"; m.ns = "path"; m.id = 0
        m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.scale.x = 0.05
        m.color.a = 1.0; m.color.r = 0.1; m.color.g = 0.7; m.color.b = 1.0
        for x, y in self.pts_xy:
            pt = Point(); pt.x = float(x); pt.y = float(y); pt.z = 0.0
            m.points.append(pt)
        self.marker_pub.publish(m)



    def publish_live_pose(self):
        x, y = gpath._latlon_to_xy_m(self.lat, self.lon, self.path.lat0, self.path.lon0)
        yaw = _yaw_from_north_ccw(self.curr_heading)
        qx, qy, qz, qw = _quat_from_yaw_np(yaw)

        self._traj_maybe_add(x, y)

        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x); msg.pose.position.y = float(y)
        msg.pose.orientation.x = qx; msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz; msg.pose.orientation.w = qw
        self.pose_pub.publish(msg)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def goal_reached(self):
        return self.path.distance_to_goal(self.lat, self.lon) < self.dist_tol

    def _init_zone_mapping(self, nav_cfg):
        # 1) read Zones from cfg (case-insensitive keys)
        cfg_map = {}
        if nav_cfg.has_section('Zones'):
            for k, v in nav_cfg['Zones'].items():
                try:
                    cfg_map[self._norm_id(k)] = int(v)
                except Exception:
                    pass

        # 2) derive ids in order from the built path (normalized)
        ids_in_order = []
        if isinstance(self.path, SpinePath):
            for p in self.path.list_pieces():
                pid = self._norm_id(p.get('id'))
                if pid is not None and pid not in ids_in_order:
                    ids_in_order.append(pid)
        elif isinstance(self.path, MultiSegmentPath):
            for s in self.path.segs:
                pid = self._norm_id(getattr(s, 'id', None))
                if pid is not None and pid not in ids_in_order:
                    ids_in_order.append(pid)
        else:
            pid = self._norm_id(getattr(self.path, 'id', None))
            if pid is not None:
                ids_in_order = [pid]

        # 3) start from cfg_map, then assign any missing ids in order
        mapping = dict(cfg_map)
        next_idx = len(mapping)
        for pid in ids_in_order:
            if pid not in mapping:
                mapping[pid] = next_idx
                next_idx += 1

        # 4) clamp to available zones
        nz = len(self.sprayer_vals)
        self.zone_map = {pid: int(z % nz) for pid, z in mapping.items()}

        # debug
        self.get_logger().info(f"Zone map (final): {self.zone_map}")


    def _norm_id(self, s):
        return str(s).strip().lower() if s is not None else None


    def compute_zone(self, lat, lon) -> int:
        try:
            if isinstance(self.path, SpinePath):
                pid = self.path.active_piece_id(lat, lon)
            elif isinstance(self.path, MultiSegmentPath):
                pid = self.path.active_segment_id(lat, lon)
            else:
                pid = getattr(self.path, 'id', None)
        except Exception:
            pid = getattr(self.path, 'id', None)
        return int(self.zone_map.get(self._norm_id(pid), 0))


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


    def timer_callback(self):
        zone = self.compute_zone(self.lat, self.lon)
        
        self.dfr_pub.publish(self._make_f32_array(self.sprayer_vals[zone],"dfr_in"))
        self.publish_live_pose()
        #self.segmented_path_markers()
        #self.ser.flush()

        if not self.speed_enabled:
            self.cmd_val.linear.x = self.linear_fallback
        else:
            err = self.speed - self.curr_speed
            # update integral with anti-windup guard
            self.integral += err * self.timer_period
            # Optionally clamp integral to prevent runaway:
            int_max = self.max_cmd / max(self.k_lin_i,1e-6)
            self.integral = float(np.clip(self.integral, -int_max, int_max))
            # feed-forward term (normalized)
            u_ff = (self.speed / self.max_speed) * self.max_cmd
            # PI law
            u = u_ff + self.k_lin_p * err + self.k_lin_i * self.integral
            # saturate
            u_sat = float(np.clip(u, 0.0, self.max_cmd))
            # if we saturated, back out any integral beyond the bound
            if u != u_sat:
                # simple anti-windup: undo last integral bump
                self.integral -= err * self.timer_period
            self.cmd_val.linear.x = u_sat if not self.goal_reached() else 0.0       

        #print(f"here be the dfrs {self.sprayer_vals[zone]}")
        #print(f"Distance be {gpath._haversine_m(self.lat,self.lon,self.target_lat,self.target_lon)}")
        self.cmd_pub.publish(self.cmd_val)


    def heading_cb(self, msg: Float32):
        self.curr_heading = self.bearing_filter.update(msg.data)

    def vel_cb(self, msg: String):
        """Parse the JSON diagnostic from read_gps_data to get speed & heading."""
        try:
            data = json.loads(msg.data)
            #self.get_logger().info(f"vel recieved: {data}")
            vel = data.get("velocity", {}).get("ground_speed_mps", None)
            if vel is not None:
                self.get_logger().info(f"vel: {vel}")
                self.curr_speed = self.speed_filter.update(float(vel))
            else:
                self.get_logger().warn("No vel, reverting to fallback")
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warn("Failed to parse GPS diagnostic JSON")

    def gps_cb(self, msg: NavSatFix):
        self.lat = self.lat_filter.update(msg.latitude)
        self.lon = self.lon_filter.update(msg.longitude)

        desired_heading = self.path.heading_at(self.lat, self.lon)  # deg
        heading_err = gpath._wrap_pi(np.deg2rad(desired_heading - self.curr_heading))
        heading_err = self.z_max if heading_err > self.z_max else heading_err
        heading_err = -self.z_max if heading_err < -self.z_max else heading_err

        xt_err = self.path.cross_track_error(self.lat, self.lon)
        #print(f"x err: {xt_err}")# meters; +right/-left
        self.cmd_val.angular.z = self.k_ang * (heading_err + self.k_cross * xt_err)

def main(args=None):
    rclpy.init(args=args)

    pub = line_nav()

    try:
        rclpy.spin(pub)
    except KeyboardInterrupt:
        pub.get_logger().info("Shutting down diff_gps node...")
    finally:
        pub.destroy_node()
        if rclpy.ok():  # Only shutdown if ROS2 is still active
            rclpy.shutdown()

if __name__ == '__main__':
    main()
