import os
import math
import json
import time
import threading
import configparser
from datetime import datetime, timezone
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32, String
from builtin_interfaces.msg import Time as TimeMsg

import serial
from serial.tools import list_ports
import numpy as np

from diff_gps.ublox_gps import UbloxGps

CONFIG_FILE = "gps_config.cfg"


# ---------- Small windowed filters ----------
class MovingAverage:
    def __init__(self, size: int):
        assert size >= 1
        self.size = int(size)
        self.buf  = deque()
        self.sum  = 0.0
    def reset(self):
        self.buf.clear(); self.sum = 0.0
    def update(self, x):
        if x is None or isinstance(x, (float, np.floating)) and (math.isnan(float(x)) or np.isnan(x)):
            return self.value()
        x = float(x)
        if len(self.buf) == self.size:
            self.sum -= self.buf.popleft()
        self.buf.append(x)
        self.sum += x
        return self.value()
    def value(self):
        return (self.sum / len(self.buf)) if self.buf else None


class CircularMovingAverageDeg:
    def __init__(self, size: int):
        assert size >= 1
        self.size = int(size)
        self.buf  = deque()          # (sin, cos)
        self.sum_s = 0.0
        self.sum_c = 0.0
    def reset(self):
        self.buf.clear(); self.sum_s = 0.0; self.sum_c = 0.0
    def update(self, ang_deg):
        if ang_deg is None or (isinstance(ang_deg, float) and math.isnan(ang_deg)):
            return self.value()
        r = math.radians(float(ang_deg))
        s, c = math.sin(r), math.cos(r)
        if len(self.buf) == self.size:
            os_, oc_ = self.buf.popleft()
            self.sum_s -= os_; self.sum_c -= oc_
        self.buf.append((s, c))
        self.sum_s += s; self.sum_c += c
        return self.value()
    def value(self):
        if not self.buf:
            return None
        mean = math.degrees(math.atan2(self.sum_s, self.sum_c))
        return mean + 360.0 if mean < 0.0 else mean

class MovingAverageVec:
    """Vector sliding-window mean (e.g., x,y,z ENU)."""
    def __init__(self, size: int, dim: int):
        assert size >= 1 and dim >= 1
        self.size = int(size)
        self.buf  = deque()
        self.sum  = np.zeros(dim, dtype=float)

    def reset(self):
        self.buf.clear(); self.sum[:] = 0.0

    def update(self, x):
        v = np.asarray(x, dtype=float)
        if len(self.buf) == self.size:
            self.sum -= self.buf.popleft()
        self.buf.append(v)
        self.sum += v
        return self.sum / len(self.buf)



# ---------- Node ----------
class read_gps_data(Node):

    def __init__(self):
        # Load config
        cfg_path = os.path.abspath(os.path.join(os.getcwd(), CONFIG_FILE))
        self.config = configparser.ConfigParser()
        if not self.config.read(cfg_path):
            print(f"Config file {CONFIG_FILE} not found or invalid.")
            try: rclpy.shutdown()
            except Exception: pass
            return

        robot_name = self.config['Robot']['name']

        gps_topic_name     = self.config['Gps']['gps_topic']
        gps_agg_name       = self.config['Gps']['agg_topic']
        gps_heading_name   = self.config['Gps']['heading_topic']
        self.gps_left_antenna  = self.config['Gps']['left']
        self.gps_right_antenna = self.config['Gps']['right']

        node_name   = self.config['Node']['name']
        queue_len   = int(self.config['Node']['queue_length'])
        timer_period = float(self.config['Node']['gps_timer_period'])
        self.gps     = self.config['Node']['gps'].lower() == "true"
        self.diff    = self.config['Node']['diff'].lower() == "true"
        self.heading = self.config['Node']['heading'].lower() == "true"

        self._last_diag_pub_sec = 0.0
        self._diag_period = float(self.config['Gps']['diag_period'])

        # Polling rates (configurable)
        self.coords_hz   = float(self.config['Gps'].get('coords_hz', '10'))
        self.dop_hz      = float(self.config['Gps'].get('dop_hz', '2'))
        self.coords_per  = (1.0 / self.coords_hz) if self.coords_hz > 0 else None
        self.dop_per     = (1.0 / self.dop_hz)    if self.dop_hz > 0 else None

        # Global epoch to align both side threads to system time boundaries
        self._sched_epoch = math.floor(time.time())  # align to next whole-second grid

        # USB VID:PID pair (e.g. "0x1546:0x01a9")
        self.gps_port_ids = list(map(lambda s: int(s.strip(), 0),
                                     self.config['Ublox']['port_ids'].split(':')))

        super().__init__(f'{robot_name}_{node_name}')

        # Publishers
        if self.gps:
            self.publisher_gps1_satnav = self.create_publisher(NavSatFix, f'/{robot_name}/{gps_topic_name}1', queue_len)
            self.publisher_gps2_satnav = self.create_publisher(NavSatFix, f'/{robot_name}/{gps_topic_name}2', queue_len)
            self.publisher_gps1_json   = self.create_publisher(String,     f'/{robot_name}/{gps_topic_name}1/Diagnostic', queue_len)
            self.publisher_gps2_json   = self.create_publisher(String,     f'/{robot_name}/{gps_topic_name}2/Diagnostic', queue_len)
        if self.diff:
            self.publisher_gps_agg      = self.create_publisher(NavSatFix, f'/{robot_name}/{gps_agg_name}', queue_len)
            self.publisher_gps_agg_json = self.create_publisher(String,    f'/{robot_name}/{gps_agg_name}/Diagnostic', queue_len)
        if self.heading:
            self.publisher_brng = self.create_publisher(Float32, f'/{robot_name}/{gps_heading_name}', queue_len)

        # State
        self.state_lock = threading.Lock()
        self.state = {
            'left':  {'coords': None, 'dop': None, 'stamp_ros': None, 'key_ms': None},
            'right': {'coords': None, 'dop': None, 'stamp_ros': None, 'key_ms': None},
        }
        self.last_pub_key = {'left': None, 'right': None}

        # Heading filter (optional)
        self.heading_filter = CircularMovingAverageDeg(int(self.config['Gps'].get('heading_window', '5')))
        self.speed_filter   = MovingAverage(int(self.config['Gps'].get('speed_window', '5')))
        # Position (ENU) filter for aggregated position
        self.pos_filter_enabled = self.config['Gps'].get('pos_filter_enabled', 'true').lower() == 'true'
        self.pos_window = int(self.config['Gps'].get('pos_window', '5'))
        self.pos_filter = MovingAverageVec(self.pos_window, 3) if self.pos_filter_enabled else None
        self.pos_origin = None     # (lat0_rad, lon0_rad, alt0_m)
        self.cos_lat0   = None     # precompute cos(lat0) for xy conversion

        # UBX handles & ports
        self.gps_left  = None
        self.gps_right = None
        self.left_port  = None
        self.right_port = None

        # Sync buffers & tolerance
        self.sync_lock    = threading.Lock()
        self.left_buf     = deque(maxlen=20)
        self.right_buf    = deque(maxlen=20)
        self.sync_skew_ms = float(self.config['Gps'].get('sync_skew_ms', '75'))

        # Open devices
        self.open_gps()

        # Threads: independent per side, aligned to system time
        self._stop_evt = threading.Event()
        self.left_thread  = threading.Thread(target=self._side_loop,  args=('left',),  daemon=True)
        self.right_thread = threading.Thread(target=self._side_loop,  args=('right',), daemon=True)
        self.left_thread.start()
        self.right_thread.start()

        # ROS timer
        self.timer = self.create_timer(timer_period, self.gps_callback)

    def _ensure_pos_origin(self, lat_deg, lon_deg, alt_m):
        """Set ENU linearization origin once (first good agg fix)."""
        if self.pos_origin is None:
            lat0 = math.radians(float(lat_deg))
            lon0 = math.radians(float(lon_deg))
            self.pos_origin = (lat0, lon0, float(alt_m))
            self.cos_lat0 = math.cos(lat0)

    def _llh_to_xy(self, lat_deg, lon_deg):
        """Approximate local tangent plane (equirectangular) — good for local (<~10 km)."""
        R = 6378137.0  # WGS-84 a (m)
        lat = math.radians(float(lat_deg))
        lon = math.radians(float(lon_deg))
        lat0, lon0, _ = self.pos_origin
        x = R * (lon - lon0) * self.cos_lat0
        y = R * (lat - lat0)
        return x, y
    
    def _xy_to_llh(self, x, y):
        R = 6378137.0
        lat0, lon0, _ = self.pos_origin
        lat = lat0 + (y / R)
        lon = lon0 + (x / (R * self.cos_lat0))
        return math.degrees(lat), math.degrees(lon)

    # ---------- Scheduling helpers ----------
    def _aligned_next_due(self, period):
        """Return next due time (absolute, seconds) aligned to global epoch and given period."""
        if period is None or period <= 0.0:
            return None
        now = time.time()
        k = math.ceil((now - self._sched_epoch) / period)
        return self._sched_epoch + k * period

    def _advance_due(self, nxt, period, now=None):
        if nxt is None or period is None:
            return None
        if now is None:
            now = time.time()
        # ensure nxt is strictly in the future (catch-up if we fell behind)
        if now >= nxt:
            steps = max(1, math.floor((now - nxt) / period) + 1)
            nxt += steps * period
        return nxt

    def aggregate_gps(self, left_state, right_state, do_diag=False):
        """Fuse left/right fixes into one NavSatFix; filter in ENU; publish + diagnostics."""
        l = left_state['coords']; r = right_state['coords']
        if l is None or r is None:
            return
    
        # Choose the newer stamp for header
        stamp_left  = left_state.get('stamp_ros')
        stamp_right = right_state.get('stamp_ros')
        stamp = stamp_left if stamp_right is None else (
                stamp_right if stamp_left is None else
                (stamp_left if (stamp_left.sec, stamp_left.nanosec) >= (stamp_right.sec, stamp_right.nanosec) else stamp_right)
        )
    
        # ---------------- (A) Weighted fusion in ENU ----------------
        # Horizontal weights from hAcc if available (variance ~ hAcc^2)
        def _w_from_hacc(hacc_mm):
            if hacc_mm is None:
                return 1.0
            h = float(hacc_mm) / 1000.0
            return 1.0 / max(h*h, 1e-6)
    
        # Vertical weights from vAcc if available
        def _w_from_vacc(vacc_mm):
            if vacc_mm is None:
                return 1.0
            v = float(vacc_mm) / 1000.0
            return 1.0 / max(v*v, 1e-6)
    
        wL = _w_from_hacc(getattr(l, 'hAcc', None))
        wR = _w_from_hacc(getattr(r, 'hAcc', None))
        wS = wL + wR if (wL + wR) > 0 else 1.0
    
        latL, lonL = float(l.lat), float(l.lon)
        latR, lonR = float(r.lat), float(r.lon)
        altL = (float(getattr(l, 'height', 0.0))/1000.0) if hasattr(l, 'height') else None
        altR = (float(getattr(r, 'height', 0.0))/1000.0) if hasattr(r, 'height') else None
    
        # Seed origin once using rough mid-point
        seed_lat = 0.5 * (latL + latR)
        seed_lon = 0.5 * (lonL + lonR)
        seed_alt = 0.5 * ((altL or 0.0) + (altR or 0.0))
        self._ensure_pos_origin(seed_lat, seed_lon, seed_alt)
    
        xL, yL = self._llh_to_xy(latL, lonL)
        xR, yR = self._llh_to_xy(latR, lonR)
    
        # Weighted average in meters
        x_raw = (wL*xL + wR*xR) / wS
        y_raw = (wL*yL + wR*yR) / wS
    
        wLz = _w_from_vacc(getattr(l, 'vAcc', None))
        wRz = _w_from_vacc(getattr(r, 'vAcc', None))
        z_raw = None
        if (altL is not None) or (altR is not None):
            z_raw = ((wLz*(altL or 0.0)) + (wRz*(altR or 0.0))) / max(wLz + wRz, 1e-6)
    
        # ---------------- (B) Optional ENU smoothing ----------------
        if self.pos_filter is not None:
            vec = np.array([x_raw, y_raw, z_raw if z_raw is not None else 0.0], dtype=float)
            x_f, y_f, z_f = self.pos_filter.update(vec)
        else:
            x_f, y_f, z_f = x_raw, y_raw, (z_raw if z_raw is not None else 0.0)
    
        # Convert back to LLH
        lat_f, lon_f = self._xy_to_llh(x_f, y_f)
    
        # ---------------- (C) Build & publish NavSatFix -------------
        msg_agg = NavSatFix()
        msg_agg.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        msg_agg.header.frame_id = "gps_agg"
    
        msg_agg.latitude  = lat_f
        msg_agg.longitude = lon_f
        msg_agg.altitude  = z_f
    
        # Status: FIX only if both have 3D fix
        fix_ok = (getattr(l, 'fixType', 0) >= 3 and getattr(r, 'fixType', 0) >= 3)
        msg_agg.status.status  = NavSatStatus.STATUS_FIX if fix_ok else NavSatStatus.STATUS_NO_FIX
        msg_agg.status.service = NavSatStatus.SERVICE_GPS
    
        # Covariance: use averaged hAcc/vAcc (meters) as before
        if hasattr(l, 'hAcc') and hasattr(r, 'hAcc') and hasattr(l, 'vAcc') and hasattr(r, 'vAcc'):
            hacc_avg = ((float(l.hAcc) + float(r.hAcc)) / 2.0) / 1000.0
            vacc_avg = ((float(l.vAcc) + float(r.vAcc)) / 2.0) / 1000.0
            msg_agg.position_covariance = [
                hacc_avg**2, 0.0,        0.0,
                0.0,         hacc_avg**2,0.0,
                0.0,         0.0,        vacc_avg**2
            ]
            msg_agg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        else:
            msg_agg.position_covariance = [0.0]*9
            msg_agg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
    
        self.publisher_gps_agg.publish(msg_agg)
    
        # ---------------- (D) Diagnostics JSON ----------------------
        if do_diag:
            debug = {
                "stamp_sec": msg_agg.header.stamp.sec + msg_agg.header.stamp.nanosec*1e-9,
                "position_filtered": {"lat": msg_agg.latitude, "lon": msg_agg.longitude, "alt_m": msg_agg.altitude},
                "position_raw_xy_m": {"x": x_raw, "y": y_raw},
                "fix": {"left": getattr(l, 'fixType', None), "right": getattr(r, 'fixType', None), "bothFix": int(fix_ok)},
                "satellites": {"left": getattr(l, 'numSV', None), "right": getattr(r, 'numSV', None)}
            }
    
            if hasattr(l, 'hAcc') and hasattr(r, 'hAcc'):
                debug["accuracy_h_m"] = ((float(l.hAcc) + float(r.hAcc))/2.0)/1000.0
            if hasattr(l, 'vAcc') and hasattr(r, 'vAcc'):
                debug["accuracy_v_m"] = ((float(l.vAcc) + float(r.vAcc))/2.0)/1000.0
    
            # Ground speed (m/s), filtered
            gs_raw = None
            if hasattr(l, 'gSpeed') and hasattr(r, 'gSpeed') and (l.gSpeed is not None) and (r.gSpeed is not None):
                gs_raw = 0.5 * (float(l.gSpeed) + float(r.gSpeed)) / 1000.0  # mm/s -> m/s
            elif hasattr(l, 'gSpeed') and (l.gSpeed is not None):
                gs_raw = float(l.gSpeed) / 1000.0
            elif hasattr(r, 'gSpeed') and (r.gSpeed is not None):
                gs_raw = float(r.gSpeed) / 1000.0
    
            if gs_raw is not None:
                gs_f = self.speed_filter.update(gs_raw)
                debug["velocity"] = {
                    "ground_speed_mps": gs_f,
                    "raw_ground_speed_mps": gs_raw
                }
    
            msg = String(); msg.data = json.dumps(debug)
            gs = None
            if hasattr(l, 'gSpeed') and hasattr(r, 'gSpeed') and (l.gSpeed is not None) and (r.gSpeed is not None):
                gs = 0.5 * (float(l.gSpeed) + float(r.gSpeed)) / 1000.0  # mm/s -> m/s
            elif hasattr(l, 'gSpeed') and (l.gSpeed is not None):
                gs = float(l.gSpeed) / 1000.0
            elif hasattr(r, 'gSpeed') and (r.gSpeed is not None):
                gs = float(r.gSpeed) / 1000.0

            if gs is not None:
                debug["velocity"] = {"ground_speed_mps": gs}
            msg = String()
            msg.data = json.dumps(debug)
            self.publisher_gps_agg_json.publish(msg)

    
    # ---------- Time & key helpers ----------
    def _gps_stamp_to_ros_time(self, coords):
        try:
            dt = datetime(coords.year, coords.month, coords.day,
                          coords.hour, coords.min, coords.sec,
                          tzinfo=timezone.utc)
            sec  = int(dt.timestamp())
            nsec = int(coords.nano) if hasattr(coords, 'nano') else 0
            nsec = 0 if nsec < 0 else (999_999_999 if nsec >= 1_000_000_000 else nsec)
            t = TimeMsg(); t.sec = sec; t.nanosec = nsec
            return t
        except Exception:
            return self.get_clock().now().to_msg()

    def _key_from_coords(self, coords, stamp_ros):
        itow = getattr(coords, 'iTOW', None)
        if itow is not None:
            try: return int(itow)
            except Exception: pass
        return int(stamp_ros.sec * 1000 + stamp_ros.nanosec / 1e6)

    def _pop_synced_pair(self):
        with self.sync_lock:
            if not self.left_buf or not self.right_buf:
                return None
            while self.left_buf and self.right_buf:
                kl, cl, sl = self.left_buf[0]
                kr, cr, sr = self.right_buf[0]
                diff = kl - kr
                if abs(diff) <= self.sync_skew_ms:
                    self.left_buf.popleft()
                    self.right_buf.popleft()
                    stamp = sl if (sl.sec, sl.nanosec) >= (sr.sec, sr.nanosec) else sr
                    return (cl, cr, stamp)
                if diff < -self.sync_skew_ms:
                    self.left_buf.popleft()
                elif diff > self.sync_skew_ms:
                    self.right_buf.popleft()
            return None

    # ---------- Side worker loop (independent, aligned) ----------
    def _side_loop(self, side: str):
        left = (side == 'left')
        next_pvt = self._aligned_next_due(self.coords_per)
        next_dop = self._aligned_next_due(self.dop_per)

        while not self._stop_evt.is_set() and rclpy.ok():
            now = time.time()

            if self.coords_per is not None and now >= next_pvt:
                self._reader_side_pvt(left)
                next_pvt = self._advance_due(next_pvt, self.coords_per, now)

            if self.dop_per is not None and now >= next_dop:
                self._reader_side_dop(left)
                next_dop = self._advance_due(next_dop, self.dop_per, now)

            # sleep until the next due task (or a small fallback)
            candidates = [t for t in (next_pvt, next_dop) if t is not None]
            if candidates:
                sleep_for = max(0.0, min(candidates) - time.time())
                time.sleep(sleep_for if sleep_for > 0.0 else 0.001)
            else:
                time.sleep(0.01)

    # ---------- Readers ----------
    def _reader_side_pvt(self, left):
        gps  = self.gps_left if left else self.gps_right
        side = 'left' if left else 'right'
        if gps is None:
            time.sleep(0.005)
            return
        try:
            coords = gps.geo_coords()
            if coords:
                stamp_ros = self._gps_stamp_to_ros_time(coords)
                key_ms    = self._key_from_coords(coords, stamp_ros)
                with self.state_lock:
                    self.state[side]['coords']    = coords
                    self.state[side]['stamp_ros'] = stamp_ros
                    self.state[side]['key_ms']    = key_ms
                with self.sync_lock:
                    buf = self.left_buf if left else self.right_buf
                    buf.append((key_ms, coords, stamp_ros))
        except serial.SerialException as e:
            self.get_logger().error(f"{side.capitalize()} GPS read error: {e} — reconnecting…")
            ok = False
            for _ in range(10):
                if self._reopen_one(side):
                    ok = True; break
                time.sleep(0.5)
            if not ok:
                time.sleep(1.0)
        except Exception as e:
            self.get_logger().warning(f"{side.capitalize()} PVT loop warning: {e}")

    def _reader_side_dop(self, left):
        gps  = self.gps_left if left else self.gps_right
        side = 'left' if left else 'right'
        if gps is None:
            time.sleep(0.005)
            return
        try:
            dop = gps.get_DOP()
            if dop:
                with self.state_lock:
                    self.state[side]['dop'] = dop
        except serial.SerialException as e:
            self.get_logger().error(f"{side.capitalize()} GPS read error: {e} — reconnecting…")
            ok = False
            for _ in range(10):
                if self._reopen_one(side):
                    ok = True; break
                time.sleep(0.5)
            if not ok:
                time.sleep(1.0)
        except Exception as e:
            self.get_logger().warning(f"{side.capitalize()} DOP loop warning: {e}")

    # ---------- ROS callback ----------
    def gps_callback(self):
        with self.state_lock:
            left  = self.state['left'].copy()
            right = self.state['right'].copy()

        age_left = self.get_clock().now().nanoseconds*1e-9 - (left.get('stamp_ros').sec + left.get('stamp_ros').nanosec*1e-9)
        age_right = self.get_clock().now().nanoseconds*1e-9 - (right.get('stamp_ros').sec + right.get('stamp_ros').nanosec*1e-9)
        if age_left > self.buffer_timeout or age_right > self.buffer_timeout:  # e.g., >2s old = backlog
            self.get_logger().warn(f"GPS backlog ({age:.1f}s) — flushing serial buffers")
            self.gps_left.hard_port.reset_input_buffer()
            self.gps_right.hard_port.reset_input_buffer()

        # Publish individual GPS messages (if enabled and available)
        now_ros = self.get_clock().now()
        do_diag = (now_ros.nanoseconds * 1e-9 - self._last_diag_pub_sec) >= self._diag_period

        # per-side raw NavSatFix publishes only on new sample (key_ms change)
        if self.gps:
            if left.get('coords') is not None and left.get('key_ms') != self.last_pub_key['left']:
                msg1, json1 = self.assign_gps(left['coords'], left.get('dop'), "gps_left", left.get('stamp_ros'))
                self.publisher_gps1_satnav.publish(msg1)
                if do_diag: self.publisher_gps1_json.publish(json1)
                self.last_pub_key['left'] = left.get('key_ms')

            if right.get('coords') is not None and right.get('key_ms') != self.last_pub_key['right']:
                msg2, json2 = self.assign_gps(right['coords'], right.get('dop'), "gps_right", right.get('stamp_ros'))
                self.publisher_gps2_satnav.publish(msg2)
                if do_diag: self.publisher_gps2_json.publish(json2)
                self.last_pub_key['right'] = right.get('key_ms')

        # synchronized pair for heading + aggregate
        pair = self._pop_synced_pair()
        if pair is not None:
            cl, cr, stamp = pair
            if self.heading:
                dLon = (cr.lon - cl.lon)
                x = np.cos(np.radians(cr.lat)) * np.sin(np.radians(dLon))
                y = (np.cos(np.radians(cl.lat)) * np.sin(np.radians(cr.lat))
                     - np.sin(np.radians(cl.lat)) * np.cos(np.radians(cr.lat)) * np.cos(np.radians(dLon)))
                brng = 90.0 - np.degrees(np.arctan2(x, y))
                if brng < 0.0: brng += 360.0
                brng_f = self.heading_filter.update(float(brng))
                msg = Float32(); msg.data = float(brng_f if brng_f is not None else brng)
                self.publisher_brng.publish(msg)

            if self.diff:
                left_state  = {'coords': cl, 'dop': left.get('dop'),  'stamp_ros': stamp}
                right_state = {'coords': cr, 'dop': right.get('dop'), 'stamp_ros': stamp}
                self.aggregate_gps(left_state, right_state, do_diag)

        if do_diag:
            self._last_diag_pub_sec = now_ros.nanoseconds * 1e-9

    # ---------- Message builders (unchanged from your version) ----------
    def assign_gps(self, coords_gps, dop=None, frame_id="", stamp_ros=None):
        msg = NavSatFix()
        msg.header.stamp = stamp_ros if stamp_ros is not None else self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.latitude  = coords_gps.lat
        msg.longitude = coords_gps.lon
        msg.altitude  = (float(coords_gps.height)/1000.0) if hasattr(coords_gps, 'height') else 0.0
        msg.status.status  = NavSatStatus.STATUS_FIX if getattr(coords_gps, 'fixType', 0) >= 3 else NavSatStatus.STATUS_NO_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        if hasattr(coords_gps, 'hAcc') and hasattr(coords_gps, 'vAcc'):
            hacc = float(coords_gps.hAcc)/1000.0
            vacc = float(coords_gps.vAcc)/1000.0
            msg.position_covariance = [
                hacc**2, 0.0,   0.0,
                0.0,    hacc**2,0.0,
                0.0,    0.0,    vacc**2
            ]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        else:
            msg.position_covariance = [0.0]*9
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        debug = {
            "stamp_sec": (msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9),
            "iTOW": getattr(coords_gps, 'iTOW', None),
            "datetime": {
                "year":  getattr(coords_gps, 'year', None),
                "month": getattr(coords_gps, 'month', None),
                "day":   getattr(coords_gps, 'day', None),
                "hour":  getattr(coords_gps, 'hour', None),
                "min":   getattr(coords_gps, 'min', None),
                "sec":   getattr(coords_gps, 'sec', None),
                "nano":  getattr(coords_gps, 'nano', None),
            },
            "fix": {
                "fixType":   getattr(coords_gps, 'fixType', None),
                "gnssFixOK": getattr(coords_gps.flags, 'gnssFixOK', None) if hasattr(coords_gps, 'flags') else None,
                "diffSoln":  getattr(coords_gps.flags, 'diffSoln', None)   if hasattr(coords_gps, 'flags') else None,
                "carrSoln":  getattr(coords_gps, 'carrSoln', None)         if hasattr(coords_gps, 'carrSoln') else None,
            },
            "satellites": getattr(coords_gps, 'numSV', None),
            "position": {
                "lat": coords_gps.lat,
                "lon": coords_gps.lon,
                "height_ellipsoid_m": getattr(coords_gps, 'height', 0.0)/1000.0 if hasattr(coords_gps, 'height') else None,
                "height_msl_m":       getattr(coords_gps, 'hMSL',   0.0)/1000.0 if hasattr(coords_gps, 'hMSL')   else None
            },
            "accuracy": {
                "horizontal_m": getattr(coords_gps, 'hAcc', 0.0)/1000.0 if hasattr(coords_gps, 'hAcc') else None,
                "vertical_m":   getattr(coords_gps, 'vAcc', 0.0)/1000.0 if hasattr(coords_gps, 'vAcc') else None,
                "speed_mps":    getattr(coords_gps, 'sAcc', 0.0)/1000.0 if hasattr(coords_gps, 'sAcc') else None,
                "heading_acc_deg":  getattr(coords_gps, 'headAcc', None)
            },
            "velocity": {
                "velN_mps":        getattr(coords_gps, 'velN',   0.0)/1000.0 if hasattr(coords_gps, 'velN')   else None,
                "velE_mps":        getattr(coords_gps, 'velE',   0.0)/1000.0 if hasattr(coords_gps, 'velE')   else None,
                "velD_mps":        getattr(coords_gps, 'velD',   0.0)/1000.0 if hasattr(coords_gps, 'velD')   else None,
                "ground_speed_mps":getattr(coords_gps, 'gSpeed', 0.0)/1000.0 if hasattr(coords_gps, 'gSpeed') else None,
                "heading_deg":     getattr(coords_gps, 'headMot', None),
                "headingVeh_deg":  getattr(coords_gps, 'headVeh', None)
            }
        }
        if dop:
            debug["dop"] = {"pDOP": dop.pDOP, "hDOP": dop.hDOP, "vDOP": dop.vDOP}

        j = String(); j.data = json.dumps(debug)
        return msg, j

    # ---------- Ports & reopen ----------
    def find_ports(self):
        devices = {}
        for port in list_ports.comports():
            ublox = (port.vid == self.gps_port_ids[0] and port.pid == self.gps_port_ids[1])
            if ublox and port.location:
                if self.gps_left_antenna in port.location:
                    devices["Left_GPS_Antenna"] = port.device
                elif self.gps_right_antenna in port.location:
                    devices["Right_GPS_Antenna"] = port.device
        return devices

    def open_gps(self):
        self.get_logger().info("Looking for Gps Modules…")
        while rclpy.ok():
            left  = self._reopen_one('left')
            right = self._reopen_one('right')
            if left and right:
                break
            time.sleep(1)

    def _reopen_one(self, side):
        try:
            devs = self.find_ports()
            key  = 'Left_GPS_Antenna' if side == 'left' else 'Right_GPS_Antenna'
            if key not in devs:
                return False
            port = devs[key]
            ser  = serial.Serial(port, int(self.config['Gps']['baudrate']),
                                 timeout=float(self.config['Gps']['usb_timeout']))
            if side == 'left':
                self.left_port = port
                self.gps_left  = UbloxGps(ser)
            else:
                self.right_port = port
                self.gps_right  = UbloxGps(ser)

            # clear buffers
            for gps in (self.gps_left, self.gps_right):
                try:
                    if gps: gps.hard_port.reset_input_buffer()
                except Exception:
                    pass
            # disable NMEA autos
            for gps in (self.gps_left, self.gps_right):
                if not gps:
                    continue
                for nmea in ('GGA','GLL','GSA','GSV','RMC','VTG','ZDA'):
                    try:
                        gps.set_auto_msg('NMEA', nmea, 0)
                    except Exception:
                        pass

            self.get_logger().info(f"Reconnected {side.upper()} GPS on {port}")
            return True
        except serial.SerialException as e:
            self.get_logger().warning(f"Reopen {side} failed: {e}")
            return False

    # ---------- Shutdown ----------
    def destroy_node(self):
        self._stop_evt.set()
        try:
            if self.left_thread.is_alive():
                self.left_thread.join(timeout=1.0)
            if self.right_thread.is_alive():
                self.right_thread.join(timeout=1.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = read_gps_data()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down diff_gps node...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
