import os
import configparser

from serial import Serial
from cobs import cobs
import rclpy
from rclpy.node import Node
import threading, struct
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import time
import numpy as np
from serial.tools import list_ports

CONFIG_FILE = "sr.cfg"

class serial_ros(Node):

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
        node_name = nav_cfg['Node']['name']
        queue = int(nav_cfg['Node']['queue_length'])
        self.timer_period = float(nav_cfg['Node']['timer'])
        self.VID = int(nav_cfg['Arduino']['vid'], 0)
        self.PID = int(nav_cfg['Arduino']['pid'], 0)
        
        super().__init__(f'{robot_name}_{node_name}')

        self.open_port()
        self._rx_lock = threading.Lock()
        self._rx_latest = {
            "dfr":       [0.0]*4,
            "flow_rate": [0.0]*4,
            "pump_pwr":  [0.0]*4,
        }
        self._rx_thread = threading.Thread(target=self._uart_reader, daemon=True)
        self._rx_thread.start()

        # --- ROS interfaces ---
        self.dfr_sub = self.create_subscription(Float32MultiArray, f'/{robot_name}/in/dfr', self.dfr_cb, queue)
        self.dfr_pub  = self.create_publisher(Float32MultiArray, f"/{robot_name}/out/dfr",        queue)
        self.flow_pub = self.create_publisher(Float32MultiArray, f"/{robot_name}/out/flow_rate",  queue)
        self.pwr_pub  = self.create_publisher(Float32MultiArray, f"/{robot_name}/out/pump_pwr",   queue)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def find_ports(self, VID, PID):
        for port in list_ports.comports():
            # for Roboteq
            if port.vid==VID and port.pid==PID:
                return port.device

    def open_port(self):
        self.get_logger().info("Looking for Arduino…")
        while rclpy.ok():
            device = self.find_ports(self.VID, self.PID)
            if device:
                try:
                    self.ser = Serial(device, 9600, timeout=1)
                    self.get_logger().info(f"Connected to Arduino on {device}")
                    self.ser.dtr = False  # prevent auto-reset
                    self.ser.rts = False
                    time.sleep(0.8)
                    self.ser.reset_input_buffer()
                    return
                except SerialException as e:
                    self.get_logger().warn(f"Arduino open failed: {e}")
            time.sleep(1)

    def dfr_cb(self, msg:Float32MultiArray) -> None:
        if np.allclose(msg.data, self._rx_latest['dfr'], rtol=1e-6, atol=1e-12, equal_nan=False):
            return
        try:
            payload = b"".join(struct.pack(">H", int(v)) for v in msg.data)
        except (TypeError, ValueError, struct.error) as e:
            self.get_logger().warn(f"Bad TX payload: {e}; dropping")
            return
        try:
            frame = cobs.encode(payload) + b"\x00"
        except cobs.EncodeError as e:
            self.get_logger().warn(f"COBS encode failed: {e}")
            return
        try:
            self.ser.write(frame)
        except (SerialException, OSError) as e:
            self.get_logger().warn(f"Write failed: {e}; reopening port…")
            self.open_port()

    def timer_callback(self) -> None:
        dfr, flow, pwr = self.get_arduino_vals()   # (tuples of length 4)
        self.dfr_pub.publish( self._make_f32_array(dfr,  "dfr") )
        self.flow_pub.publish(self._make_f32_array(flow, "flow_rate") )
        self.pwr_pub.publish( self._make_f32_array(pwr,  "pump_pwr") )

    def _make_f32_array(self, data: list[float], label: str) -> Float32MultiArray:
        msg = Float32MultiArray()
        msg.data = [float(x) for x in data]
        dim = MultiArrayDimension()
        dim.label = label
        dim.size = len(msg.data)
        dim.stride = len(msg.data)
        msg.layout.dim = [dim]
        msg.layout.data_offset = 0
        return msg

    def get_arduino_vals(self):
        with self._rx_lock:
            return (self._rx_latest["dfr"],
                    self._rx_latest["flow_rate"],
                    self._rx_latest["pump_pwr"])

    def _uart_reader(self):
        """Background UART reader: COBS frames of 12 floats, little-endian."""
        while rclpy.ok():
            try:
                frame = self.ser.read_until(b"\x00")
            except (SerialException, OSError) as e:
                self.get_logger().warn(f"Serial read error: {e}; reopening…")
                self.open_port()
                continue
            if not frame:
                continue
            try:
                payload = cobs.decode(frame[:-1])
            except cobs.DecodeError as e:
                self.get_logger().warn(f"COBS decode error: {e}; skipping frame")
                continue
            if len(payload) != 48:
                self.get_logger().warn(f"Bad payload size {len(payload)}; expect 48")
                continue
            try:
                vals = struct.unpack("<12f", payload)
            except struct.error as e:
                self.get_logger().warn(f"Unpack error: {e}")
                continue
            with self._rx_lock:
                self._rx_latest["dfr"]       = list(vals[0:4])
                self._rx_latest["flow_rate"] = list(vals[4:8])
                self._rx_latest["pump_pwr"]  = list(vals[8:12])

def main(args=None):
    rclpy.init(args=args)
    pub = serial_ros()

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
