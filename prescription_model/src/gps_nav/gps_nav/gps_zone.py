import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

import numpy as np

import os
import configparser
import time

CONFIG_FILE = "gps_nav_config.cfg"

class read_gps_data(Node):

    def __init__(self):

        config_file_path = os.getcwd()
        config_file_path = os.path.abspath(os.path.join(config_file_path, CONFIG_FILE))  

        self.config = configparser.ConfigParser()
        if not self.config.read(config_file_path):
            print(f"Config file {CONFIG_FILE} not found or invalid.")
            try:
                rclpy.shutdown()
            except Exception:
                pass
            return

        robot_name = self.config['Robot']['name']

        self.gps_port_ids = list(
            map(lambda s: int(s.strip(), 0),
                self.config['Ublox']['port_ids'].split(':'))
        )


        super().__init__(f'{robot_name}_{node_name}')

        self.open_gps()
        
        self.publisher_gps_agg = self.create_publisher(NavSatFix, f'/{robot_name}/{gps_agg_name}', queue_length)

    def gps_callback(self):
        try:
            coords_gps_left = self.gps_left.geo_coords()
        except serial.SerialException as e:
            self.get_logger().error(f"Left GPS read error: {e} — reconnecting")
            self.open_gps()
            return
        try:
            coords_gps_right = self.gps_right.geo_coords()
        except serial.SerialException as e:
            self.get_logger().error(f"Right GPS read error: {e} — reconnecting")
            self.open_gps()
            return

        if coords_gps_left is None or coords_gps_right is None:
            return

        if self.heading:
            self.get_bearing(coords_gps_left, coords_gps_right)
        if self.diff:
            self.aggregate_gps(coords_gps_left, coords_gps_right)
        
        if self.gps:
            msg1 = NavSatFix()
            msg1.latitude = coords_gps_left.lat
            msg1.longitude = coords_gps_left.lon
            
            msg2 = NavSatFix()
            msg2.latitude = coords_gps_right.lat
            msg2.longitude = coords_gps_right.lon

            self.publisher_gps1.publish(msg1)
            self.publisher_gps2.publish(msg2)


    def find_ports(self):
        devices = {}
        for port in list_ports.comports():
            # for Roboteq
            Ublox = port.vid==self.gps_port_ids[0] and port.pid==self.gps_port_ids[1]
            if Ublox and port.location:
                if self.gps_left_antenna in port.location:
                    devices["Left_GPS_Antenna"] = port.device
                elif self.gps_right_antenna in port.location:
                    devices["Right_GPS_Antenna"] = port.device
        return devices

    def open_gps(self):
        self.get_logger().info("Looking for Gps Modules…")
        while rclpy.ok():
            devices = self.find_ports()
            if 'Left_GPS_Antenna' in devices and 'Right_GPS_Antenna' in devices:
                try:
                    self.gps_left = UbloxGps(serial.Serial(devices['Left_GPS_Antenna'], 38400, timeout=1))
                    self.gps_right = UbloxGps(serial.Serial(devices['Right_GPS_Antenna'], 38400, timeout=1))
                    self.get_logger().info(f"Connected to GPS on Left: {devices['Left_GPS_Antenna']} and Right: {devices['Right_GPS_Antenna']}")
                    return
                except serial.SerialException as e:
                    self.get_logger().warn(f"Sonar open failed: {e}")
            time.sleep(1)


def main(args=None):
    rclpy.init(args=args)

    pub = read_gps_data()

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
