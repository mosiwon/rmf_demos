#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseWithCovarianceStamped
import threading
from fastapi import FastAPI
import uvicorn
import argparse
import yaml
import sys
from rclpy.qos import qos_profile_sensor_data



app = FastAPI()

class FleetManager(Node):
    def __init__(self):
        super().__init__('fleet_manager')
        self.battery_state_subscription = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            qos_profile_sensor_data)
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
        self.robot_data = {
            'battery': None,
            'position': None
        }

    def battery_callback(self, msg):
        print('Fleet Manager: battery_callback received :', msg.percentage, '%')
        self.robot_data['battery'] = msg.percentage

    def pose_callback(self, msg):
        self.robot_data['position'] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': self.get_yaw_from_pose(msg.pose.pose)
        }

    def get_yaw_from_pose(self, pose):
        # Quaternion to Euler conversion
        print(pose.orientation)

@app.get('/status/{robot_name}')
def get_status(robot_name: str):
    # Dummy data for map name and other fields
    data = {
        'robot_name': robot_name,
        'map_name': 'L1',
        'position': fleet_manager.robot_data.get('position'),
        'battery': fleet_manager.robot_data.get('battery'),
        'destination_arrival': None,
        'last_completed_request': None,
        'replan': False
    }
    return {'data': data, 'success': True, 'msg': ''}

def main(argv=sys.argv):
    rclpy.init(args=argv)
    global fleet_manager
    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    
    
    parser = argparse.ArgumentParser(
        prog="fleet_manager",
        description="Configure and spin up the fleet adapter")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file")
    parser.add_argument("-n", "--nav_graph", type=str, required=True,
                        help="Path to the nav_graph for this fleet adapter")
    args = parser.parse_args(args_without_ros[1:])
    
    with open(args.config_file, "r") as f:
        config = yaml.safe_load(f)
        
    fleet_manager = FleetManager()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(fleet_manager,))
    spin_thread.start()
        
    uvicorn.run(app,
                host=config['fleet_manager']['ip'],
                port=config['fleet_manager']['port'],
                log_level='warning')


if __name__ == '__main__':
    main()

