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
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy



app = FastAPI()

class FleetManager(Node):
    def __init__(self):
        super().__init__('fleet_manager')
        self.robot_data = {}

    def add_robot(self, robot_name):
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        print(f'Adding robot {robot_name}')
        self.robot_data[robot_name] = {
            'battery': None,
            'position': None,
            'battery_subscription': self.create_subscription(
                BatteryState,
                f'/{robot_name}/battery_state',
                lambda msg: self.battery_callback(msg, robot_name),
                qos_profile_sensor_data),
            'pose_subscription': self.create_subscription(
                PoseWithCovarianceStamped,
                f'/{robot_name}/amcl_pose',
                lambda msg: self.pose_callback(msg, robot_name),
                qos)
        }

    def battery_callback(self, msg, robot_name):
        if robot_name in self.robot_data:
            self.robot_data[robot_name]['battery'] = msg.percentage
        print(f'{robot_name} Battery callback: {msg.percentage}')

    def pose_callback(self, msg, robot_name):
        if robot_name in self.robot_data:
            self.robot_data[robot_name]['position'] = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'yaw': self.get_yaw_from_pose(msg.pose.pose)
            }


    def get_yaw_from_pose(self, pose):
        # Quaternion to Euler conversion
        print(pose.orientation)

@app.get('/status/{robot_name}')
def get_status(robot_name: str):
    robot_status = fleet_manager.robot_data.get(robot_name)
    if robot_status:
        data = {
            'robot_name': robot_name,
            'map_name': 'L1',  # or dynamic value based on the robot's data
            'position': robot_status.get('position'),
            'battery': robot_status.get('battery'),
            'destination_arrival': None,
            'last_completed_request': None,
            'replan': False
        }
        return {'data': data, 'success': True, 'msg': ''}
    else:
        return {'data': {}, 'success': False, 'msg': f'No data found for {robot_name}'}


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
    robot_names = config['rmf_fleet']['robots']
    for robot_name in robot_names:
        fleet_manager.add_robot(robot_name)

    spin_thread = threading.Thread(target=rclpy.spin, args=(fleet_manager,))
    spin_thread.start()
        
    uvicorn.run(app,
                host=config['fleet_manager']['ip'],
                port=config['fleet_manager']['port'],
                log_level='warning')


if __name__ == '__main__':
    main()

