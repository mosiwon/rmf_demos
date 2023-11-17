# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


'''
    The RobotAPI class is a wrapper for API calls to the robot. Here users
    are expected to fill up the implementations of functions which will be used
    by the RobotCommandHandle. For example, if your robot has a REST API, you
    will need to make http request calls to the appropriate endpoints within
    these functions.
'''
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped, Twist
import math
import requests


class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, config_yaml):
        
        self.ip = config_yaml['ip']
        self.port = str(config_yaml['port'])
        self.user = config_yaml['user']
        self.password = config_yaml['password']
        self.prefix = f'http://{self.ip}:{self.port}'
        self.timeout = 5.0
        self.debug = False
        self.odom_received = False
        self.battery_received = False
        self.current_pose = None
        self.current_battery = None
        # ROS 2 노드 초기화
        self.node = rclpy.create_node('robot_api_node')
        print("RobotAPI: ROS 2 노드 초기화 완료")
        # 필요한 토픽, 서비스, 액션 구독 및 생성


    def check_connection(self):
        print("Checking connection in RobotclientAPI...")
        ''' 연결 상태 확인 '''
        try:
            rclpy.spin_once(self.node, timeout_sec=5)  # 5초 동안 메시지 수신 대기
            return self.odom_received and self.battery_received
        except Exception as e:
            print(f"Connection check failed: {e}")
            return False

    def pose_callback(self, msg):
        print("pose_callback: 수신된 AMCL 데이터")
        self.current_pose = msg

    def battery_callback(self, msg):
        print("battery_callback: 수신된 BatteryState 데이터")
        self.current_battery = msg
 
    def position(self, robot_name: str):
        ''' 현재 로봇의 위치를 반환 '''
        if self.current_pose:
           
            return None
        return None

    def battery_soc(self, robot_name: str):
        ''' 현재 로봇의 배터리 잔량을 반환 '''
        print("battery_soc")
        if self.current_battery:
            
            return self.current_battery.percentage
        return None



    def navigate(
        self,
        robot_name: str,
        pose,
        map_name: str,
        speed_limit=0.0
    ):
        ''' Request the robot to navigate to pose:[x,y,theta] where x, y and
            and theta are in the robot's coordinate convention. This function
            should return True if the robot has accepted the request,
            else False '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def start_activity(
        self,
        robot_name: str,
        activity: str,
        label: str
    ):
        ''' Request the robot to begin a process. This is specific to the robot
        and the use case. For example, load/unload a cart for Deliverybot
        or begin cleaning a zone for a cleaning robot.
        Return True if process has started/is queued successfully, else
        return False '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def stop(self, robot_name: str):
        ''' Command the robot to stop.
            Return True if robot has successfully stopped. Else False. '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def map(self, robot_name: str):
        ''' Return the name of the map that the robot is currently on or
        None if any errors are encountered. '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return 'L1'

    def is_command_completed(self):
        ''' Return True if the robot has completed its last command, else
        return False. '''
        # ------------------------ #
        # IMPLEMENT YOUR CODE HERE #
        # ------------------------ #
        return False

    def get_data(self, robot_name: str):
        ''' Returns a RobotUpdateData for one robot if a name is given. Otherwise
        return a list of RobotUpdateData for all robots. '''
        try:
            url = self.prefix + f'/status/{robot_name}'
            response = requests.get(url, timeout=self.timeout)
            if response.status_code == 200:
                data = response.json()['data']
                map = data.get('map_name')
                position = data.get('position')
                battery_soc = data.get('battery') # 백분율을 소수로 변환
                return None #RobotUpdateData(robot_name, map, position, battery_soc)
        except Exception as e:
            print(f"Error while getting robot data: {e}")
        return None


class RobotUpdateData:
    ''' Update data for a single robot. '''

    def __init__(self,
                 robot_name: str,
                 map: str,
                 position: list[float],
                 battery_soc: float,
                 requires_replan: bool | None = None):
        self.robot_name = robot_name
        self.position = position
        self.map = map
        self.battery_soc = battery_soc
        self.requires_replan = requires_replan
