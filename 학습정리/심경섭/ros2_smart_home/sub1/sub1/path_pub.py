import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from squaternion import Quaternion
from nav_msgs.msg import Odometry, Path

from math import pi, cos, sin, sqrt
import tf2_ros
import os


# path_pub 노드는 make_path 노드에서 만든 텍스트 파일을 읽어와 전역 경로(/global_path)로 사용하고,
# 전역 경로 중 로봇과 가장 가까운 포인트를 시작점으로 실제 로봇이 경로 추종에 사용하는 경로인 지역 경로(/local_path)를 만들어주는 노드입니다.


# 노드 로직 순서
# 1. publisher, subscriber 만들기
# 2. 만들어 놓은 경로 데이터를 읽기 모드로 open
# 3. 경로 데이터를 읽어서 Path 메시지에 데이터를 넣기
# 4. 주기마다 실행되는 타이머함수 생성, local_path_size 설정
# 5. global_path 중 로봇과 가장 가까운 포인트 계산
# 6. local_path 예외 처리
# 7. global_path 업데이트 주기 재설정


class pathPub(Node):
    def __init__(self):
        super().__init__("path_pub")

        # 로직 1. publisher, subscriber 만들기
        self.global_path_pub = self.create_publisher(Path, "global_path", 10)
        self.local_path_pub = self.create_publisher(Path, "local_path", 10)
        self.subscription = self.create_subscription(
            Odometry, "/odom", self.listener_callback, 10
        )
        self.lidar_sub = self.create_subscription(
            Odometry, "/odom", self.listener_callback, 10
        )

        self.odom_msg = Odometry()
        # 아직 위치 정보를 수신받지 않았으므로 False로 초기화
        self.is_odom = False

        # 전역경로 메시지
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = "map"

        """
        로직 2. 만들어 놓은 경로 데이터를 읽기 모드로 open


        full_path=
        self.f=

        """
        full_path ="C:/Users/SSAFY/Desktop/skeleton/mobility-smarthome-skeleton/ros2_smart_home/sub1/result.txt"
        self.f = open(full_path, "r")

        """
        로직 3. 경로 데이터를 읽어서 Path 메시지에 데이터를 넣기

        lines=
        for line in lines :
            tmp=
            read_pose=
            read_pose.pose.position.x=
            read_pose.pose.position.y=
            read_pose.pose.orientation.w=
            self.global_path_msg.poses.append()
        
        self.f.close()

        """
        lines = self.f.readlines()
        for line in lines:
            '''
            기존 종호 코드
            tmp = line.split()
            하단은 수정 코드
            한줄에 X와 Y의 구분은 tab으로 구분되므로...
            맞나여?
            '''
            tmp = line.strip().split('\t')
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(tmp[0])
            read_pose.pose.position.y = float(tmp[1])
            # 쿼터니언에서 w 성분이 1.0이면 회전이 없는 것을 의미합니다.
            read_pose.pose.orientation.w = 1.0
            self.global_path_msg.poses.append(read_pose)

        self.f.close()

        # 로직 4. 주기마다 실행되는 타이머함수 생성, local_path_size 설정
        time_period = 0.02
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.local_path_size = 20

        self.count = 0

    def listener_callback(self, msg):
        self.is_odom = True
        self.odom_msg = msg

    def timer_callback(self):
        if self.is_odom == True:
            local_path_msg = Path()
            local_path_msg.header.frame_id = "/map"

            x = self.odom_msg.pose.pose.position.x
            y = self.odom_msg.pose.pose.position.y
            print(x, y)
            # 로봇의 현재 위치에 가장 가까운 전역 경로(global_path_msg) 상의 경로 포인트(waypoint)의 인덱스
            current_waypoint = -1
            """
            로직 5. global_path 중 로봇과 가장 가까운 포인트 계산

            min_dis=
            for i,waypoint in enumerate(self.global_path_msg.poses) :

                distance=
                if distance < min_dis :
                    min_dis=
                    current_waypoint=
            
            """
            min_dis = float("inf")
            for i, waypoint in enumerate(self.global_path_msg.poses):
                distance = sqrt(
                    pow(x - waypoint.pose.position.x, 2)
                    + pow(y - waypoint.pose.position.y, 2)
                )
                if distance < min_dis:
                    min_dis = distance
                    current_waypoint = i

            """
            로직 6. local_path 예외 처리

            if current_waypoint != -1 :
                if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):                 
                    
                

                else :
                  
                    
                        
            """
            
            # min_dis가 존재하며
            if current_waypoint != -1:
                # 전역 경로에서 가장 가까운 지점으로부터  local_path_size(인덱스를 나타내며 길이가 짧을수록 뚫고갈 수 있음=> 적절히 설정)를 더한값이
                # 전역경로의 포인트를 벗어나지 않을 때
                if current_waypoint + self.local_path_size < len(
                    self.global_path_msg.poses
                ):
                    # 가장 가까운 지점으로부터 current_waypoint + self.local_path_size인덱스까지의 포인트들에 대해서 지역경로에 넣어줌
                    for num in range(
                        current_waypoint, current_waypoint + self.local_path_size
                    ):
                        tmp_pose = PoseStamped()
                        tmp_pose.pose.position.x = self.global_path_msg.poses[
                            num
                        ].pose.position.x
                        tmp_pose.pose.position.y = self.global_path_msg.poses[
                            num
                        ].pose.position.y
                        tmp_pose.pose.orientation.w = 1.0
                        local_path_msg.poses.append(tmp_pose)
                        #위의 여러줄과 아래는 같은 말
                        # local_path_msg.poses = self.global_path_msg.poses[current_waypoint:current_waypoint + self.local_path_size]

                else:
                    # 만약 전역경로의 인덱스를 벗어나게 되면
                    #가장 가까운 지점으로부터 끝지점까지 local_path_msg에 실어줌
                    for num in range(current_waypoint, len(self.global_path_msg.poses)):
                        tmp_pose = PoseStamped()
                        tmp_pose.pose.position.x = self.global_path_msg.poses[
                            num
                        ].pose.position.x
                        tmp_pose.pose.position.y = self.global_path_msg.poses[
                            num
                        ].pose.position.y
                        tmp_pose.pose.orientation.w = 1.0
                        local_path_msg.poses.append(tmp_pose)
                        #위의 반복문과 같은 말
                        # local_path_msg.poses = self.global_path_msg.poses[current_waypoint:]
                        

            self.local_path_pub.publish(local_path_msg)
            
            
            

        # 로직 7. global_path 업데이트 주기 재설정
        # 로봇이 이동하면서 경로가 변경될 수 있으므로, 주기적으로 새로운 경로 정보를 전달하여 로봇의 경로를 최신 상태로 유지하기 위한 메커니즘입니다. 
        if self.count % 10 == 0:
            self.global_path_pub.publish(self.global_path_msg)
        self.count += 1


def main(args=None):
    rclpy.init(args=args)

    path_pub = pathPub()

    rclpy.spin(path_pub)

    path_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
