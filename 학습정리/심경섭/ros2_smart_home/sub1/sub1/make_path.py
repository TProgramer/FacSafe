import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from squaternion import Quaternion
from nav_msgs.msg import Odometry, Path
import os
from math import sqrt
import sub2

# make_path 노드 설명
# 로봇의 위치(Odometry)를 받아서 매 0.1m 간격으로 x,y 좌표를 텍스트 파일에 기록하고, Path 메시지를 Publish 합니다.
# rviz에서 Path 메시지를 볼 수 있습니다. 경로가 잘 만들어지는지 rviz를 통해 확인해주세요.
# 기록을 해서 만드는 방법은 주행할 경로를 만드는 가장 쉬운 방법입니다. 생성된 텍스트 파일은 추 후에 텍스트 파일을 읽어서 Path 메시지를 publish 하는 노드(path_pub)에서 사용하고, 경로 추종알고리즘에서 사용 됩니다.
# $ ros2 run sub1 make_path 실행 시 Terminal 경로를 catkin_ws\src\ros2_smart_home\sub1\sub1로 세팅 후 실행해야합니다.

# 노드 로직 순서
# 1. 노드에 필요한 publisher, subscriber 생성
# 2. 저장할 경로 및 텍스트파일 이름을 정하고, 쓰기 모드로 열기
# 3. 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장
# 4. 콜백함수에서 이전 위치와 현재 위치의 거리 계산
# 5. 이전 위치보다 0.1m 이상일 때 위치를 path_msg.poses에 추가하고 publish
# 6. x,y 를 문자열로 바꾸고 x와 y 사이의 문자열은 /t 로 구분


class makePath(Node):
    def __init__(self):
        super().__init__("make_path")

        # 로직 1. 노드에 필요한 publisher, subscriber 생성
        self.path_pub = self.create_publisher(Path, "global_path", 10)
        self.subscription = self.create_subscription(
            Odometry, "/odom", self.listener_callback, 10
        )

        """
        로직 2. 저장할 경로 및 텍스트파일 이름을 정하고, 쓰기 모드로 열기
        full_path=
        self.f=
        """
        full_path = "C:/Users/SSAFY/Desktop/skeleton/mobility-smarthome-skeleton/ros2_smart_home/sub1/result.txt"
        # open 함수는 파일을 열기 위해 사용됩니다
        # w는 파일을 쓰기 모드로 열라는 것
        self.f = open(full_path, "w")


        # self.is_odom 변수는 처음 메시지가 도착한 시점을 식별하고, 
        # 그 이후에도 메시지를 지속적으로 처리하기 위한 플래그 역할
        self.is_odom = True
        
        # 이전 위치를 저장할 변수입니다.
        self.prev_x = 0.0
        self.prev_y = 0.0



        # Path 메시지는 nav_msgs 패키지에서 정의되어 있으며, 
        # 로봇의 경로를 구성하는 geometry_msgs/PoseStamped[] 유형의 필드를 가지고 있습니다. 
        # 이 필드는 로봇의 위치와 방향을 포함하는 PoseStamped 메시지의 배열로 구성됩니다.
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

    def listener_callback(self, msg):
        print(
            "x : {} , y : {} ".format(
                msg.pose.pose.position.x, msg.pose.pose.position.y
            )
        )
        if self.is_odom == False:
            pass
            """
            로직 3. 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장해줍니다. 
            self.is_odom = 
            self.prev_x = 
            self.prev_y = 
            """
            
            # CHAT GPT코드
            self.is_odom = True
            self.prev_x = msg.pose.pose.position.x
            self.prev_y = msg.pose.pose.position.y

        else:
            
            # PoseStamped는 ROS에서 자세한 위치 및 방향 정보를 나타내는 메시지 유형입니다. 
            # 이 메시지는 3D 공간에서의 위치(x, y, z)와 방향(롤, 피치, 요우)을 표현할 수 있습니다. 
            # PoseStamped 메시지에는 geometry_msgs 패키지에서 가져온 PoseStamped 메시지 유형을 사용합니다.
            waypint_pose = PoseStamped()
            
            # x,y 는 odom 메시지에서 받은 로봇의 현재 위치를 나타내는 변수입니다.
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            """
            로직 4. 콜백함수에서 이전 위치와 현재 위치의 거리 계산
            (테스트) 유클리디안 거리를 구하는 부분으로 x=2, y=2 이고, self.prev_x=0, self.prev_y=0 이라면 distance=2.82가 나와야합니다.

            distance = 
            """
            # 종호코드
            distance = sqrt(pow(x - self.prev_x, 2) + pow(y - self.prev_y, 2))

            """
            if distance > 0.1 :
                로직 5. 거리차이가 위치보다 0.1m 이상일 때 위치를 path_msg.poses에 추가하고 publish
                waypint_pose.pose.position.x=
                waypint_pose.pose.position.y=
                waypint_pose.pose.orientation.w=1.0
                self.path_msg.poses.append(waypint_pose)
                self.path_pub.publish(self.path_msg)                
            """
            # 종호코드
            # 이전 위치와 거리차이가 0.1 이상일 때 기록하기
            if distance > 0.1:
                waypint_pose.pose.position.x = x
                waypint_pose.pose.position.y = y
                waypint_pose.pose.orientation.w = 1.0
                self.path_msg.poses.append(waypint_pose)
                self.path_pub.publish(self.path_msg)

                """
                로직 6. x,y 를 문자열로 바꾸고 x와 y 사이의 문자열은 /t 로 구분
                data=
                self.f
                self.prev_x=x
                self.prev_y=y
                """
                data = str(x) + "\t" + str(y)
                self.f.write(data)
                self.prev_x = x
                self.prev_y = y


def main(args=None):
    rclpy.init(args=args)

    odom_based_make_path = makePath()

    rclpy.spin(odom_based_make_path)

    odom_based_make_path.f.close()
    odom_based_make_path.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
