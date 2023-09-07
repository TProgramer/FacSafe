import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ssafy_msgs.msg import TurtlebotStatus
from sensor_msgs.msg import Imu
from squaternion import Quaternion
from nav_msgs.msg import Odometry
from math import pi, cos, sin
import tf2_ros
import geometry_msgs.msg
import time
import math

# odom 노드는 로봇의 속도(/turtlebot_status), Imu센서(/imu) 메시지를 받아서 로봇의 위치를 추정하는 노드입니다.
# sub1_odom은 imu로 부터 받은 Quaternion을 사용하거나 각속도, 가속도 데이터를 이용해서 로봇의 포즈를 추정 할 것입니다.

# 노드 로직 순서
# 1. publisher, subscriber, broadcaster 만들기
# 2. publish, broadcast 할 메시지 설정
# 3. imu 에서 받은 quaternion을 euler angle로 변환해서 사용
# 4. 로봇 위치 추정
# 5. 추정한 로봇 위치를 메시지에 담아 publish, broadcast


class odom(Node):
    def __init__(self):
        super().__init__("odom")

        '''로직 1. publisher, subscriber, broadcaster 만들기'''
        self.subscription = self.create_subscription(
            TurtlebotStatus, "/turtlebot_status", self.listener_callback, 10
        )
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        # broadcaster는 좌표계를 만들어주는역활을 함
        # TransformStamped타입의 메시지에 담아 sendTransform을 하면 좌표계가 broadcast됨
        # rviz2 에서 tf를 추가해 확인 가능 

        # 로봇의 pose를 저장해 publish 할 메시지 변수 입니다.
        self.odom_msg = Odometry()
        # Map -> base_link 좌표계에 대한 정보를 가지고 있는 변수 입니다.
        self.base_link_transform = geometry_msgs.msg.TransformStamped()
        # base_link -> laser 좌표계에 대한 정보를 가지고 있는 변수 입니다.
        
        # Map -> base_link 좌표계와 base_link -> laser 좌표계는 로봇 시스템에서 사용되는 두 개의 상대적인 좌표계입니다. 
        # 이러한 좌표계는 로봇의 상대적인 위치와 방향을 나타내는 데 사용됩니다.

        # Map -> base_link 좌표계:

        # Map 좌표계는 로봇이 움직이는 전역(global) 좌표계로, 로봇의 움직임을 추적하는 데 사용됩니다. 
        # 주로 로봇의 위치와 방향을 나타내는 데 사용됩니다.
        # base_link 좌표계는 로봇의 기준 프레임으로, 로봇의 중심 또는 기준점을 나타냅니다. 
        # 이 좌표계는 로봇의 움직임을 상대적으로 정의하기 위해 사용됩니다.
        # Map -> base_link 변환은 로봇의 전역 위치와 방향을 base_link 좌표계로 매핑합니다. 
        # 이렇게 하면 로봇이 어디에 있는지와 어느 방향을 향해 있는지를 알 수 있습니다.

        # base_link -> laser 좌표계:

        # base_link 좌표계는 앞서 언급한 것처럼 로봇의 중심 또는 기준점을 나타내며, 
        # 이것이 로봇 팔, 카메라, 센서 등과 같은 하위 부분의 기준점으로 사용됩니다.
        # laser 좌표계는 로봇의 레이저 스캐너 또는 레이저 센서를 나타냅니다. 
        # 이 좌표계는 로봇의 레이저 스캐너가 향하는 방향과 위치를 나타내며, 레이저 데이터를 정확하게 해석하기 위해 사용됩니다.
        
        
        
        self.laser_transform = geometry_msgs.msg.TransformStamped()
        self.is_status = False
        self.is_imu = False
        self.is_calc_theta = False
        # x,y,theta는 추정한 로봇의 위치를 저장할 변수 입니다.
        self.x = -8.75
        self.y = -8.75

        self.theta = 0
        # imu_offset은 초기 로봇의 orientation을 저장할 변수 입니다.
        self.imu_offset = 0
        self.prev_time = 0
        
        
        self.odom_msg.header.frame_id = "map"
        self.odom_msg.child_frame_id = "base_link"

        self.base_link_transform.header.frame_id = "map"
        self.base_link_transform.child_frame_id = "base_link"
        # map 좌표계 ->  base_link좌표계(로봇기준 움직임 좌표계) 변환
        

        self.laser_transform.header.frame_id = "base_link"
        self.laser_transform.child_frame_id = "laser"
        # base_link좌표계 -> laser 좌표계
        
        self.laser_transform.transform.translation.x = 0.0
        self.laser_transform.transform.translation.y = 0.0
        self.laser_transform.transform.translation.z = 1.0
        self.laser_transform.transform.rotation.w = 1.0
        # 레이저는 터틀봇에 즉 base_link와에 달려있는 레이저로서
        # base_link와 laser간의 거리, 회전은 변하지 않으므로 초기 세팅
        

    def imu_callback(self, msg):

        # 로직3. CHAT GPT채워줌
        
        if self.is_imu ==False :
            # 초기 imu_offset 설정
            # 로봇의 시작 방향을 설정하는 역할
            self.is_imu=True
            # IMU로부터 Quaternion의 값을 받아들임
            imu_q = Quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
            self.imu_offset = (imu_q.to_euler())[2] - 90*math.pi/180
            

        else :
            # 현재 자세 (orientation) 데이터를 이용하여 self.theta 계산
            # IMU 데이터의 쿼터니언을 사용하여 현재 자세 (orientation)에서 
            # 초기 imu_offset을 뺀 값을 self.theta에 저장합니다. 
            # 이렇게 하면 현재 로봇의 방향이 계산됩니다.
            imu_q = Quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
            self.theta = (imu_q.to_euler())[2] - self.imu_offset
        
        

    def listener_callback(self, msg):
        
        # TurtlebotStatus 메시지로부터 선속도와 각속도 추출
        # 원래의 코드
        # print(
        #     "linear_vel : {}  angular_vel : {}".format(
        #         msg.twist.linear.x, -msg.twist.angular.z
        #     )
        # )
        if self.is_imu == True:
            
            # 처음 메시지를 받았을 때, 초기화 및 시간 설정
            if self.is_status == False: 
                self.is_status = True
                self.prev_time = rclpy.clock.Clock().now()
            else:
                
                # 현재 시간 계산
                self.current_time = rclpy.clock.Clock().now()
                
                # 계산 주기 계산 (단위: 초)
                # 계산 주기를 저장한 변수 입니다. 단위는 초(s)
                self.period = (
                    self.current_time - self.prev_time
                ).nanoseconds / 1000000000
                
                # 로봇의 선속도, 각속도를 저장하는 변수, 시뮬레이터에서 주는 각 속도는 방향이 반대이므로 (-)를 붙여줍니다.
                # 원래의 코드
                linear_x = msg.twist.linear.x
                angular_z = -msg.twist.angular.z
                
                # 로봇 위치 추정
                # 로봇의 x, y 위치 업데이트 (오일러 각도와 선속도를 사용하여)
                
                # 기존의 x,y에서 해당 방향의 선속도와 시간을 
                # 곱하여 더해줌으로서 현재 위치를 저장
                # (각도도 마찬가지)
                self.x += linear_x * cos(self.theta) * self.period
                self.y += linear_x * sin(self.theta) * self.period
                self.theta += angular_z * self.period

                # TF2 변환 메시지 및 Odometry 메시지 설정
                # base_link의 위치 및 방향을 설정
                
                # self.base_link_transform은 로봇의 base_link 프레임과 다른 프레임 간의 변환 정보를 나타내는 TF2 변환 메시지입니다.
                # .header는 메시지의 헤더 부분을 나타냅니다. 헤더에는 메시지의 시간 정보 등이 포함됩니다.
                # .stamp는 헤더의 timestamp(시간 정보)를 나타냅니다.
                
                
                # 원래의 코드
                self.base_link_transform.header.stamp = (
                    
                # rclpy.clock.Clock().now()는 현재 ROS 2 시간을 가져오는 메서드입니다. 즉, 현재 시간을 나타냅니다.
                # .to_msg() 메서드는 현재 시간을 ROS 메시지 형식으로 변환하는 역할을 합니다. 
                # ROS 메시지의 시간은 std_msgs.msg.Time 형식을 사용하며, 이 형식에 맞게 현재 시간을 변환합니다.
                # 원래의 코드
                    rclpy.clock.Clock().now().to_msg()
                )
                self.laser_transform.header.stamp = rclpy.clock.Clock().now().to_msg()
                
                # 로봇의 움직임이 2D일 때는 Roll(롤) 각도와 Pitch(피치) 각도를 0으로 설정  0,0,self.theta는 (롤각도, 피치각도, 요우각도)
                # 이는 단순히 필요치 않기 때문
                # 롤 각도는 물체가 좌우로 회전하는 각도
                # 피치 각도는 물체가 위아래로 회전하는 각도
                # 요 각도는 물체가 주위의 수평 축 주위로 회전하는 각도
                
                # 하단 전부 종호와 같은 코드
                q = Quaternion.from_euler(0, 0, self.theta)

                self.base_link_transform.transform.translation.x = self.x
                self.base_link_transform.transform.translation.y = self.y
                self.base_link_transform.transform.rotation.x = q.x
                self.base_link_transform.transform.rotation.y = q.y
                self.base_link_transform.transform.rotation.z = q.z
                self.base_link_transform.transform.rotation.w = q.w

                self.odom_msg.pose.pose.position.x = self.x
                self.odom_msg.pose.pose.position.y = self.y
                self.odom_msg.pose.pose.orientation.x = q.x
                self.odom_msg.pose.pose.orientation.y = q.y
                self.odom_msg.pose.pose.orientation.z = q.z
                self.odom_msg.pose.pose.orientation.w = q.w
                self.odom_msg.twist.twist.linear.x = linear_x
                self.odom_msg.twist.twist.angular.z = angular_z

                
                self.broadcaster.sendTransform(self.base_link_transform)
                self.broadcaster.sendTransform(self.laser_transform)
                self.odom_publisher.publish(self.odom_msg)
                self.prev_time = self.current_time


def main(args=None):
    rclpy.init(args=args)

    sub1_odom = odom()

    rclpy.spin(sub1_odom)

    sub1_odom.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
