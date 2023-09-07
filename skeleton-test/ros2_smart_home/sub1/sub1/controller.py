import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ssafy_msgs.msg import TurtlebotStatus,EnviromentStatus
from std_msgs.msg import Float32,Int8MultiArray
import time

# controller는 시뮬레이터로 부터를 데이터를 수신해서 확인(출력)하고, 송신해서 제어가 되는지 확인해보는 통신 테스트를 위한 노드입니다.
# 메시지를 받아서 어떤 데이터들이 있는지 확인하고, 어떤 메시지를 보내야 가전 또는 터틀봇이 제어가 되는지 확인해보면서 ros2 통신에 익숙해지세요.
# 수신 데이터 : 터틀봇 상태(/turtlebot_status), 환경정보(/envir_status), 가전정보(/app_status)
# 송신 데이터 : 터틀봇 제어(/ctrl_cmd), 가전제어(/app_control)

# 노드 로직 순서
# 1. 수신 데이터 출력
# 2. 특정 가전제품 ON
# 3. 특정 가전제품 OFF
# 4. 터틀봇 정지
# 5. 터틀봇 시계방향 회전
# 6. 터틀봇 반시계방향 회전


class Controller(Node):

    def __init__(self):
        super().__init__('sub1_controller')
        ## 메시지 송신을 위한 PUBLISHER 생성
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.app_control_pub = self.create_publisher(Int8MultiArray, 'app_control', 10)

        ## 메시지 수신을 위한 SUBSCRIBER 생성
        self.turtlebot_status_sub = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.listener_callback,10)
        self.envir_status_sub = self.create_subscription(EnviromentStatus,'/envir_status',self.envir_callback,10)
        self.app_status_sub = self.create_subscription(Int8MultiArray,'/app_status',self.app_callback,10)
        self.timer = self.create_timer(0.033, self.timer_callback)

        ## 로봇 제어 메시지 변수 생성 
        self.cmd_msg=Twist()
        
        # int8[] data형식
        self.app_control_msg=Int8MultiArray()
        
        for i in range(17):
            self.app_control_msg.data.append(0)
            # Int8MultiArray 메시지는 data라는 이름의 정수 리스트를 가지고 있습니다
            # 각 가전 제품에 대해 초기 상태 0으로 설정하고 있습니다
            # append()는 배열의 맨 뒤에 추가하는 형식으로 기존에 아무것도 없으므로 추가만 해주면 된디..


        self.turtlebot_status_msg=TurtlebotStatus()
        self.envir_status_msg=EnviromentStatus()
        self.app_status_msg=Int8MultiArray()
        self.is_turtlebot_status=False
        self.is_app_status=False
        self.is_envir_status=False


# sub

    def listener_callback(self, msg):
        # 터틀봇의 상태를 받아왔음을 체크
        self.is_turtlebot_status=True
        # 터틀봇의 상태 메시지 저장
        self.turtlebot_status_msg=msg


#sub
    def envir_callback(self, msg):
        # 주변 환경정보 받아옴을 체크
        self.is_envir_status=True
        # 주변 환경정보 저장
        self.envir_status_msg=msg

#sub
    def app_callback(self, msg):
        # 가전정보 받아옴 체크
        self.is_app_status=True
        # 가전정보 저장
        self.app_status_msg=msg  

    def app_all_on(self):
        print("on")
        for i in range(17):
            # 모든 가전 켜도록 메시지 설정
            self.app_control_msg.data[i]=1
        # 메시지를 발행해서 가전 켜기
        self.app_control_pub.publish(self.app_control_msg)
        
    def app_all_off(self):
        for i in range(17):
            self.app_control_msg.data[i]=2
        self.app_control_pub.publish(self.app_control_msg)
        
    def app_on_select(self,num):
        # 특정 가전 선택해서 켜도록 메시지 설정
        self.app_control_msg.data[num]=1
        # 메시지 발행해서 특정 가전 켜기
        self.app_control_pub.publish(self.app_control_msg)
        '''
        로직 2. 특정 가전 제품 ON
        '''

    def app_off_select(self,num):
        self.app_control_msg.data[num]=2
        self.app_control_pub.publish(self.app_control_msg)
        '''
        로직 3. 특정 가전 제품 OFF
        '''

    def turtlebot_go(self) :
        # 터틀봇의 선속도 설정
        self.cmd_msg.linear.x=0.3
        # 터틀봇의 각속도 설정
        self.cmd_msg.angular.z=0.0
        
        
         # 터틀봇의 선속도 설정
        # 터틀봇의 각속도 설정
    def turtlebot_stop(self) :
        self.cmd_msg.linear.x=0.0
        self.cmd_msg.angular.z=0.0
        
      
        '''
        로직 4. 터틀봇 정지
        '''
     # 터틀봇의 선속도 설정
        # 터틀봇의 각속도 설정
    def turtlebot_cw_rot(self) :
        
        self.cmd_msg.linear.x=0.0
        self.cmd_msg.angular.z=5.0
        
    
        
        '''
        로직 5. 터틀봇 시계방향 회전
        '''
 # 터틀봇의 선속도 설정
        # 터틀봇의 각속도 설정
    def turtlebot_ccw_rot(self) :
        
        self.cmd_msg.linear.x=0.0
        self.cmd_msg.angular.z=-5.0
        
        
        
        '''
        로직 6. 터틀봇 반시계방향 회전
        '''

    # 설정한 타이머 시간마다 반복하는 함수
    def timer_callback(self):

        '''
        로직1. 수신 데이터 출력
        터틀봇 상태 : 현재 선솏도, 현재 각속도, 배터리 상태, 충전 상태 출력
        환경 정보 : 날짜, 시간, 온도, 날씨 출력
        가전 제품 : 가전상태 출력        
        print('Sub: '+msg.data)
        '''

        self.app_all_on()
        time.sleep(1)
        self.app_all_off()
        time.sleep(1)
        self.app_on_select(11)
        time.sleep(1)
        self.app_off_select(11)
        time.sleep(1)

        ## 터틀봇 제어 함수
        self.turtlebot_go()
        self.cmd_publisher.publish(self.cmd_msg)
        time.sleep(0.5)
        self.turtlebot_stop()
        self.cmd_publisher.publish(self.cmd_msg)
        time.sleep(0.5)
        self.turtlebot_cw_rot()
        self.cmd_publisher.publish(self.cmd_msg)
        time.sleep(0.5)
        self.turtlebot_ccw_rot()
        self.cmd_publisher.publish(self.cmd_msg)
        time.sleep(0.5)

        self.cmd_publisher.publish(self.cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    sub1_controller = Controller()
    rclpy.spin(sub1_controller)
    sub1_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()