import rclpy
from rclpy.node import Node
import os, time
from ssafy_msgs.msg import TurtlebotStatus,HandControl

# Hand Control 노드는 시뮬레이터로부터 데이터를 수신해서 확인(출력)하고, 메세지를 송신해서 Hand Control기능을 사용해 보는 노드입니다. 
# 메시지를 받아서 Hand Control 기능을 사용할 수 있는 상태인지 확인하고, 제어 메시지를 보내 제어가 잘 되는지 확인해보세요. 
# 수신 데이터 : 터틀봇 상태 (/turtlebot_status)
# 송신 데이터 : Hand Control 제어 (/hand_control)


# 노드 로직 순서
# 1. publisher, subscriber 만들기
# 2. 사용자 메뉴 구성
# 3. Hand Control Status 출력
# 4. Hand Control - Preview
# 5. Hand Control - Pick up
# 6. Hand Control - Put down


class Handcontrol(Node):

    def __init__(self):
        super().__init__('hand_control')
                
        ## 로직 1. publisher, subscriber 만들기
        self.hand_control = self.create_publisher(HandControl, '/hand_control', 10)    

# /HandControl
# control_mode 제어 모드
# 0 : 노말 모드
# 1 : 프리뷰 모드
# 2 : 들기
# 3 : 놓기
# put_distance 물건을 놓을 거리(x축) m
# put_height 물건을 놓을 높이(z축) m            
        self.turtlebot_status = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.turtlebot_status_cb,10)
        
# twist 속도 데이터
# linear.x : 선속도(m/s)
# angular.z : 각속도(rad/s)
# linear.z : 로봇의 헤딩(rad)
# angluar.x : 로봇의 절대 좌표x(m)
# angluar.y : 로봇의 절대 좌표y(m)
# battery_percentage 터블봇 배터리 퍼센트 0~100
# power_supply_status 충전 상태
# 1 : 충전중
# 2 : 충전중이지 않음
# 3 : 충전완료
# can_lift 물건을 내려 놓을 수 있는지 여부 True or False
# can_put 물건을 놓을 수 있는지 여부 True or False
# can_use_hand 현재 물건을 들고 있는지 여부 True or False

        self.timer = self.create_timer(1, self.timer_callback)
        
        ## 제어 메시지 변수 생성 
        self.hand_control_msg=HandControl()        


        self.turtlebot_status_msg = TurtlebotStatus()
        self.is_turtlebot_status = False
        
        

    def timer_callback(self):
        # while True:
            # 로직 2. 사용자 메뉴 구성
            print('Select Menu [0: status_check, 1: preview, 2:pick_up, 3:put_down')
            menu=input(">>")
            # 사용자로부터 입력을 받는 부분입니다. 사용자가 선택할 메뉴 번호를 입력하도록 유도하고, 입력된 값을 menu 변수에 저장합니다.
            if menu=='0' :               
                self.hand_control_status()
            if menu=='1' :
                self.hand_control_preview()               
            if menu=='2' :
                self.hand_control_pick_up()   
            if menu=='3' :
                self.hand_control_put_down()


    def hand_control_status(self):
        '''
        로직 3. Hand Control Status 출력
        '''
        print('can_lift의 상태는 {}'.format(self.turtlebot_status_msg.can_lift))
        print('can_put의 상태는 {}'.format(self.turtlebot_status_msg.can_put))
        print('can_use_hand의 상태는 {}'.format(self.turtlebot_status_msg.can_use_hand))
        
        
        
        

    def hand_control_preview(self):
        '''
        로직 4. Hand Control - Preview
        '''
        print('프리뷰 모드')
        self.hand_control_msg.control_mode = 1
        
        
    
        self.hand_control.publish(self.hand_control_msg)

    def hand_control_pick_up(self):
        '''
        로직 5. Hand Control - Pick up        
        '''
        
        print('들수 있음')
        self.hand_control_msg.control_mode = 2
        self.hand_control.publish(self.hand_control_msg)
       
        
        
        
        
        
    def hand_control_put_down(self):        
        ''' 
        로직 6. Hand Control - Put down
        '''

        print('내려놓기 모드 가능')
        self.hand_control_msg.put_distance = 1.0
        self.hand_control_msg.put_height = 0.0
        self.hand_control_msg.control_mode = 3
        
        self.hand_control.publish(self.hand_control_msg)
          
        


    def turtlebot_status_cb(self,msg):
        self.is_turtlebot_status=True
        self.turtlebot_status_msg=msg
        

def main(args=None):
    rclpy.init(args=args)
    sub1_hand_control = Handcontrol()    
    rclpy.spin(sub1_hand_control)
    sub1_hand_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()