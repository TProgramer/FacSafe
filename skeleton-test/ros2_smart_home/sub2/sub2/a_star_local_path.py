import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path
from math import pi,cos,sin,sqrt

# a_star_local_path 노드는 a_star 노드에서 나오는 전역경로(/global_path)를 받아서, 로봇이 실제 주행하는 지역경로(/local_path)를 publish 하는 노드입니다.
# path_pub 노드와 하는 역할은 비슷하나, path_pub은 텍스트를 읽어서 global_path를 지역경로를 생성하는 반면, a_star_local_path는 global_path를 다른 노드(a_star)에서 받아서 지역경로를 생성합니다.


# 노드 로직 순서
# 1. publisher, subscriber 만들기
# 2. global_path 데이터 수신 후 저장
# 3. 주기마다 실행되는 타이머함수 생성, local_path_size 설정
# 4. global_path 중 로봇과 가장 가까운 포인트 계산
# 5. local_path 예외 처리


class astarLocalpath(Node):

    def __init__(self):
        super().__init__('a_star_local_path')
        # 로직 1. publisher, subscriber 만들기
        self.local_path_pub = self.create_publisher(Path, 'local_path', 10)    # 로컬 경로 메시지를 발행하기 위한 퍼블리셔
        self.subscription = self.create_subscription(Path,'/global_path',self.path_callback,10)  # 전역 경로 메시지 수신
        self.subscription = self.create_subscription(Odometry,'/odom',self.listener_callback,10) # 오도메트리 메시지 수신
        self.odom_msg=Odometry()   # 오도메트리 메시지를 저장하기 위한 변수 생성
        self.is_odom=False  # odom 메시지를 수신했는지 나타내는 플래그 변수
        self.is_path=False  # global_path 메시지를 수신했는지 나타내는 플래그 변수

        self.global_path_msg=Path()  # global_path 메시지를 저장하기 위한 변수 생성


        # 로직 3. 주기마다 실행되는 타이머함수 생성, local_path_size 설정
        time_period=0.05 
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.local_path_size=30   # 로컬 경로의 크기 30할당
        self.count=0  # count변수 0으로 초기화
        self.local_path_msg=Path()
        self.current_waypoint=-1
        self.min_dis = float('inf')  # 무한대로 초기화


    def listener_callback(self,msg):
        self.is_odom=True  # Odometry 데이터(로봇의 현재 위치와 자세)를 수신했음
        self.odom_msg=msg  # odom_msg 변수에 수신된 Odometry 메시지 저장


    def path_callback(self,msg):
    
        # 로직 2. global_path 데이터 수신 후 저장

        self.is_path=True  # 경로 메시지 수신 여부를 True로 설정
        self.global_path_msg=msg  # 수신한 메시지를 클래스 변수에 저장
        
    def timer_callback(self):

        if self.is_odom and self.is_path ==True:   # Odometry와 global_path 둘 다 있어야 실행 

            print(self.global_path_msg)   # 테스트용 출력
            
            self.local_path_msg.header.frame_id='/map'
            
            x=self.odom_msg.pose.pose.position.x
            y=self.odom_msg.pose.pose.position.y
            
            self.current_waypoint=-1
            
            # 로직 4. global_path 중 로봇과 가장 가까운 포인트 계산
            
            self.min_dis = float('inf')  # 무한대로 초기화


        for i,waypoint in enumerate(self.global_path_msg.poses) :    
                distance=sqrt((waypoint.pose.position.x - x) ** 2 + (waypoint.pose.position.y - y) ** 2)      # 현재 위치(x, y)와 각 웨이포인트 사이의 거리 계산
                if distance < self.min_dis :
                    self.min_dis=distance
                    self.current_waypoint = i
						# 모든 웨이포인트를 반복하면서 현재 위치 (x, y)와 각 웨이포인트의 거리를 계산하여 min_dis와 비교
            # 가장 가까운 웨이포인트의 인덱스를 current_waypoint에 저장
            
            # 로직 5. local_path 예외 처리
		# 로봇의 현재 위치와 가장 가까운 글로벌 웨이포인트 기반으로 로컬 경로를 생성
        if self.current_waypoint != -1 :    # current_waypoint가 -1이 아닌 경우(유효한 가장 가까운 웨이포인트를 찾은 경우)
            
            
            #  순환경로인경우
            if(self.global_path_msg.poses[0].pose.position.x==self.global_path_msg.poses[-1].pose.position.x and self.global_path_msg.poses[0].pose.position.y==self.global_path_msg.poses[-1].pose.position.y) :
            
            
                if self.current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                        # 로봇의 현재 위치와 가장 가까운 글로벌 웨이포인트 기반으로 로컬 경로를 생성
                        self.local_path_msg.poses = self.global_path_msg.poses[self.current_waypoint:self.current_waypoint + self.local_path_size]
                else:
                    # 로봇이 글로벌 경로의 끝에 도달한 경우
                    remaining_poses = self.local_path_size - (len(self.global_path_msg.poses) - self.current_waypoint)
                    self.local_path_msg.poses = self.global_path_msg.poses[self.current_waypoint:] + self.global_path_msg.poses[:remaining_poses]
            
            # 비순환경로인경우
            else :
                
                if self.current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                    self.local_path_msg.poses = self.global_path_msg.poses[self.current_waypoint:self.current_waypoint + self.local_path_size]
                    
                
                else :    # (로봇이 글로벌 경로의 끝에 도달한 경우)
                    self.local_path_msg.poses = self.global_path_msg.poses[self.current_waypoint:]

        self.local_path_pub.publish(self.local_path_msg)   # 로컬 경로를 발행
        
        
def main(args=None):
    rclpy.init(args=args)

    a_star_local = astarLocalpath()

    rclpy.spin(a_star_local)

    a_star_local.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()