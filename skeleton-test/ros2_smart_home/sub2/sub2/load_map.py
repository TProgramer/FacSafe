import rclpy
import numpy as np
from rclpy.node import Node

import os
from geometry_msgs.msg import Pose
from squaternion import Quaternion
from nav_msgs.msg import Odometry,OccupancyGrid,MapMetaData
from math import pi

# load_map 노드는 맵 데이터를 읽어서, 
# 맵 상에서 점유영역(장애물) 근처에 로봇이 움직일 수 없는 영역을 설정하고 
# 맵 데이터로 publish 해주는 노드입니다. (맵, 장애물인식 정보 발송) 

# 추 후 a_star 알고리즘에서 맵 데이터를 subscribe 해서 사용합니다.

# 노드 로직 순서
# 1. 맵 파라미터 설정
# 2. 맵 데이터 읽고, 2차원 행렬로 변환
# 3. 점유영역 근처 필터처리

class loadMap(Node):

    def __init__(self):
        super().__init__('load_map')
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 1)   # map 토픽을 발행할거임
        
        time_period=1  
        self.timer = self.create_timer(time_period, self.timer_callback)
				# 1초마다 콜백함수 호출

        # 로직 1. 맵 파라미터 설정
        # 제공한 맵 데이터의 파라미터입니다. 
				# size_x,y는 x,y 방향으로 grid의 개수이고, resolution은 grid 하나당 0.05m라는 것을 의미합니다.
        # offset_x,y 의 -8, -4 는 맵 데이터가 기준 좌표계(map)로 부터 떨어진 거리를 의미합니다. 
        # 각 항에 -8.75를 뺀이유는 ros에서 occupancygrid의 offset이라는 데이터는 맵의 중앙에서 기준좌표계까지 거리가 아니라 맵의 우측하단에서 부터 기준좌표계까지의 거리를 의미합니다.
        # 따라서 (350*0.05)/2를 해준 값을 빼줍니다.
        self.map_msg=OccupancyGrid()
        self.map_size_x=350     # 맵이 그럼 17.5m 인건가 
        self.map_size_y=350
        self.map_resolution=0.05
        self.map_offset_x=-16.75     # offset값 : 기준 좌표계(맵의 중심)와 오차
        self.map_offset_y=-12.75
        self.map_data = [0 for i in range(self.map_size_x*self.map_size_y)]  # 맵에 있는 그리드를 다 읽어서
        grid=np.array(self.map_data)    # 데이터 처리를 위해 NumPy배열로 변환
        grid=np.reshape(grid,(350, 350))  # 350x350 크기의 2차원배열로 재구성

        self.map_msg.header.frame_id="map"   
				# self.map_msg의 header 속성에 있는 frame_id 값을 "map"으로 설정합니다.
				# 지도 메시지가 어떤 좌표 프레임에서 정의되었는지를 나타냄

        m = MapMetaData()
        m.resolution = self.map_resolution   # map_resolution : 해상도 정보
        m.width = self.map_size_x         # width, height : 지도의 가로 및 세로 크기 정보 지정
        m.height = self.map_size_y
        m.origin = Pose()            # origin : 지도의 원점에 대한 정보
        m.origin.position.x = self.map_offset_x
        m.origin.position.y = self.map_offset_y

        self.map_meta_data = m
        self.map_msg.info=self.map_meta_data  # map_msg에 데이터 담기 
        
        
        # 로직 2. 맵 데이터 읽고, 2차원 행렬로 변환

        full_path="C:/Users/SSAFY/Desktop/project_git/S09P22A101/skeleton-test/ros2_smart_home/sub2/map/map.txt" 
        self.f = open(full_path, 'r')
        line = self.f.read()      # 맵 파일 열기
        line_data = line.split()   # 공백제거하고 line_data 리스트에 저장

        for num, data in enumerate(line_data):
            self.map_data[num] = int(data)

        map_to_grid = np.array(self.map_data)    # 2차원 배열로 self.map_data 작성
        grid = np.reshape(map_to_grid, (350, 350))


        for y in range(350):        # 100인 지점 주변의 값을 어떻게 처리할지에 대한 코드
            for x in range(350):
                if grid[x][y]==100 :

                    
                    # 로직 3. 점유영역 근처 필터처리    # 일단 주변좌표를 0으로 바꿈

                    if x > 0:
                        grid[x-1][y] = 127  # 왼쪽의 값을 변경
                    if x < 349:
                        grid[x+1][y] = 127  # 오른쪽의 값을 변경
                    if y > 0:
                        grid[x][y-1] = 127  # 위쪽의 값을 변경
                    if y < 349:
                        grid[x][y+1] = 127  # 아래쪽의 값을 변경


        
        np_map_data=grid.reshape(1,350*350)    # 2차원 배열을 1차원 배열로 변환
        list_map_data=np_map_data.tolist()     # NumPy 배열 np_map_data를 Python 리스트로 변환

        ## 로직2를 완성하고 주석을 해제 시켜주세요.
        self.f.close()      # self.f 파일을 닫고
        print('read_complete')    # 파일 읽기 작업이 완료되었음을 표시
        self.map_msg.data=list_map_data[0]   # map_msg.dat에 list_map_data[0] 할당


    def timer_callback(self):        # 메시지 타임스탬프 설정하고, 발행하기(메시지 전송)
        self.map_msg.header.stamp =rclpy.clock.Clock().now().to_msg()  # 현재 시간을 가져와서 self.map_msg 메시지의 header.stamp에 설정
        self.map_pub.publish(self.map_msg)   # self.map_pub : 메시지를 발행하는 ROS Publisher 객체


def main(args=None):
    rclpy.init(args=args)    # ROS 2 노드를 초기화

    load_map = loadMap()     # 클래스의 인스턴스를 생성
    rclpy.spin(load_map)     # spin 함수 : ROS 메시지 및 이벤트를 처리하고, 콜백 함수를 호출하여 노드의 주요 작업을 수행
    load_map.destroy_node()    # destroy_node : 노드가 종료되면 노드를 정리하고 노드 자원을 해제
    rclpy.shutdown()         # ROS 2 라이브러리를 종료

    # 스핀 함수를 호출하면 이후의 코드는 노드가 계속 실행될 때까지 블록된다.


if __name__ == '__main__':
    main()