#!/ C:\Python37\python.exe

import numpy as np
# Numpy는 다차원 배열을 쉽게 처리하고 효율적으로 사용할 수 있도록지원하는 파이썬의 패키지

import cv2
# openCV를 사용하기 위한 패키지
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

# image parser 노드는 이미지를 받아서 opencv 의 imshow로 윈도우창에 띄우는 역할을 합니다.
# 이를 resize나 convert를 사용하여 이미지를 원하는대로 바꿔보세요.

# 노드 로직 순서
# 1. image subscriber 생성
# 2. 카메라 콜백함수에서 compressed image 디코딩
# 3. 이미지 색 채널을 gray scale로 컨버팅
# 4. 이미지 resizing
# 5. 이미지 imshow


class IMGParser(Node):

    def __init__(self):
        super().__init__(node_name='image_convertor')

        # 로직 1. image subscriber 생성
        ## 아래와 같이 subscriber가 
        ## 미리 정의된 토픽 이름인 '/image_jpeg/compressed' 에서
        ## CompressedImage 메시지를 받도록 설정된다.

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            10)
        

    def img_callback(self, msg):

        # 로직 2. 카메라 콜백함수에서 이미지를 클래스 내 변수로 저장
        ## msg.data 는 bytes로 되어 있고 이를 uint8로 바꾼 다음
        ## cv2 내의 이미지 디코딩 함수로 bgr 이미지로 바꾸세요.        

        np_arr = np.frombuffer(msg.data, np.uint8)
        # data는 이미지 데이터를 나타내는 bytes 형식의 데이터입니다. 
        # 이 데이터를 np.uint8 자료형으로 변환하여 NumPy 배열로 읽어옵니다.


        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # cv2.imdecode 함수를 사용하여 np_arr로부터 BGR 색상 형식의 이미지로 디코딩합니다.
        # 이렇게 하면 OpenCV에서 다룰 수 있는 이미지 형식으로 변환됩니다.


        
        '''
        로직 3. 이미지 색 채널을 gray scale로 컨버팅
        cv2. 내의 이미지 색 채널 컨터버로 bgr 색상을 gary scale로 바꾸십시오.

        '''
        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        # cv2.cvtColor 함수를 사용하여 BGR 이미지를 그레이스케일 이미지로 변환합니다. 
        # 이 과정을 통해 이미지의 색상 채널이 제거되고 흑백 이미지가 생성됩니다.




        '''
        로직 4. 이미지 resizing
        cv2를 사용해서 이미지를 원하는 크기로 바꿔보십시오.
        '''
        img_resize = cv2.resize(img_bgr, (256, 256))
        # cv2.resize 함수를 사용하여 이미지 크기를 조정합니다. 
        # width와 height는 원하는 크기로 설정하셔야 합니다. 이를 통해 이미지의 크기가 조정됩니다.



        # 로직 5. 이미지 출력 (cv2.imshow)       
        
        # cv2.imshow("img_bgr", img_bgr)
        # "img_bgr"라는 창 이름으로 사진을 띄움
        cv2.imshow("img_gray", img_gray)
        # cv2.imshow("resize and gray", img_resize)       
        
        cv2.waitKey(1)
        # cv2.imshow 함수로 열린 창을 보여주기 위한 대기 시간을 설정합니다. 
        # 1밀리초 동안 대기하고, 사용자 입력을 처리합니다. 이 함수를 사용하여 이미지 창이 업데이트되는 주기를 조정할 수 있습니다.




def main(args=None):

    ## 노드 초기화 : rclpy.init 은 node의 이름을 알려주는 것으로, ROS master와 통신을 가능하게 해줍니다.
    rclpy.init(args=args)

    ## 메인에 돌릴 노드 클래스 정의 
    image_parser = IMGParser()

    ## 노드 실행 : 노드를 셧다운하기 전까지 종료로부터 막아주는 역할을 합니다
    rclpy.spin(image_parser)


if __name__ == '__main__':

    main()