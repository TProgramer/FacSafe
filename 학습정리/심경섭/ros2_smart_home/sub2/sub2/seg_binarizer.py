#!/ C:\Python37\python.exe

import numpy as np
import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage


# segmentation image object detection node의 전체 로직 순서
# 1. 노드에 필요한 publisher, subscriber, timer 정의
# 2. compressed image 받기
# 3. bgr 이미지의 binarization
# 4. 물체의 contour 찾기
# 5. 물체의 bounding box 좌표 찾기
# 6. 물체의 bounding box 가 그려진 이미지 show


class IMGParser(Node):

    def __init__(self):
        super().__init__(node_name='image_parser')

        # 로직 1. 노드에 필요한 publisher, subscriber, timer 정의
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            10)

        #초기 이미지를 None으로 초기화한다
        self.img_bgr = None

        self.timer_period = 0.03

        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        

    def img_callback(self, msg):

        # 로직 2. compressed image 받기
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


    def find_bbox(self):

        """
        
        
        lower_wal = 
        upper_wal = 
        lower_bp = 
        upper_bp = 
        lower_rc = 
        upper_rc = 
        lower_key = 
        upper_key = 

        self.img_wal = cv2.inRange(self.img_bgr, lower_wal, upper_wal)

        self.img_bp = cv2.inRange(self.img_bgr, lower_bp, upper_bp)

        self.img_rc = cv2.inRange(self.img_bgr, lower_rc, upper_rc)

        self.img_key = cv2.inRange(self.img_bgr, lower_key, upper_key)

        """
        # 로직 3. bgr 이미지의 binarization
        # 지갑, 키 등의 물체에 대한 bgr 값을 알고, 이 값 범위에 해당되는
        # cv2.inRange 함수를 써서 각 물체에 대해 binarization 하십시오.
        lower_wal = np.array([0, 0, 0])
        upper_wal = np.array([255, 255, 255])
        lower_bp = np.array([0, 0, 0])
        upper_bp = np.array([255, 255, 255])
        lower_rc = np.array([0, 0, 0])
        upper_rc = np.array([255, 255, 255])
        lower_key = np.array([0, 0, 0])
        upper_key = np.array([255, 255, 255])

        self.img_wal = cv2.inRange(self.img_bgr, lower_wal, upper_wal)

        self.img_bp = cv2.inRange(self.img_bgr, lower_bp, upper_bp)

        self.img_rc = cv2.inRange(self.img_bgr, lower_rc, upper_rc)

        self.img_key = cv2.inRange(self.img_bgr, lower_key, upper_key)

        """
        # 로직 4. 물체의 contour 찾기
        # 지갑, 키 등의 물체들이 차지한 픽셀만 흰색으로 이진화되어 있는 이미지에 대해서,
        # 흰색 영역을 감싸는 contour들을 구하십시오.
        # cv2.findContours를 가지고 

        contours_wal, _ = cv2.findContours( .....

        contours_bp, _ = 

        contours_rc, _ = 

        contours_key, _ = 

        """
        # 로직 4. 물체의 contour 찾기
        # 지갑, 키 등의 물체들이 차지한 픽셀만 흰색으로 이진화되어 있는 이미지에 대해서,
        # 흰색 영역을 감싸는 contour들을 구하십시오.
        # cv2.findContours를 가지고 

        # RETR_EXTERNAL: 윤곽선을 검출할 때 계층 구조를 무시하고 가장 바깥쪽의 윤곽선만 검출 즉, 내부의 구멍이나 하위 윤곽선은 무시
        # CHAIN_APPROX_SIMPLE: 윤곽선을 검출한 후에 윤곽선의 꼭지점 좌표만 저장합니다. 
        # 이것은 꼭지점만 필요한 경우, 불필요한 중간점을 제거하여 메모리를 절약
        contours_wal, _ = cv2.findContours(self.img_wal, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contours_bp, _ = cv2.findContours(self.img_bp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contours_rc, _ = cv2.findContours(self.img_rc, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contours_key, _ = cv2.findContours(self.img_key, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

      
        # 로직 5. 물체의 bounding box 좌표 찾기
        
        self.find_cnt(contours_wal)
        
        self.find_cnt(contours_bp)
        
        self.find_cnt(contours_rc)
        
        self.find_cnt(contours_key)

        

    def find_cnt(self, contours):

        """
        # 로직 5. 물체의 bounding box 좌표 찾기
        # 지갑, 키 등의 물체들의 흰색 영역을 감싸는 contour 결과를 가지고
        # bbox를 원본 이미지에 draw 하십시오.
        
        for cnt in contours:
    
            x, y, w, h = 

            cv2.rectangle( ... )

        """     
        
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            # 이미지에 그림을 그리는데 왼쪽 상단, 우측하단, 색상, 두께 를 짖어하여 사각형을 그린다.
            cv2.rectangle(self.img_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
        


    def timer_callback(self):

        if self.img_bgr is not None:
            
            # 이미지가 ros subscriber로 받고 None이 아닌 array로 정의됐을 때,
            # object에 대한 bbox 추정을 시작.             
            self.find_bbox()

            # 로직 6. 물체의 bounding box 가 그려진 이미지 show
            cv2.imshow("seg_results", self.img_bgr)

            cv2.waitKey(1)
        else:
            pass

        
def main(args=None):

    rclpy.init(args=args)

    image_parser = IMGParser()

    rclpy.spin(image_parser)


if __name__ == '__main__':

    main()