import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Float32MultiArray
from ssafy_msgs.msg import BBox

# human detector node의 전체 로직 순서
# 로직 1 : 노드에 필요한 publisher, subscriber, descriptor, detector 정의
# 로직 2 : image binarization
# 로직 3 : human detection 실행 후 bounding box 출력
# 로직 4 : non maximum supresion으로 bounding box 정리
# 로직 5 : bbox를 ros msg 파일에 write
# 로직 6 : bbox를 원본 이미지에 draw
# 로직 7 : bbox 결과 show
# 로직 8 : bbox msg 송신

# Non-Maximum Suppression을 수행하여 중복된 BBox를 제거하는 함수
# threshold는 중복 여부를 결정하는 임계값
def non_maximum_supression(bboxes, threshold=0.5):

    """
    non maximum supression 로직
    로직 1 : bounding box 크기 역순으로 sort
    로직 2 : new_bboxes 리스트 정의 후 첫 bbox save
    로직 3 : 기존 bbox 리스트에 첫 bbox delete
    로직 4 : 두 bbox의 겹치는 영역을 구해서, 영역이 안 겹칠때 new_bbox로 save
    """    
    # 로직 1 : bounding box 크기 역순으로 sort   
    # key=lambda detections: detections[3]의 뜻은 정렬 기준을 정하는 것으로
    # 람다 함수를 사용하여 detections내의 요소들에 대해서 3번째 인덱스를 기준으로 정렬한다는 뜻
    bboxes = sorted(bboxes, key=lambda detections: detections[3],
            reverse=True)
    new_bboxes=[]
    
    # 로직 2 : new_bboxes 리스트 정의 후 첫 bbox save
    new_bboxes.append(bboxes[0])
    
    # 로직 3 : 기존 bbox 리스트에 첫 bbox delete
    bboxes.pop(0)

    # enumerate를 사용할 때에는 원소와 인덱스가 튜플형태로 담긴다. 
    # 현재의 반복문에서는 인덱스가 필요 없어서 _,으로 정의했다.
    for _, bbox in enumerate(bboxes):

        for new_bbox in new_bboxes:


            # 아래의 코드부터는 두 개의 바운딩 박스(BBox)의 좌표를 비교하여 겹치는 영역을 찾기 위한 작업
            # x_tl: BBox의 왼쪽 상단 꼭지점의 x 좌표
            # x_br: BBox의 오른쪽 하단 꼭지점의 x 좌표
            # y_tl: BBox의 왼쪽 상단 꼭지점의 y 좌표
            # y_br: BBox의 오른쪽 하단 꼭지점의 y 좌표
            # 0번 인덱스: 왼쪽 상단 꼭지점의 x 좌표
            # 1번 인덱스: 왼쪽 상단 꼭지점의 y 좌표
            # 2번 인덱스: BBox의 너비 (x 좌표 간의 거리)
            # 3번 인덱스: BBox의 높이 (y 좌표 간의 거리)
            
            x1_tl = bbox[0]
            x2_tl = new_bbox[0]
            x1_br = bbox[0] + bbox[2]
            x2_br = new_bbox[0] + new_bbox[2]
            y1_tl = bbox[1]
            y2_tl = new_bbox[1]
            y1_br = bbox[1] + bbox[3]
            y2_br = new_bbox[1] + new_bbox[3]
            
            """
            # 로직 4 : 두 bbox의 겹치는 영역을 구해서, 영역이 안 겹칠때 new_bbox로 save
            x_overlap = 
            y_overlap = 
            overlap_area = x_overlap * y_overlap
            
            area_1 = 
            area_2 = 
            
            total_area = area_1 + area_2 - overlap_area
            overlap_area = overlap_area / float(total_area)

            if overlap_area < threshold:

                new_bboxes.append(bbox)

            """
            
            # 그림으로 설명하는 것이 좋아보임
            # 겹치는 영역의 정의
            x_overlap = max(0, min(x1_br, x2_br) - max(x1_tl, x2_tl))
            y_overlap = max(0, min(y1_br, y2_br) - max(y1_tl, y2_tl))
            overlap_area = x_overlap * y_overlap

            area_1 = bbox[2] * bbox[3] # 기존 박스의 크기
            area_2 = new_bbox[2] * new_bbox[3] # 새로운 박스의 크기
            total_area = area_1 + area_2 - overlap_area # 박스 1과 박스 2가 존재하는 영역
            overlap_area = overlap_area / float(total_area) # overlap_area가 전체 영역(박스1과 박스2) 중 차지하는 비율

            if overlap_area < threshold: # 만약 임계치 보다 작을 경우 새로운 박스
                new_bboxes.append(bbox)

    return new_bboxes



class HumanDetector(Node):

    def __init__(self):
        super().__init__(node_name='human_detector')

        # 로직 1 : 노드에 필요한 publisher, subscriber, descriptor, detector, timer 정의
        self.subs_img = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            1)

        self.img_bgr = None
        
        self.timer_period = 0.03

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.bbox_pub_ = self.create_publisher(BBox, '/bbox', 1)

        # Histogram of Oriented Gradients(HOG)
        
        # HOG 디스크립터를 초기화합니다. 
        # HOG 디스크립터는 이미지에서 특징을 추출하는 방법 중 하나로, 주로 보행자 검출과 같은 객체 감지 작업에 사용됩니다
        self.pedes_detector = cv2.HOGDescriptor()                              
        # setSVMDetector 함수는 HOG 디스크립터에 SVM(Support Vector Machine) 분류기를 설정 
        # SVM은 객체를 감지하기 위한 분류 알고리즘 중 하나로, 훈련된 SVM 모델을 사용하여 이미지에서 객체를 감지하는 데 사용
        self.pedes_detector.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self.able_to_pub = True

    def img_callback(self, msg):

        # data는 이미지 데이터를 나타내는 bytes 형식의 데이터입니다. 
        # 이 데이터를 np.uint8 자료형으로 변환하여 NumPy 배열로 읽어옵니다.
        np_arr = np.frombuffer(msg.data, np.uint8)

        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
  
    def detect_human(self, img_bgr):
    
        self.bbox_msg = BBox()
    
        # 로직 2 : image grayscale conversion
        img_pre = cv2.cvtColor(self.img_bgr, cv2.COLOR_BGR2GRAY)

        # 로직 3 : human detection 실행 후 bounding box 출력
        
        # rects_temp: 보행자 감지 결과인 박스(BBox)의 목록을 저장할 변수 
        
        # self.pedes_detector.detectMultiScale(): HOG 디스크립터를 사용하여 이미지에서 
        #                               다중 스케일로 객체(보행자)를 감지하는 함수입니다
        
        # img_pre은 보행자를 검출할 이미지. 해당 이미지는 그레이 스케일이여야 함(효율)
        
        # winStride=(2, 2): 슬라이딩 윈도우를 이동시킬 때 픽셀 단위로 이동할 크기를 지정합니다. 
        # (이미지를 작은 윈도우로 나눈 후 윈도우를 이동시키며 객체 탐지)
        # 여기에서는 (2, 2) 픽셀 단위로 윈도우를 이동시키라는 의미
        
        # padding=(8, 8): 입력 이미지 주위에 패딩을 추가하는 것으로, 이것은 윈도우가 이미지의 가장자리에 닿지 않도록 하는 데 사용됩니다. 
        # (8, 8)은 패딩의 크기를 지정하는 것으로, 위쪽과 아래쪽에 각각 8픽셀, 왼쪽과 오른쪽에도 각각 8픽셀의 패딩을 추가하라는 의미
        
        # cale=2: 이미지 피라미드(scale pyramid)의 스케일(factor)을 지정합니다. 이미지 피라미드는 이미지를 다양한 크기로 변환하여 
        # 다중 스케일에서 객체를 검출하는 데 사용됩니다. 
        # scale=2는 이미지를 2배 확대하여 검출하라는 의미
        (rects_temp, _) = self.pedes_detector.detectMultiScale(img_pre, winStride=(2, 2), padding=(8, 8), scale=2)

        if len(rects_temp) != 0:

            # 로직 4 : non maximum supression으로 bounding box 정리
            # 각각의 변수에 리스트 정의
            xl, yl, wl, hl = [], [], [], []
            rects = non_maximum_supression(rects_temp)

            """
    
            # 로직 5 : bbox를 ros msg 파일에 write

            ## 각 bbox의 center, width, height의 꼭지점들을 리스트에 넣어 놓고
            ## 메세지 내 x,y,w,h에 채워넣는 방식으로 하시면 됩니다.
           
            for (x,y,w,h) in rects:
    
                xl.append(x)
                yl.append(y)
                wl.append(w)
                hl.append(h)

            if self.able_to_pub:

                self.bbox_msg.num_bbox = 

                obj_list = 

                self.bbox_msg.idx_bbox = 

                self.bbox_msg.x = 
                self.bbox_msg.y = 
                self.bbox_msg.w = 
                self.bbox_msg.h = 

            """
            ## 각 bbox의 center, width, height의 꼭지점들을 리스트에 넣어 놓고
            ## 메세지 내 x,y,w,h에 채워넣는 방식으로 하시면 됩니다.
           
            for (x,y,w,h) in rects:
    
                xl.append(x)
                yl.append(y)
                wl.append(w)
                hl.append(h)

            if self.able_to_pub:

                self.bbox_msg.num_bbox = len(rects)  # 검출된 BBox 개수


                # 여기에 들어갈 값이 없어 보임...?
                obj_list = []  # BBox 정보를 담을 리스트 초기화 

                self.bbox_msg.idx_bbox = 0

                self.bbox_msg.x = xl
                self.bbox_msg.y = yl
                self.bbox_msg.w = wl
                self.bbox_msg.h = hl

            for (x,y,w,h) in rects:

                cv2.rectangle(img_bgr, (x,y),(x+w,y+h),(0,255,255), 2)

        else:
            # pass
            self.bbox_msg.num_bbox = len(rects_temp)


        """
        로직 7 : bbox 결과 show
        cv2.
        cv2.waitKey(1)
        """           
        cv2.imshow("detection result", img_bgr)        
        cv2.waitKey(1)

    def timer_callback(self):

        if self.img_bgr is not None:

            self.detect_human(self.img_bgr)

            # 로직 8 : bbox msg 송신s
            self.bbox_pub_.publish(self.bbox_msg)

        else:
            pass

def main(args=None):

    rclpy.init(args=args)

    hdetector = HumanDetector()

    rclpy.spin(hdetector)

if __name__ == '__main__':

    main()

