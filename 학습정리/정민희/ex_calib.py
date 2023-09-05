import numpy as np
import cv2
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan

params_lidar = {
    "Range" : 90, #min & max range of lidar azimuths
    "CHANNEL" : int(1), #verticla channel of a lidar
    "localIP": "127.0.0.1",
    "localPort": 2368,
    "Block_SIZE": int(1206),
    "X": 0, # meter
    "Y": 0,
    "Z": 0.4+0.1,
    "YAW": 0, # deg
    "PITCH": 0,
    "ROLL": 0
}


params_cam = {
    "WIDTH": 320, # image width
    "HEIGHT": 240, # image height
    "FOV": 60, # Field of view
    "localIP": "127.0.0.1",
    "localPort": 1232,
    "Block_SIZE": int(65000),
    "X": 0., # meter
    "Y": 0,
    "Z":  0.8,
    "YAW": 0, # deg
    "PITCH": 0.0,
    "ROLL": 0
}

# ex 노드 설명
# 로봇에 달려있는 라이다와 카메라 간의 위치 및 자세 정보를 위의 params_lidar, params_cam으로
# 받은 다음, 이를 가지고 좌표 변환 행렬을 만들고, 카메라 이미지에 라이다 포인트들을 projection
# 하는 노드입니다.
# 2d 공간만 표현되는 카메라는 3d 위치정보를 포함하지 않기 때문에,
# 라이다의 포인트들을 프레임에 정사영시켜, 카메라 내 객체들의 위치 정보를 추정하도록 만들 수
# 있습니다.

# 노드 로직 순서
# 1. 노드에 필요한 라이다와 카메라 topic의 subscriber 생성
# 2. Params를 받아서 라이다 포인트를 카메라 이미지에 projection 하는 transformation class 정의하기
# 3. 카메라 콜백함수에서 이미지를 클래스 내 변수로 저장.
# 4. 라이다 콜백함수에서 2d scan data(거리와 각도)를 가지고 x,y 좌표계로 변환
# 5. 라이다 x,y 좌표 데이터 중 정면 부분만 crop
# 6. transformation class 의 transform_lidar2cam로 라이다 포인트를 카메라 3d좌표로 변환
# 7. transformation class 의 project_pts2img 로 라이다 포인트를 2d 픽셀 좌표상으로 정사영
# 8. draw_pts_img()로 카메라 이미지에 라이다 포인트를 draw 하고 show


# 좌표변환을 하는데 필요한 rotation, translation 행렬을 아래와 같이 완성시켜 놓았습니다. 
# 이를 활용하여 라이다 scan 포인트들을 이미지 프레임 상으로 변환시켜주는 클래스인 
# LIDAR2CAMTransform 를 완성시키십시오.

def rotationMtx(yaw, pitch, roll):     # 회전 행렬(Rotation Matrix) : Euler 각도(yaw, pitch, roll)를 사용, 3D 공간에서 오브젝트를 회전하는 데 사용
		#  roll : x축 주위의 회전
		#  pitch: y축 주위의 회전
		#  yaw  : z축 주위의 회전

    R_x = np.array([[1,         0,              0,                0],
                    [0,         math.cos(roll), -math.sin(roll) , 0],
                    [0,         math.sin(roll), math.cos(roll)  , 0],
                    [0,         0,              0,               1],
                    ])
                     
    R_y = np.array([[math.cos(pitch),    0,      math.sin(pitch) , 0],
                    [0,                  1,      0               , 0],
                    [-math.sin(pitch),   0,      math.cos(pitch) , 0],
                    [0,         0,              0,               1],
                    ])
    
    R_z = np.array([[math.cos(yaw),    -math.sin(yaw),    0,    0],
                    [math.sin(yaw),    math.cos(yaw),     0,    0],
                    [0,                0,                 1,    0],
                    [0,         0,              0,               1],
                    ])
                     
    R = np.matmul(R_x, np.matmul(R_y, R_z))
		# 두 회전 행렬을 np.matmul 함수로 곱하면 이 두 회전이 연속으로 적용되어 새로운 회전 행렬이 생성
		# 두 회전을 동시에 적용하는 것과 같음.(인자순서대로 적용)
    # y 축 주위의 회전(R_y)을 먼저 적용 -> z 축 주위의 회전(R_z)을 적용

    return R

def translationMtx(x, y, z):    # 3차원 공간에서의 이동 변환 행렬을 생성(x, y, z 좌표만큼 오브젝트를 이동)
     
    M = np.array([[1,         0,              0,               x],
                  [0,         1,              0,               y],
                  [0,         0,              1,               z],
                  [0,         0,              0,               1],
                  ])
    
    return M



def transformMTX_lidar2cam(params_lidar, params_cam):

  
    # transformMTX_lidar2cam 내 좌표 변환행렬 로직 순서
    # 1. params에서 라이다와 카메라 센서들의 자세, 위치 정보를 뽑기.
    # 2. 라이다에서 카메라 위치까지 변환하는 translation 행렬을 정의
    # 3. 카메라의 자세로 맞춰주는 rotation 행렬을 정의.
    # 4. 위의 두 행렬을 가지고 최종 라이다-카메라 변환 행렬을 정의.
    

    # 로직 1. params에서 라이다와 카메라 센서들의 자세, 위치 정보를 뽑기.

    lidar_yaw, lidar_pitch, lidar_roll = params_lidar['yaw'], params_lidar['pitch'], params_lidar['roll']
    cam_yaw, cam_pitch, cam_roll = params_cam['yaw'], params_cam['pitch'], params_cam['roll']
    
    lidar_pos = [params_lidar['x'], params_lidar['y'], params_lidar['z']]
    cam_pos = [params_cam['x'], params_cam['y'], params_cam['z']]


    # 로직 2. 라이다에서 카메라 까지 변환하는 translation 행렬을 정의
    Tmtx = translationMtx(cam_pos[0] - lidar_pos[0], cam_pos[1] - lidar_pos[1], cam_pos[2] - lidar_pos[2])

    # 로직 3. 카메라의 자세로 맞춰주는 rotation 행렬을 정의
    Rmtx = rotationMtx(cam_yaw, cam_pitch, cam_roll)

    # 로직 4. 위의 두 행렬을 가지고 최종 라이다-카메라 변환 행렬을 정의
    RT = np.matmul(Rmtx, Tmtx)



    #테스트

    params_lidar = {
        "X": 0, # meter
        "Y": 0,
        "Z": 0.6,
        "YAW": 0, # deg
        "PITCH": 0,
        "ROLL": 0
    }


    params_cam = {
        "WIDTH": 640, # image width
        "HEIGHT": 480, # image height
        "FOV": 90, # Field of view
        "X": 0., # meter
        "Y": 0,
        "Z":  1.0,
        "YAW": 0, # deg
        "PITCH": 0.0,
        "ROLL": 0
    }

    # 이면

    R_T = np.array([
            [6.12323400e-17, -1.00000000e+00, 0.00000000e+00, 0.00000000e+00],
            [6.12323400e-17, 3.74939946e-33, -1.00000000e+00, 4.00000000e-01],
            [1.00000000e+00, 6.12323400e-17, 6.12323400e-17, -2.44929360e-17],
            [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
        ])

    return np.eye(4)


def project2img_mtx(params_cam):      # 3D 공간의 포인트를 2D 이미지 평면으로 변환하기 위한 투영 행렬을 생성

    # project2img_mtx 내 projection 행렬 계산 로직 순서
    # 1. params에서 카메라의 width, height, fov를 가져와서 focal length를 계산.
    # 2. 카메라의 파라메터로 이미지 프레임 센터를 계산.
    # 3. Projection 행렬을 계산 


    # 로직 1. params에서 카메라의 width, height, fov를 가져와서 focal length를 계산.
    fc_x = params_cam["WIDTH"] / (2 * np.tan(np.radians(params_cam["FOV"]) / 2))
    fc_y = params_cam["HEIGHT"] / (2 * np.tan(np.radians(params_cam["FOV"]) / 2))


    # 로직 2. 카메라의 파라메터로 이미지 프레임 센터를 계산.
    cx = params_cam["WIDTH"] / 2
    cy = params_cam["HEIGHT"] / 2


    # 로직 3. Projection 행렬을 계산.
    R_f = np.array([[fc_x, 0, cx],
                    [0, fc_y, cy],
                    [0, 0, 1]])


    """
    테스트

    params_cam = {
        "WIDTH": 320, # image width
        "HEIGHT": 240, # image height
        "FOV": 60, # Field of view
        "X": 0., # meter
        "Y": 0,
        "Z":  1.0,
        "YAW": 0, # deg
        "PITCH": 0.0,
        "ROLL": 0
    }

    이면

    R_f = 
    [[207.84609691   0.         160.        ]
    [  0.         207.84609691 120.        ]]
    """

    # return np.zeros((2,3))
    return R_f


def draw_pts_img(img, xi, yi):  # 이미지 점으로 그려내기 xi, yi는 좌표들의 리스트

    point_np = img     # img : 이미지를 나타내는 Numpy 배열, 원본 이미지를 건드리지 않고 새로운 이미지에 그리기 위해 복사함.

    #Left Lane
    for ctr in zip(xi, yi):    # x좌표와 y좌표를 만들어 냄. zip : 두 리스트에서 동일한 인덱스의 요소들을 짝지어 주는 함수.
        point_np = cv2.circle(point_np, ctr, 2, (255,0,0),-1)    # ctr을 원의 중심으로 하여 point_np이미지에 원을 그림. 세번째인자 2는 반지름.(픽셀단위)

    return point_np    # 모든 작업을 마친 후에 이미지 point_np를 반환


class LIDAR2CAMTransform:   # 라이다 포인트들을 라이다 좌표계에서 카메라 좌표계로 변환하고, 이를 이용하여 이미지 프레임 상의 픽셀 좌표로 투영하기.
    def __init__(self, params_cam, params_lidar):

        """

        LIDAR2CAMTransform 정의 및 기능 로직 순서
        1. Params를 입력으로 받아서 필요한 파라메터들과 RT 행렬, projection 행렬 등을 정의. 
        2. 클래스 내 self.RT로 라이다 포인트들을 카메라 좌표계로 변환.
        3. RT로 좌표 변환된 포인트들의 normalizing plane 상의 위치를 계산. 
        4. normalizing plane 상의 라이다 포인트들에 proj_mtx를 곱해 픽셀 좌표값 계산.
        5. 이미지 프레임 밖을 벗어나는 포인트들을 crop.
        """
        
        # 로직 1. Params에서 필요한 파라메터들과 RT 행렬, projection 행렬 등을 정의

				# 카메라 이미지의 너비와 높이, 그리고 카메라 이미지의 가로 및 세로 픽셀 수를 변수에 저장
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]

        self.n = float(params_cam["WIDTH"])
        self.m = float(params_cam["HEIGHT"])

        self.RT = transformMTX_lidar2cam(params_lidar, params_cam)     # 라이다 좌표계에서 카메라 좌표계로 변환한 행렬

        self.proj_mtx = project2img_mtx(params_cam)    # 이미지 투영 행렬

    def transform_lidar2cam(self, xyz_p):
        
        
        # 로직 2. 클래스 내 self.RT로 라이다 포인트들을 카메라 좌표계로 변환시킨다.
        # self.RT는 라이다 좌표계에서 카메라 좌표계로의 변환 행렬
				# 변환행렬과 xyz_p를 곱해서 카메라 좌표인 xyz_c를 추출하기

        xyz_c = np.dot(self.RT, xyz_p)

        return xyz_c

    def project_pts2img(self, xyz_c, crop=True):

				# 라이다 포인트를 투영한 결과를 저장할 빈 배열 xyi를 생성
        xyi=np.zeros((xyz_c.shape[0], 2))


        # 로직 3. RT로 좌표 변환된 포인트들의 normalizing plane 상의 위치를 계산.
				# NumPy 배열 슬라이싱과 요소별 연산사용
				# => 배열 xyz_c의 각 행에서 첫 번째 열([:, 0])의 값을 해당 행의 세 번째 열([:, 2])의 값으로 나누는 연산
        xn = xyz_c[:, 0] / xyz_c[:, 2]
        yn = xyz_c[:, 1] / xyz_c[:, 2]


        
        # 로직 4. normalizing plane 상의 라이다 포인트들에 proj_mtx를 곱해 픽셀 좌표값 계산.
        xyi = np.matmul(self.proj_mtx, np.concatenate([xn, yn, np.ones_like(xn)], axis=0))   # proj_mtx를 곱해 픽셀 좌표값 계산.

        if crop: xyi = self.crop_pts(xyi)
        else: pass
        
        return xyi

    def crop_pts(self, xyi):   # NumPy 배열에서 이미지 프레임을 벗어나는 포인트들을 잘라내는 역할

        xyi = xyi[np.logical_and(xyi[:, 0]>=0, xyi[:, 0]<self.width), :]
        xyi = xyi[np.logical_and(xyi[:, 1]>=0, xyi[:, 1]<self.height), :]

        return xyi

    


class SensorCalib(Node):

    def __init__(self):
        super().__init__(node_name='ex_calib')

        # 로직 1. 노드에 필요한 라이다와 카메라 topic의 subscriber 생성

        self.subs_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback, 10)    # LaserScan(라이다)topic 수신

        self.subs_img = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            10)     # CompressedImage(카메라)topic 수신

        # 로직 2. Params를 받아서 라이다 포인트를 카메라 이미지에 projection 하는
        # transformation class 정의하기

        self.l2c_trans = LIDAR2CAMTransform(params_cam, params_lidar)

        self.timer_period = 0.1

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.xyz, self.R, self.intens = None, None, None
        self.img = None



    def img_callback(self, msg):
    
        # 로직 3. 카메라 콜백함수에서 이미지를 클래스 내 변수로 저장.

        np_arr = np.frombuffer(msg.data, np.uint8)   # 이미지 데이터를 NumPy 배열로 변환 / frombuffer : 버퍼에서 NumPy 배열을 만들어내는 함수(이진 데이터를 NumPy 배열로 변환)
        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)   # NumPy 배열을 이미지로 변환


    def scan_callback(self, msg):
    

        # 로직 4. 라이다 2d scan data(거리와 각도)를 가지고 x,y 좌표계로 변환

		# 라이다 데이터에서 거리 정보 추출
        ranges = msg.ranges

		# 라이다 데이터에서 각도 정보 추출
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        self.R = np.column_stack((ranges, angles))

        # 거리와 각도를 이용하여 x, y 좌표 계산
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)  # 2D 스캔 데이터이므로 z는 0으로 설정 

        self.xyz = np.concatenate([
            x.reshape([-1, 1]),
            y.reshape([-1, 1]),
            z.reshape([-1, 1])
        ], axis=1)
        


    def timer_callback(self):

        if self.xyz is not None and self.img is not None :

            # 로직 5. 라이다 x,y 좌표 데이터 중 정면 부분만 crop
						# x 좌표가 0보다 크고 10보다 작은 범위에 있는 데이터만 선택하여 xyz_p에 저장
            xyz_p = self.xyz[(self.xyz[:, 0] > 0) & (self.xyz[:, 0] < 10), :]

            # 로직 6. transformation class 의 transform_lidar2cam 로 카메라 3d 좌표 변환
            xyz_c = self.l2c_trans.transform_lidar2cam(xyz_p)

            # 로직 7. transformation class 의 project_pts2img로 카메라 프레임으로 정사영
            xy_i = self.l2c_trans.project_pts2img(xyz_c)

            # 로직 8. draw_pts_img()로 카메라 이미지에 라이다 포인트를 draw 하고 show
            img_l2c = self.l2c_trans.draw_pts_img(self.img, xy_i[:, 0], xy_i[:, 1])

            cv2.imshow("Lidar2Cam", img_l2c)
            cv2.waitKey(1)

        else:

            print("waiting for msg")
            pass


def main(args=None):

    rclpy.init(args=args)

    calibrator = SensorCalib()

    rclpy.spin(calibrator)


if __name__ == '__main__':

    main()