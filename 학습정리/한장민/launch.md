
ssafybridge_launch.py 실행 시 4개의 노드가 자동으로 실행됨

udp_to_pub

1. 각각의 data_type(envir_status, app_status, turtlebot_status, imu, custom_object_info)에 맞춰 퍼블리셔를 생성
2. ssafy_udp_parser 노드의 erp_udp_parser 메소드를 이용하여 해당하는 유저 ip와 포트에 맞게 udp데이터를 파싱

sub_to_udp

udp_to_cam

```python
params_cam_0 = {
    "SOCKET_TYPE": 'JPG',
    "WIDTH": 320, # image width
    "HEIGHT": 240, # image height
    "FOV": 60, # Field of view
    "localIP": "127.0.0.1",
    "localPort": 1232,
    "Block_SIZE": int(65000),
    "UnitBlock_HEIGHT": int(30),
    "X": 1.7, # meter
    "Y": 0,
    "Z": 1.2,
    "YAW": 0, # deg
    "PITCH": -5,
    "ROLL": 0
}
```

1. 주어진 params_cam_0 데이터를 utils.py의 UDP_CAM_Parser 메소드에 전달하여 카메라로 촬영하는 데이터를 수신
2. CompressedImage를 발신하는 퍼블리셔 생성
3. 받아오는 데이터의 바이트가 0이 아니라면 jpeg로 포맷하여 퍼블리싱

udp_to_laser

ssafy_bridge 패키지에는 존재하나 실행되지 않는 노드

cam_viewer

- sub1의 perception 노드와 비슷한 역할을 진행
- 시뮬레이터의 터틀봇이 촬영하는 이미지 데이터를 실시간으로 전송하여 외부의 창을 통해 송출하는 역할
- 존재하나 launch.py에 포함되지 않아 실행되지는 않는다

ssafy_udp_parser

- ssafy_bridge 패키지의 노드들이 기능을 위해 필요한 클래스를 포함
    - erp_udp_parser
    
    ```python
    def __init__(self,publisher,ip,port,data_type):
    
    				# udp_to_pub 에서 실행할 때 받아온 매개변수 세팅
            self.ip=ip # UDP 데이터를 수신할 IP 주소
            self.port=port # UDP 데이터를 수신할 포트 번호
            self.publisher=publisher # 데이터를 발행하는 데 사용되는 객체
            self.data_type=data_type # 수신된 데이터의 유형을 지정하는 문자열
    				
            self.is_sender_port=False # 송신 정보 포트
    			
    				# UDP 통신에 필요한 socket 객체 생성
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    				# 수신할 ip주소와 port 번호를 튜플로 생성합니다
            recv_address = (self.ip,self.port)
    				# 튜플로 생성한 ip주소와 port번호를 소켓에 바인딩합니다
            self.sock.bind(recv_address)
    				# 데이터의 최대 크기 설정, 65535바이트
            self.data_size=65535 
    				# 파싱한 데이터를 받기 위한 빈 배열을 초기화
            self.parsed_data=[]
    
    				# UDP 데이터를 비동기적으로 처리하기 위한 스레드 생성
    				# recv_udp_data 메소드가 이 스레드의 작업대상
            thread = threading.Thread(target=self.recv_udp_data)
    				# 스레드를 데몬 쓰레드로 설정
    				# 데몬 쓰레드 : 백그라운드에서 작동
            thread.daemon = True 
    				# 쓰레드를 시작하여 UPD 데이터를 비동기적으로 수신
            thread.start()
    ```
    
    ```python
    def recv_udp_data(self):
        while True : # 무한반복
    				# 수신한 UDP 데이터를
    				# raw_data : 수신한 데이터
    				# sender : 송신자의 정보(주소 및 포트)
            raw_data, sender = self.sock.recvfrom(self.data_size)
            
    				# raw_data에 저장된 UDP 데이터를 처리하기 위해 data_parsing 메서드를 호출
    				# data_parsing : 주어진 데이터를 파싱하고 처리
    				self.data_parsing(raw_data)
            
    				# is_sender_port 의 값이 False일 때만 실행
    				# 초기에 False로 지정하고 아래의 코드 블럭이 실행되면 True로 변하기 때문에
    				# 송신 포트 정보를 처음 수신했을때만 실행되는 코드이다
            if self.is_sender_port == False :
                self.is_sender_port= True # True로 설정하여 송신 포트 정보를 한번만 저장하도록 한다
                self.sender_port=sender[1] # 송신자 정보 sender에서 포트 번호(sender[1])를 추출하여 저장
    
    				# 처음 수신한 데이터의 송신 포트 정보를 저장
    				# 나중에 데이터를 보낼 때 적절한 포트로 데이터를 전송할 수 있다
    ```
    
    ```python
    파싱되기 전의 바이너리 형태의 raw_data
    
    b'#Turtlebot$ \x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x83\x9a>\xb6\x00\x00\x00\x00\x00\x00\x00\x00\x00ta\x16\xc1dt\xf6\xc0\x80\\\xc4:\x08\xa7\xb4B\x00\x00\x00\r\n'
    b'#Enviroment$\x06\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x0b\t\x01\r\x00\r\n'
    b'#hand_control_pub$\xf0\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\r\n'
    b'#Appliances$\x11\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x02\x02\x02\x02\x02\x02\x02\x02\x02\x02\x02\x02\x02\x02\x02\x02\x02\r\n'
    
    data_parsing 메소드
    
    if self.data_type=='turtlebot_status': # data_type에 따라 실행할 코드블럭 분리
        # print(raw_data)
        header=raw_data[0:11].decode() # raw_data의 11번째 항목까지 decode하여 header로 설정
        data_length=struct.unpack('i',raw_data[11:15])
        if header == '#Turtlebot$' and data_length[0] ==32:
            msg=TurtlebotStatus()
    				# struct.unpack 하단 추가 설명
    				# 각 데이터에 맞는 값을 가져와 msg에 넣어주는 과정
            ego_status_data=struct.unpack('2f',raw_data[15+12:15+8+12])
            battery_charge_status=struct.unpack('B',raw_data[15+8+12:15+8+12+1])[0]
            battery_percentage=struct.unpack('f',raw_data[15+8+12+1:15+8+12+1+4])[0]
            msg.twist.linear.x=ego_status_data[0]
            msg.twist.angular.z=ego_status_data[1]
            msg.power_supply_status=battery_charge_status
            msg.battery_percentage=battery_percentage
    
            x,y,z,heading=struct.unpack('4f',raw_data[15+8+12+1+4:15+8+12+1+4+16])
            msg.twist.angular.x=x
            msg.twist.angular.y=y
            msg.twist.linear.z=heading
    
            hand_status=struct.unpack('???',raw_data[56:59])
            msg.can_use_hand=hand_status[0]
            msg.can_put=hand_status[1]
            msg.can_lift=hand_status[2]
    
    				# msg 퍼블리시
            self.publisher.publish(msg)
    
    ```
    
    ```python
    struct.unpack(format, buffer)
    포맷 문자열 format에 따라 버퍼 buffer(아마도 pack(format, ...)으로 패킹 된)에서 언 패킹 합니다. 정확히 하나의 항목을 포함하더라도 결과는 튜플입니다. 바이트 단위의 버퍼 크기는 (calcsize()에 의해 반영되는) 포맷이 요구하는 크기와 일치해야 합니다.
    
    포맷 문자열 format의 종류
    ```
    
    ![Untitled](https://prod-files-secure.s3.us-west-2.amazonaws.com/cd20751b-e4bb-48a3-9815-f42c4e455b3e/93b60615-2bf0-4195-8cd2-e44a9438fa74/Untitled.png)
    
    - erp_udp_sender
    - handControlSender
    - app_control_sender

utils

- ssafy_bridge 패키지의 노드들이 기능을 위해 필요한 클래스를 포함
    - UDP_CAM_Parser
    - UDP_LIDAR_Parser
