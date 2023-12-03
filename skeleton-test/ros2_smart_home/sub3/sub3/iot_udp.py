import rclpy
from rclpy.node import Node
import time
import os
import socket
import threading
import struct
import binascii
import signal

# iot_udp 노드는 udp 통신을 이용해 iot로 부터 uid를 얻어 접속, 제어를 하는 노드입니다.
# sub1,2 에서는 ros 메시지를 이용해 쉽게 제어했지만, advanced iot 제어에서는 정의된 통신프로토콜을 보고 iot와 직접 데이터를 주고 받는 형식으로 제어하게 됩니다.
# 통신 프로토콜은 명세서를 참조해주세요.


# 노드 로직 순서
# 1. 통신 소켓 생성
# 2. 멀티스레드를 이용한 데이터 수신
# 3. 수신 데이터 파싱
# 4. 데이터 송신 함수
# 5. 사용자 메뉴 생성
# 6. iot scan
# 7. iot connect
# 8. iot control

# 통신프로토콜에 필요한 데이터입니다. 명세서에 제어, 상태 프로토콜을 참조하세요.
params_status = {
    (0xA, 0x25): "IDLE",
    (0xB, 0x31): "CONNECTION",
    (0xC, 0x51): "CONNECTION_LOST",
    (0xB, 0x37): "ON",
    (0xA, 0x70): "OFF",
    (0xC, 0x44): "ERROR",
}

params_status_reversed = dict(map(reversed, params_status.items()))

params_control_cmd = {
    "TRY_TO_CONNECT": (0xB, 0x31),
    "SWITCH_ON": (0xB, 0x37),
    "SWITCH_OFF": (0xA, 0x70),
    "RESET": (0xB, 0x25),
    "DISCONNECT": (0x00, 0x25),
}


class iot_udp(Node):
    def __init__(self):
        super().__init__("iot_udp")

        self.ip = "127.0.0.1"
        self.port = 7502
        self.send_port = 7401

        # 로직 1. 통신 소켓 생성
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_address = (self.ip, self.port)
        self.sock.bind(recv_address)
        self.data_size = 65535
        self.parsed_data = []

        # 로직 2. 멀티스레드를 이용한 데이터 수신
        thread = threading.Thread(target=self.recv_udp_data)
        thread.daemon = True
        thread.start()

        self.is_recv_data = False

        os.system("cls")

        global Sentry
        while True:
            Sentry = True
            pass
            """
            로직 5. 사용자 메뉴 생성
            print('Select Menu [0: scan, 1: connect, 2:control, 3:disconnect, 4:all_procedures ] ')
            menu=??

            if menu == ?? :
                채워 넣기
            

            """
            print(
                "Select Menu [0: scan, 1: connect, 2:control, 3:disconnect, 4:all_procedures ] "
            )
            menu = input(">>")

            if menu == "0":
                self.scan()
            if menu == "1":
                self.connect()
            if menu == "2":
                self.control()
            if menu == "3":
                self.disconnect()
            if menu == "4":
                self.all_procedures()

    def data_parsing(self, raw_data):
        """
        로직 3. 수신 데이터 파싱

        header=?
        data_length=?
        aux_data=?


        if header == ?? and data_length[0] == ??:
            uid_pack=??
            uid=self.packet_to_uid(uid_pack)

            network_status=??
            device_status=??

            self.is_recv_data=True
            self.recv_data=[uid,network_status,device_status]
        """
        header = raw_data[0:19].decode()
        data_length = struct.unpack("i", raw_data[19:23])
        # aux_data = struct.unpack("i", raw_data[23:35])

        # print("header : ", header)
        # print("data_length : ", data_length)
        if header == "#Appliances-Status$" and data_length[0] == 20:
            uid_pack = raw_data[35:51]
            uid = self.packet_to_uid(uid_pack)

            network_status = struct.unpack("i", raw_data[51:53])
            device_status = struct.unpack("i", raw_data[53:55])

            self.is_recv_data = True
            self.recv_data = [uid, network_status, device_status]
            print(self.recv_data)

    def send_data(self, uid, cmd):
        pass
        """
        로직 4. 데이터 송신 함수 생성


        header=?
        data_length=?
        aux_data=?
        self.upper=?
        self.tail=?

        uid_pack=self.uid_to_packet(uid)
        cmd_pack=bytes([cmd[0],cmd[1]])

        send_data=self.upper+uid_pack+cmd_pack+self.tail
        self.sock.sendto(send_data,(self.ip,self.send_port))
        """
        header = "$Ctrl-command$".encode()
        data_length = struct.pack("i", 18)
        aux_data = struct.pack("iii", 0, 0, 0)
        self.upper = header + data_length + aux_data
        self.tail = "\r\n".encode()

        uid_pack = self.uid_to_packet(uid)
        cmd_pack = bytes([cmd[0], cmd[1]])

        send_data = self.upper + uid_pack + cmd_pack + self.tail
        self.sock.sendto(send_data, (self.ip, self.send_port))

    def recv_udp_data(self):
        while Sentry:
            raw_data, sender = self.sock.recvfrom(self.data_size)
            self.data_parsing(raw_data)

    def uid_to_packet(self, uid):
        uid_pack = binascii.unhexlify(uid)
        return uid_pack

    def packet_to_uid(self, packet):
        uid = ""
        for data in packet:
            if len(hex(data)[2:4]) == 1:
                uid += "0"

            uid += hex(data)[2:4]

        return uid

    def scan(self):
        print("SCANNING NOW.....")
        print("BACK TO MENU : Ctrl+ C")
        """
        로직 6. iot scan

        주변에 들어오는 iot 데이터(uid,network status, device status)를 출력하세요.

        """
        self.recv_udp_data()

    def connect(self):
        pass
        """
        로직 7. iot connect

        iot 네트워크 상태를 확인하고, CONNECTION_LOST 상태이면, RESET 명령을 보내고,
        나머지 상태일 때는 TRY_TO_CONNECT 명령을 보내서 iot에 접속하세요.

        """
        raw_data, sender = self.sock.recvfrom(self.data_size)
        self.data_parsing(raw_data)
        if self.recv_data[1] == params_status_reversed.get("CONNECTION_LOST"):
            self.send_data(self.recv_data[0], params_control_cmd.get("RESET"))
        else:
            self.send_data(self.recv_data[0], params_control_cmd.get("TRY_TO_CONNECT"))

    def control(self):
        pass
        """
        로직 8. iot control
        
        iot 디바이스 상태를 확인하고, ON 상태이면 OFF 명령을 보내고, OFF 상태면 ON 명령을 보내서,
        현재 상태를 토글시켜주세요.
        """
        raw_data, sender = self.sock.recvfrom(self.data_size)
        self.data_parsing(raw_data)
        if self.recv_data[2] == params_status_reversed.get("ON"):
            self.send_data(self.recv_data[0], params_control_cmd.get("SWITCH_OFF"))
        elif self.recv_data[2] == params_status_reversed.get("OFF"):
            self.send_data(self.recv_data[0], params_control_cmd.get("SWITCH_ON"))

    def disconnect(self):
        if self.is_recv_data == True:
            self.send_data(self.recv_data[0], params_control_cmd.get("DISCONNECT"))

    def all_procedures(self):
        self.connect()
        time.sleep(0.5)
        self.control()
        time.sleep(0.5)
        self.disconnect()

    def __del__(self):
        self.sock.close()
        print("del")


Sentry = True


def SignalHandler_SIGINT(SignalNumber, Frame):
    # print("SignalHandler of signal.SIGINT")
    # print(f"Signal Number -> {SignalNumber} Frame -> {Frame}")
    global Sentry
    Sentry = False


signal.signal(signal.SIGINT, SignalHandler_SIGINT)


def main(args=None):
    rclpy.init(args=args)
    iot = iot_udp()
    rclpy.spin(iot)
    iot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
