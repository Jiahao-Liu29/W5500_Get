import socket
import cv2
import numpy as np

class readImg():
    def __init__(self, target_ip, target_port):
        self.local_ip = target_ip      # 本地 ip
        self.local_port = target_port  # 本地 端口
        self.target_socket = None   # 客户端 socket 对象
        self.target_address = None  # 客户端地址
        self.__server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


    def __creat_Server(self):
        '''
        建立 TCP 连接
        :return:
        '''
        self.__server.bind((self.local_ip, self.local_port))
        print('启动服务端 {}:{} 成功！'.format(self.local_ip, self.local_port))
        self.__server.listen(100) # 最大监听5个设备

    def get_Client(self):
        '''
        获取客户端
        :return:
        '''
        self.__creat_Server()
        self.target_socket, self.target_address = self.__server.accept()
        print("{} 已连接!".format(self.target_address))

    def __close_Client(self):
        '''
        关闭 tcp 连接
        :return:
        '''
        self.target_socket.close()


    def sendMsg(self, data, type='utf-8'):
        '''
        发送信息
        :param data:
        :param type:
        :return:
        '''
        if self.target_socket is not None:
            self.target_socket.send(data.encode(encoding=type))

    def recvMsg(self, size):
        '''
        获取 size 字节数据
        :param size:
        :return:
        '''
        if self.target_socket is not None:
            return self.target_socket.recv(size)


if __name__ == '__main__':
    readimg = readImg(target_ip="0.0.0.0", target_port=5500)

    while True:
        readimg.get_Client()
        while True:
            # 初始化缓冲区，用于接收头信息
            resonData = readimg.recvMsg(1024)
            # print(int.from_bytes(resonData, byteorder='big'))
            if resonData:
                # 通知客户端
                readimg.sendMsg('ok')
                # 处理标志数据
                dataList = resonData.decode().split('.')
                # 拍照时间
                snapShotTime = int(dataList[0])
                # 图像总字节数
                total_size = int(dataList[1])
                # 接收到的数据计数
                cnt = 0
                # 存放接收到的数据
                imageBytes = b''

                while cnt < total_size:
                    data = readimg.recvMsg(2048)
                    imageBytes += data
                    cnt += len(data)
                    print(" [Python]@Server -> receive: {} / {}".format(cnt, total_size))

                readimg.sendMsg('ok')

                # 解析接收到的字节流数据，并显示图像
                img = np.asarray(bytearray(imageBytes), dtype="uint8")
                img = cv2.imdecode(img, cv2.IMREAD_COLOR)
                cv2.imshow("img", img)
                cv2.waitKey(1)

