'''
    此文件用于 W5500 的 SOCKET 之间的通信
'''

from myWiznet5k_spi import *
from myWiznet5k import *
from micropython import const
import sensor, image
from fpioa_manager import fm
from Maix import GPIO

class mySocket:
    def __init__(self, spi_port, wiznet5k, local_port=5000, img_pieces=1024, is_debug=False):
        self.__device = spi_port
        self.__wiznet5k = wiznet5k

        ''' 常量 '''
        self.local_port =local_port # 本地端口号
        # socket 端口选择
        self.SOCK_TCPS   = const(0)
        self.SOCK_HUMTEM = const(0)
        self.SOCK_PING	 = const(0)
        self.SOCK_TCPC   = const(1)
        self.SOCK_UDPS   = const(2)
        self.SOCK_WEIBO  = const(2)
        self.SOCK_DHCP   = const(3)
        self.SOCK_HTTPS  = const(4)
        self.SOCK_DNS    = const(5)
        self.SOCK_SMTP   = const(6)
        self.SOCK_NTP    = const(7)

        assert img_pieces >= 0 and img_pieces <= 2048, errorStr + 'img_pieces should in 0 - 2048 '
        self.img_pieces = img_pieces

        self.__beforeStatus = None
        self.__outlineFlag = True
        self.__is_debug = is_debug

    def socket(self, socket_num, protocol, port, flag):
        '''
        此Socket函数在特定模式下初始化通道，并设置端口并等待W5200完成。
        :param socket_num: socket number.
        :param protocol: The socket to choose.
        :param port: The port to bind.
        :param flag: Set some bit of MR,such as **< No Delayed Ack(TCP) flag.
        :return: 1 for success else 0.
        '''
        if (
            ((protocol&0x0f) == SN_MR_TCP) or
            ((protocol & 0x0f) == SN_MR_UDP) or
            ((protocol & 0x0f) == SN_MR_IPRAW) or
            ((protocol & 0x0f) == SN_MR_MACRAW) or
            ((protocol & 0x0f) == SN_MR_PPPOE)
        ):
            self.close(socket_num)  # 关闭
            self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_MR(socket_num), protocol | flag)
            if port != 0:
                self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_PORT0(socket_num), ((port&0xff00) >> 8))
                self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_PORT1(socket_num), (port&0x00ff))
            else:
                self.local_port += 1
                self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_PORT0(socket_num), ((self.local_port & 0xff00) >> 8))
                self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_PORT1(socket_num), (self.local_port & 0x00ff))
            self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_CR(socket_num), SN_CR_OPEN)
            while (self.__device.IINCHIP_READ(self.__wiznet5k.SN_CR(socket_num))):
                pass
            return 1
        else:
            return 0

    def close(self, socket_num):
        '''
        关闭 addr 的 socket
        :param socket_num:
        :return:
        '''
        self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_CR(socket_num), SN_CR_CLOSE)
        while (self.__device.IINCHIP_READ(self.__wiznet5k.SN_CR(socket_num))):
            pass
        self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_IR(socket_num), 0xFF)    # clear

    def listen(self, socket_num):
        '''
        此功能为处于被动（服务器）模式的通道建立了连接。
        此函数等待来自对等方的请求。
        :param socket_num:
        :return:
        '''
        if self.__device.IINCHIP_READ(self.__wiznet5k.SN_SR(socket_num)) == SOCK_INIT:
            self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_CR(socket_num), SN_CR_LISTEN)
            while self.__device.IINCHIP_READ(self.__wiznet5k.SN_CR(socket_num)):
                pass
            return 1
        else:
            return 0

    def connect(self, socket_num, addr, port):
        '''
        此功能为处于活动（客户端）模式的通道建立了连接。
        此功能等待，直到建立连接。
        :param socket_num:
        :param addr:
        :param port:
        :return:
        '''
        global ret
        ret = 0
        if (
                ((addr[0] == 0xFF) and (addr[1] == 0xFF) and (addr[2] == 0xFF) and (addr[3] == 0xFF)) or
                ((addr[0] == 0x00) and (addr[1] == 0x00) and (addr[2] == 0x00) and (addr[3] == 0x00)) or
                (port == 0x00)
        ):
            ret = 0
        else:
            ret = 1
            self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_DIPR0(socket_num), addr[0])
            self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_DIPR1(socket_num), addr[1])
            self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_DIPR2(socket_num), addr[2])
            self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_DIPR3(socket_num), addr[3])
            self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_DPORT0(socket_num), ((port & 0xff00) >> 8))
            self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_DPORT1(socket_num), (port & 0x00ff))
            self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_CR(socket_num), SN_CR_CONNECT)
            while self.__device.IINCHIP_READ(self.__wiznet5k.SN_CR(socket_num)):
                pass
            while (self.__device.IINCHIP_READ(self.__wiznet5k.SN_SR(socket_num)) != SOCK_SYNSENT):
                if self.__device.IINCHIP_READ(self.__wiznet5k.SN_SR(socket_num)) == SOCK_ESTABLISHED:
                    break
                if (self.__wiznet5k.getSn_IR(socket_num) & SN_IR_TIMEOUT):
                    self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_IR(socket_num), SN_IR_TIMEOUT)
                    ret = 0
                    break
        return ret

    def send(self, socket_num, buf, len):
        '''
        此函数用于在TCP模式下发送数据
        :param socket_num:
        :param buf:
        :return:
        '''
        global ret
        ret = 0

        if len > self.__wiznet5k.getIINCHIP_TxMAX(socket_num):
            ret = self.__wiznet5k.getIINCHIP_TxMAX(socket_num)
        else:
            ret = len

        while True:
            freesize = self.__wiznet5k.getSN_TX_FSR(socket_num)
            status = self.__device.IINCHIP_READ(self.__wiznet5k.SN_SR(socket_num))
            if (status != SOCK_ESTABLISHED) and (status != SOCK_CLOSE_WAIT):
                ret = 0
                break
            if freesize >= ret:
                break
        # copy data
        self.__wiznet5k.send_data_processing(socket_num, buf[:len])
        self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_CR(socket_num), SN_CR_SEND)
        # wait to process the command...
        while self.__device.IINCHIP_READ(self.__wiznet5k.SN_CR(socket_num)):
            pass
        while (self.__device.IINCHIP_READ(self.__wiznet5k.SN_IR(socket_num)) & SN_IR_SEND_OK) != SN_IR_SEND_OK:
            status = self.__device.IINCHIP_READ(self.__wiznet5k.SN_SR(socket_num))
            if ((status != SOCK_ESTABLISHED) and (status != SOCK_CLOSE_WAIT)):
                print(debugStr + "_error: SEND_OK Problem!!")
                self.close(socket_num)
                return 0
        self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_IR(socket_num), SN_IR_SEND_OK)
        # TODO [原函数 __DEF_IINCHIP_INT__ 未知]
        self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_IR(socket_num), SN_IR_SEND_OK)

        return ret

    def recv(self, socket_num, len):
        '''
        此函数是应用程序I/F函数，用于在TCP模式下接收数据。
        它继续等待应用程序想要接收的数据。
        :param socket_num:
        :param buf:
        :return:
        '''
        if len > 0:
            buf = self.__wiznet5k.recv_data_processing(socket_num, len)
            self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_CR(socket_num), SN_CR_RECV)
            while self.__device.IINCHIP_READ(self.__wiznet5k.SN_CR(socket_num)):
                pass
            return buf
        else:
            return 0

    def sendto(self, socket_num, buf, addr, port):
        global ret
        ret = 0
        if len(buf) > self.__wiznet5k.getIINCHIP_TxMAX(socket_num):
            ret = self.__wiznet5k.getIINCHIP_TxMAX(socket_num)
        else:   ret = len(buf)

        if (addr[0] == 0x00 and addr[1] == 0x00 and addr[2] == 0x00 and addr[3] == 0x00) or \
                (port == 0x00):
            ret = 0
        else:
            self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_DIPR0(socket_num), addr[0])
            self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_DIPR1(socket_num), addr[1])
            self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_DIPR2(socket_num), addr[2])
            self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_DIPR3(socket_num), addr[3])
            self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_DPORT0(socket_num), ((port & 0xff00) >> 8))
            self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_DPORT1(socket_num), (port & 0x00ff))
            # copy data
            self.__wiznet5k.send_data_processing(socket_num, buf)
            self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_CR(socket_num), SN_CR_SEND)
            while (self.__device.IINCHIP_READ(self.__wiznet5k.SN_CR(socket_num))):
                pass
            while ((self.__device.IINCHIP_READ(self.__wiznet5k.SN_IR(socket_num)) & SN_IR_SEND_OK) != SN_IR_SEND_OK):
                if self.__device.IINCHIP_READ(self.__wiznet5k.SN_IR(socket_num)) & SN_IR_TIMEOUT:
                    self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_IR(socket_num), (SN_IR_SEND_OK | SN_IR_TIMEOUT))
                    return 0
            self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_IR(socket_num), SN_IR_SEND_OK)

        return ret

    def recvfrom(self, socket_num, length, addr, port):
        '''
        此函数是一个应用程序I/F函数，用于接收其他应用程序中的数据
        TCP模式。此功能用于接收UDP、IP_RAW和MAC_RAW模式，并处理标头。
        :param socket_num:
        :param length:
        :param addr:
        :param port:
        :return:
        '''
        global buf
        global datalen
        datalen = 0
        buf = bytearray(133)

        if length > 0:
            ptr = self.__device.IINCHIP_READ(self.__wiznet5k.SN_RX_RD0(socket_num))
            ptr = ((ptr & 0x00ff) << 8) + self.__device.IINCHIP_READ(self.__wiznet5k.SN_RX_RD1(socket_num))
            addrbsb = (ptr << 8) + (socket_num << 5) + 0x18

            ret = self.__device.IINCHIP_READ(self.__wiznet5k.SN_MR(socket_num)) & 0x07

            if ret == SN_MR_UDP:
                head = self.__device.wiz_readBuff(addrbsb, 0x08)
                ptr += 8
                addr[0:4] = head[0:4]       # # IP address to receive.
                port[0] = head[4]
                port[1] = (port[0] << 8) + head[5]
                datalen = (head[6] << 8) + head[7]

                addrbsb = (ptr << 8) + (socket_num << 5) + 0x18
                buf = self.__device.wiz_readBuff(addrbsb, datalen)
                ptr += datalen

                self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_RX_RD0(socket_num), ((ptr & 0xff00) >> 8))
                self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_RX_RD1(socket_num), (ptr & 0x00ff))
            elif ret == SN_MR_IPRAW:
                head = self.__device.wiz_readBuff(addrbsb, 0x06)
                ptr += 6
                addr[0:4] = head[0:4]
                datalen = (head[4] << 8) + head[5]

                addrbsb = (ptr << 8) + (socket_num << 5) + 0x18
                buf = self.__device.wiz_readBuff(addrbsb, datalen)
                ptr += datalen

                self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_RX_RD0(socket_num), ((ptr & 0xff00) >> 8))
                self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_RX_RD1(socket_num), (ptr & 0x00ff))
                print(debugStr + "socket Status: SN_MR_IPRAW")
            elif ret == SN_MR_MACRAW:
                head = self.__device.wiz_readBuff(addrbsb, 0x02)
                ptr += 2
                datalen = (head[0] << 8) + head[1] - 2
                if datalen > 1514:
                    print(debugStr + "data_len over 1514")
                    while True:
                        pass

                addrbsb = (ptr << 8) + (socket_num << 5) + 0x18
                buf = self.__device.wiz_readBuff(addrbsb, datalen)
                ptr += datalen

                self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_RX_RD0(socket_num), ((ptr & 0xff00) >> 8))
                self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_RX_RD1(socket_num), (ptr & 0x00ff))
            self.__device.IINCHIP_WRITE(self.__wiznet5k.SN_CR(socket_num), SN_CR_RECV)
            while (self.__device.IINCHIP_READ(self.__wiznet5k.SN_CR(socket_num))):
                pass

        print(debugStr + "Recv Datalen: %d" % datalen)
        return [datalen, buf]

    def do_tcp_server(self):
        '''
        tcp 服务端函数
        :return:
        '''
        SOCKET_STATUS = self.__wiznet5k.getSn_SR(self.SOCK_TCPS)   # 获取 socket 状态
        if SOCKET_STATUS == SOCK_CLOSED:   # 关闭状态
            self.socket(self.SOCK_TCPS, SN_MR_TCP, self.local_port, SN_MR_ND) # 打开 SOCKET
        elif SOCKET_STATUS == SOCK_INIT:   # SOCKET 已初始化
            self.listen(self.SOCK_TCPS) # 建立监听
        elif SOCKET_STATUS == SOCK_ESTABLISHED:    # SOCKET 处于链接建立状态
            pass
            # if self.__wiznet5k.getSn_IR(self.SOCK_TCPS) & SN_IR_CON:
            #     self.__wiznet5k.setSn_IR(self.SOCK_TCPS, SN_IR_CON)   # 清除接收中断标志位
            # length = self.__wiznet5k.getSN_RX_RSR(self.SOCK_TCPS)  # 定义 length 为已接收数据的长度
            # if length > 0:
            #     recvBuff = self.recv(self.SOCK_TCPS, length) # 接收来自 Client 的数据
            #     recvBuff.append(0x00) # 添加字符串结束符
            #     print("SOCKET Recv-> %s" % recvBuff[:length])
            #     self.send(self.SOCK_TCPS, recvBuff, length) # 向 Client 发送数据
        elif SOCKET_STATUS == SOCK_CLOSE_WAIT: # SOCKET 处于等待关闭状态
            self.close(self.SOCK_TCPS)

        return SOCKET_STATUS

    def do_tcp_client(self, remote_ip, remote_port):
        '''
        tcp 客户端函数
        :return:
        '''
        SOCKET_STATUS = self.__wiznet5k.getSn_SR(self.SOCK_TCPC)  # 获取 socket 状态

        if SOCKET_STATUS == SOCK_CLOSED:  # socket 处于关闭状态
            if SOCKET_STATUS != self.__beforeStatus and self.__outlineFlag:
                print(debugStr + "_status: Now Status: SOCK_CLOSED")
            self.local_port += 1
            self.socket(self.SOCK_TCPC, SN_MR_TCP, self.local_port, SN_MR_ND)
        elif SOCKET_STATUS == SOCK_INIT:  # socket 处于初始化状态
            if SOCKET_STATUS != self.__beforeStatus and self.__outlineFlag:
                print(debugStr + "_status: Now Status: SOCK_INIT")
            self.connect(self.SOCK_TCPC, remote_ip, remote_port)  # socket 连接服务器
        elif SOCKET_STATUS == SOCK_ESTABLISHED:  # socket 处于连接状态
            self.__outlineFlag = True
            if SOCKET_STATUS != self.__beforeStatus and self.__outlineFlag:
                print(debugStr + "_status: Now Status: SOCK_ESTABLISHED ...... ❤")
                print(debugStr + "_status: Console log: {}\n".format(self.__is_debug))
            ''' 发送图像 '''
            # self.image_send()
            ''' 发送图像 '''
            # if (self.getSn_IR(self.SOCK_TCPC) & self.SN_IR_CON):
            #     self.setSn_IR(self.SOCK_TCPC, self.SN_IR_CON)   #清除接收中断标志位
            # length = self.getSN_RX_RSR(self.SOCK_TCPC)  # 定义 length 为已接收数据的长度
            # if length > 0:
            #     recvBuff = self.recv(self.SOCK_TCPC, length)    # 接收来自 Server 的数据
            #     recvBuff.append(0x00)   # 添加字符串结束符
            #     print("\nrecvBuff-> %s" % (recvBuff.decode()))
            #     self.send(self.SOCK_TCPC, recvBuff, length) # 向 Server 发送数据
        elif SOCKET_STATUS == SOCK_CLOSE_WAIT:  # socket 处于等待关闭状态
            if SOCKET_STATUS != self.__beforeStatus and self.__outlineFlag:
                print(debugStr + "_status: Now Status: SOCK_CLOSE_WAIT")
            self.close(self.SOCK_TCPC)
        elif SOCKET_STATUS == SOCK_SYNSENT:
            if SOCKET_STATUS != self.__beforeStatus and self.__outlineFlag:
                self.__outlineFlag = False
                print(debugStr + "_status: Now Status: Server outline ...... ✖")
        self.__beforeStatus = SOCKET_STATUS  # 更新上次输出信息

        return SOCKET_STATUS

    def image_send(self, quality=80):
        '''
        通过 tcp 发送图像
        '''
        global deltime, fps
        deltime = None
        fps = None

        start_time = time.ticks()   # 获取时间
        img = sensor.snapshot()         # 拍摄一张图片
        img = img.compress(quality=quality).to_bytes()     # 压缩图像
        total_size = len(img) # 计算图像总字节
        int_pieces = total_size // self.img_pieces
        end_pieces = total_size % self.img_pieces
        head_buf = str(start_time).encode() + ".".encode() + str(total_size).encode()
        # 发送数据头
        self.send(self.SOCK_TCPC, head_buf, len(head_buf))
        while True:
            ''' 等待服务端处理完成 '''
            if (self.__wiznet5k.getSn_IR(self.SOCK_TCPC) & SN_IR_CON):
                self.__wiznet5k.setSn_IR(self.SOCK_TCPC, SN_IR_CON)  # 清除接收中断标志位
            length = self.__wiznet5k.getSN_RX_RSR(self.SOCK_TCPC)  # 定义 length 为已接收数据的长度
            if length > 0:
                recvBuff = self.recv(self.SOCK_TCPC, length)  # 接收来自 Server 的数据
                if recvBuff.decode() == 'ok':   # 收到服务端的完成信号
                    ''' 分包发送图像 '''
                    # 整包
                    for i in range(int_pieces):
                        self.send(self.SOCK_TCPC, img[i*self.img_pieces:(i+1)*self.img_pieces], self.img_pieces)
                    # 剩下的包
                    self.send(self.SOCK_TCPC, img[int_pieces*self.img_pieces:], end_pieces)
                    break
        while True:
            ''' 等待服务端处理完成 '''
            if (self.__wiznet5k.getSn_IR(self.SOCK_TCPC) & SN_IR_CON):
                self.__wiznet5k.setSn_IR(self.SOCK_TCPC, SN_IR_CON)  # 清除接收中断标志位
            length = self.__wiznet5k.getSN_RX_RSR(self.SOCK_TCPC)  # 定义 length 为已接收数据的长度
            if length > 0:
                recvBuff = self.recv(self.SOCK_TCPC, length)  # 接收来自 Server 的数据
                if recvBuff.decode() == 'ok':  # 收到服务端的完成信号
                    deltime = time.ticks() - start_time
                    fps =  (1 / deltime)*1000
                    print(debugStr + "处理时间: {} ms FPS: {:.2f}".format(deltime, fps))
                    break
        # DEBUG 信息输出
        if self.__is_debug:
            print(debugStr + "{}.{}".format(start_time, total_size))

        return (img, deltime, fps)

    def do_udp(self, remote_ip, remote_port):
        '''
        udp 接收服务函数
        '''
        SOCKET_STATUS = self.__wiznet5k.getSn_SR(self.SOCK_UDPS)    # 获取 SOKCET 状态

        if SOCKET_STATUS == SOCK_CLOSED:    # SOCKET 处于关闭状态
            self.socket(self.SOCK_UDPS, SN_MR_UDP, self.local_port, 0)   # 初始化 SOCKET
        elif SOCKET_STATUS == SOCK_UDP:     # SOCKET 初始化完成
            time.sleep_ms(10)
            if self.__wiznet5k.getSn_IR(self.SOCK_UDPS) & SN_IR_RECV:
                # 清除接收中断
                self.__wiznet5k.setSn_IR(self.SOCK_UDPS, SN_IR_RECV)
            length = self.__wiznet5k.getSN_RX_RSR(self.SOCK_UDPS)   # 接收到数据
            if length > 0:
                recvBuf = self.recvfrom(self.SOCK_UDPS, length, remote_ip, remote_port) # 接收发送过来的数据
                recvBuf[length - 8] = 0x00  # 添加字符串结束符
                print(debugStr + "SOCKET Recv-> %s" % recvBuf[:length])
                self.sendto(self.SOCK_UDPS, recvBuf[:length - 8], remote_ip, remote_port)

        return SOCKET_STATUS

if __name__ == '__main__':
    # 配置信息
    config = (
        bytearray([0x00, 0x08, 0xdc, 0x11, 0x11, 0x11]),  # mac 地址
        bytearray([192, 168, 5, 5]),  # ip地址
        bytearray([255, 255, 255, 0]),  # 子网掩码
        bytearray([192, 168, 5, 1])  # 网关
    )

    remote_ip = bytearray([192, 168, 5, 20])  # 服务端ip
    remote_port = 5500  # 服务端端口

    # reset 脚
    fm.register(31, fm.fpioa.GPIOHS11, force=True)  # reset
    reset = GPIO(GPIO.GPIOHS11, GPIO.OUT)  # reset 脚
    reset.value(1)

    # 初始化 mySpi 类
    spi = mySpi(1)

    # 延时，等待稳定
    for i in range(1, 11):
        print(debugStr + 'wait for start ....... have {} second!'.format(10 - i))
        time.sleep_ms(1000)

    time.sleep_ms(1000)
    # w5500 初始化
    w5500 = myWiznet5k(spi, config=config, reset_pin=reset, is_debug=True)

    # 初始化 socket
    socket = mySocket(spi, w5500, img_pieces=2040)

    print(debugStr + ' W5500 为客户端')
    print(debugStr + "连接到服务端 %d.%d.%d.%d:%d" % (remote_ip[0], remote_ip[1], remote_ip[2],
                                          remote_ip[3], remote_port))

    # 摄像头初始化
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=2000)

    clock = time.clock()

    while True:
        SOCK_STATUS = socket.do_tcp_client(remote_ip, remote_port)
        if SOCK_STATUS == SOCK_ESTABLISHED:
            socket.image_send()
