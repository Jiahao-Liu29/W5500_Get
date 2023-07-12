'''
    此文件用于 W5500 的 SOCKET 之间的通信
'''

from myWiznet5k_spi import *
from myWiznet5k import *
from micropython import const

class mySocket:
    def __init__(self, spi_port, wiznet5k, local_port=5000):
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

        if len > self.__device.getIINCHIP_TxMAX(socket_num):
            ret = self.__device.getIINCHIP_TxMAX(socket_num)
        else:
            ret = len

        while True:
            freesize = self.__device.getSN_TX_FSR(socket_num)
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
                print("SEND_OK Problem!!")
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
                print("socket Status: SN_MR_IPRAW")
            elif ret == SN_MR_MACRAW:
                head = self.__device.wiz_readBuff(addrbsb, 0x02)
                ptr += 2
                datalen = (head[0] << 8) + head[1] - 2
                if datalen > 1514:
                    print("data_len over 1514")
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

        print("Recv Datalen: %d" % datalen)
        return [datalen, buf]