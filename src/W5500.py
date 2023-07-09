from machine import SPI
from fpioa_manager import fm
from Maix import GPIO
import time
import struct

''' SPI 类 '''
class spiChat():
    def __init__(self, baudrate=10000000):
        fm.register(27, fm.fpioa.GPIOHS10, force=True)  # cs
        self.cs = GPIO(GPIO.GPIOHS10, GPIO.OUT)    # cs 脚
        self.spi = SPI(SPI.SPI1, mode=SPI.MODE_MASTER, baudrate=baudrate, polarity=0, phase=0, bits=8, firstbit=SPI.MSB, sck=28, mosi=29, miso=30)


''' W5500类 '''
class w5500(spiChat):
    def __init__(self):
        super().__init__()
        fm.register(31, fm.fpioa.GPIOHS11, force=True)  # reset
        self.reset = GPIO(GPIO.GPIOHS11, GPIO.OUT)  # reset 脚

        self.SHAR0 = 0x000900
        self.SUBR0 = 0x000500
        self.SIPR0 = 0x000F00
        self.GAR0  = 0x000100

        self.SOCKET_CLOSED = 0x00    # closed
        self.SOCK_IPRAW = 0x32      # ip raw mode socket

        self.SN_IR_SEND_OK = 0x10   # complete sending
        self.SN_IR_TIMEOUT = 0x08   # assert timeout

        self.SN_MR_TCP = 0x01       # tcp
        self.SN_MR_UDP = 0x02       # udp
        self.SN_MR_IPRAW = 0x03     # IP LAYER RAW SOCK
        self.SN_MR_MACRAW = 0x04    # MAC LAYER RAW SOCK
        self.SN_MR_PPPOE = 0x05     # PPPoE

        self.SN_CR_OPEN = 0x01      # initialize or open socket
        self.SN_CR_CLOSE = 0x10     # close socket
        self.SN_CR_SEND = 0x20      # update txbuf pointer, send data
        self.SN_CR_RECV = 0x40      # update rxbuf pointer, recv data

        self.IPPROTO_ICMP = 1

    @staticmethod
    def SN_CR(ch):  # 通道Sn_CR寄存器
        return (0x000108 + (ch << 5))
    @staticmethod
    def SN_IR(ch):
        return (0x000208 + (ch << 5))
    @staticmethod
    def SN_SR(ch):  # 通道状态寄存器
        return (0x000308 + (ch << 5))
    @staticmethod
    def SN_PROTO(ch):
        return (0x001408 + (ch << 5))
    @staticmethod
    def SN_MR(ch): # 套接字模式寄存器
        return (0x000008 + (ch << 5))

    ''' 源端口寄存器 '''
    @staticmethod
    def SN_PORT0(ch):
        return (0x000408 + (ch << 5))
    @staticmethod
    def SN_PORT1(ch):
        return (0x000508 + (ch << 5))

    ''' 对等IP寄存器地址 '''
    @staticmethod
    def SN_DIPR0(ch):
        return (0x000C08 + (ch << 5))
    @staticmethod
    def SN_DIPR1(ch):
        return (0x000D08 + (ch << 5))
    @staticmethod
    def SN_DIPR2(ch):
        return (0x000E08 + (ch << 5))
    @staticmethod
    def SN_DIPR3(ch):
        return (0x000F08 + (ch << 5))

    ''' 对等端口寄存器地址 '''
    @staticmethod
    def SN_DPORT0(ch):
        return (0x001008 + (ch << 5))
    @staticmethod
    def SN_DPORT1(ch):
        return (0x001108 + (ch << 5))

    ''' 传输内存读取指针寄存器地址 '''
    @staticmethod
    def SN_TX_WR0(ch):
        return (0x002408 + (ch << 5))
    @staticmethod
    def SN_TX_WR1(ch):
        return (0x002508 + (ch << 5))

    ''' 接收数据大小寄存器 '''
    @staticmethod
    def SN_RX_RSR0(ch):
        return (0x002608 + (ch << 5))
    @staticmethod
    def SN_RX_RSR1(ch):
        return (0x002708 + (ch << 5))

    ''' 接收存储器的读取点 '''
    @staticmethod
    def SN_RX_RD0(ch):
        return (0x002808 + (ch << 5))
    @staticmethod
    def SN_RX_RD1(ch):
        return (0x002908 + (ch << 5))

    def __wiz_writeBuff(self, addr, buf):
        '''
        写数据
        :param addr:
        :param buf:
        :return:
        '''
        self.cs.value(0)
        self.spi.write((addr & 0x00FF0000) >> 16)
        self.spi.write((addr & 0x0000FF00) >> 8)
        self.spi.write((addr & 0x000000F8) + 4)
        self.spi.write(buf)
        self.cs.value(1)

    def __wiz_readBuff(self, addr, length):
        '''
        读数据
        :param addr:
        :param length:
        :return: bytearray 数组
        '''
        dataBuff = bytearray(length)
        if length == 0:    print("read datalen Error!")
        self.cs.value(0)
        self.spi.write((addr & 0x00FF0000) >> 16)
        self.spi.write((addr & 0x0000FF00) >> 8)
        self.spi.write(addr & 0x000000F8)
        self.spi.readinto(dataBuff, write=0x00)
        self.cs.value(1)

        return  dataBuff

    def __setSHAR(self, buf):
        '''
        设置 SHAR 寄存器
        :param buf:
        :return:
        '''
        if len(buf) != 6: print("setSHAR False!")
        self.__wiz_writeBuff(self.SHAR0, buf)

    def __setSUBR(self, buf):
        '''
        设置 SUBR 寄存器
        :param buf:
        :return:
        '''
        if len(buf) != 4: print("setSUBR False!")
        self.__wiz_writeBuff(self.SUBR0, buf)

    def __setGAR(self, buf):
        '''
        设置 GAR 寄存器
        :param buf:
        :return:
        '''
        if len(buf) != 4: print("setGAR False!")
        self.__wiz_writeBuff(self.GAR0, buf)

    def __setSIPR(self, buf):
        '''
        设置SIPR
        :param buf:
        :return:
        '''
        if len(buf) != 4: print("setSIPR False!")
        self.__wiz_writeBuff(self.SIPR0, buf)

    def __getSIPR(self):
        '''
        读 SIPR 寄存器
        :param buf:
        :return:
        '''
        local_ip = self.__wiz_readBuff(self.SIPR0, 4)
        print("W5500 IP地址： %d.%d.%d.%d" % (local_ip[0], local_ip[1], local_ip[2], local_ip[3]))

    def __getSUBR(self):
        '''
        读取 SUBR 寄存器
        :return:
        '''
        subnet = self.__wiz_readBuff(self.SUBR0, 4)
        print("W5500 子网掩码： %d.%d.%d.%d" % (subnet[0], subnet[1], subnet[2], subnet[3]))

    def __getGAR(self):
        '''
        读取 GAR 寄存器
        :return:
        '''
        gateway = self.__wiz_readBuff(self.GAR0, 4)
        print("W5500 网关： %d.%d.%d.%d" % (gateway[0], gateway[1], gateway[2], gateway[3]))

    def __getSn_SR(self, addr):
        '''
        获取 socket 状态
        :param addr:
        :return: byte
        '''
        return (self.IINCHIP_READ(self.SN_SR(addr)))

    def IINCHIP_WRITE(self, addr, data):
        '''
        对外提供接口，只写8位数据，即一字节
        :param addr:
        :param data:
        :return:
        '''
        self.__wiz_writeBuff(addr, data)

    def IINCHIP_READ(self, addr):
        '''
        对外提供接口，只读8位数据，即一字节
        :param addr:
        :return: byte
        '''
        return (self.__wiz_readBuff(addr, 1)[0])

    def wiz_readBuff(self, addr, length):
        '''
        提供给外部调用
        :param addr:
        :param length:
        :return:
        '''
        return (self.__wiz_readBuff(addr, length))

    def getSn_SR(self, addr):
        '''
        对外提供用函数
        :param addr:
        :return:
        '''
        return (self.__getSn_SR(addr))

    def reset_w5500(self):
        '''
        复位操作
        :return:
        '''
        self.reset.value(0)
        time.sleep_us(2)
        self.reset.value(1)
        time.sleep_ms(1600)

    def W5500_setMac(self, mac):
        '''
        设置 mac 地址
        :param mac:
        :return:
        '''
        self.__setSHAR(mac)

    def W5500_setIP(self, sub, gw, lip):
        '''
        设置 ip
        :param sub:
        :param gw:
        :param lip:
        :return:
        '''
        self.__setSUBR(sub)     # 设置子网掩码
        self.__setGAR(gw)       # 设置网关
        self.__setSIPR(lip)     # 设置本地ip

    def W5500_getIP(self):
        self.__getSIPR()        # 获取ip
        self.__getSUBR()        # 获取子网掩码
        self.__getGAR()         # 获取网关

    def getSN_RX_RSR(self, socket_num):
        '''
        给出接收缓冲区中接收数据的大小
        :param socket_num:
        :return:
        '''
        global val, val1
        val = 0
        val1 = 0
        while True:
            val1 = self.IINCHIP_READ(self.SN_RX_RSR0(socket_num))
            val1 = (val1 << 8) + self.IINCHIP_READ(self.SN_RX_RSR1(socket_num))
            if val1 != 0:
                val = self.IINCHIP_READ(self.SN_RX_RSR0(socket_num))
                val = (val << 8) + self.IINCHIP_READ(self.SN_RX_RSR1(socket_num))
            if val == val1:
                break
        return val

    def send_data_processing(self, socket_num, buf):
        '''
        此函数读取Tx写入指针寄存器，并在复制缓冲区中的数据后更新Tx写入指示器寄存器。
        用户应该先读取高位字节，然后再读取低位字节，以获得正确的值
        :param socket_num: socket number
        :param buf: data buffer to send
        :return: TX free buf size
        '''
        if len(buf) == 0:
            print("CH: %d Unexpected1 length 0" % socket_num)
            return

        ptr = self.IINCHIP_READ(self.SN_TX_WR0(socket_num))
        ptr = ((ptr & 0x00ff) << 8) + self.IINCHIP_READ(self.SN_TX_WR1(socket_num))

        addrbsb = (ptr << 8) + (socket_num << 5) + 0x10
        self.__wiz_writeBuff(addrbsb, buf)

        ptr += len(buf)
        self.IINCHIP_WRITE(self.SN_TX_WR0(socket_num), ((ptr & 0xff00) >> 8))
        self.IINCHIP_WRITE(self.SN_TX_WR1(socket_num), (ptr & 0x00ff))


''' socket 类 '''
class mysocket(w5500):
    def __init__(self,local_port=5000, TINCHIP_DBG=False):
        super().__init__()
        self.local_port = local_port
        self.MAX_SOCK_NUM = 8
        self.MAX_SOCK_SIZE = 16384
        self.SSIZE = [0] * self.MAX_SOCK_NUM
        self.RSIZE = [0] * self.MAX_SOCK_NUM
        self.txsize = [2,2,2,2,2,2,2,2]
        self.rxsize = [2,2,2,2,2,2,2,2]

        self.TINCHIP_DBG = TINCHIP_DBG

    @staticmethod
    def Sn_TXMEM_SIZE(ch):
        return (0x001F08 + (ch << 5))

    @staticmethod
    def Sn_RXMEM_SIZE(ch):
        return (0x001E08 + (ch << 5))

    def __IINCHIP_WRITE(self, addr, data):
        '''
        写入 8 位数据到 W5500
        :param addr:
        :param data:
        :return:
        '''
        self.cs.value(0)
        self.spi.write((addr & 0x00FF0000) >> 16)
        self.spi.write((addr & 0x0000FF00) >> 8)
        self.spi.write((addr & 0x000000F8) + 4)
        self.spi.write(data)
        self.cs.value(1)

    def __IINCHIP_READ(self, addr):
        '''
        从 W5500 读出一个 8 位数据
        :param addr:
        :return:
        '''
        data = bytearray(1)
        self.cs.value(0)
        self.spi.write((addr & 0x00FF0000) >> 16)
        self.spi.write((addr & 0x0000FF00) >> 8)
        self.spi.write(addr & 0x000000F8)
        self.spi.readinto(data, write=0x00)
        self.cs.value(1)

        return data[0]

    def checkNum(self, buf):
        '''
        计算字符串校验值
        :param buf:
        :return:
        '''
        global tsum
        global lsum
        tsum = 0
        lsum = 0

        j = len(buf) >> 1

        for i in range(j):
            tsum = buf[i * 2]
            tsum = tsum << 8
            tsum += buf[i * 2 + 1]
            lsum += tsum
        if len(buf) % 2:
            tsum = buf[j * 2]
            lsum += tsum << 8
        sum = lsum
        sum = ~(sum + (lsum >> 16))
        return sum

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
            ((protocol&0x0f) == self.SN_MR_TCP) or
            ((protocol & 0x0f) == self.SN_MR_UDP) or
            ((protocol & 0x0f) == self.SN_MR_IPRAW) or
            ((protocol & 0x0f) == self.SN_MR_MACRAW) or
            ((protocol & 0x0f) == self.SN_MR_PPPOE)
        ):
            self.close(socket_num)  # 关闭
            self.IINCHIP_WRITE(self.SN_MR(socket_num), protocol | flag)
            if port != 0:
                self.IINCHIP_WRITE(self.SN_PORT0(socket_num), ((port&0xff00) >> 8))
                self.IINCHIP_WRITE(self.SN_PORT1(socket_num), (port&0x00ff))
            else:
                self.local_port += 1
                self.IINCHIP_WRITE(self.SN_PORT0(socket_num), ((self.local_port & 0xff00) >> 8))
                self.IINCHIP_WRITE(self.SN_PORT1(socket_num), (self.local_port & 0x00ff))
            self.IINCHIP_WRITE(self.SN_CR(socket_num), self.SN_CR_OPEN)
            while (self.IINCHIP_READ(self.SN_CR(socket_num))):
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
        self.__IINCHIP_WRITE(self.SN_CR(socket_num), self.SN_CR_CLOSE)
        while (self.__IINCHIP_READ(self.SN_CR(socket_num))):
            pass
        self.__IINCHIP_WRITE(self.SN_IR(socket_num), 0xFF)    # clear

    def socketBuf_Init(self):
        '''
        此功能根据使用的信道设置发送和接收缓冲区大小
        :return:
        '''
        ssum = 0
        rsum = 0
        for i in range(self.MAX_SOCK_NUM):
            self.__IINCHIP_WRITE(self.Sn_TXMEM_SIZE(i), self.txsize[i])
            self.__IINCHIP_WRITE(self.Sn_RXMEM_SIZE(i), self.rxsize[i])
            if self.TINCHIP_DBG:
                print("tx_size[%d]: %d, Sn_TXMEM_SIZE = %d" % (i, self.txsize[i],
                        self.__IINCHIP_READ(self.Sn_TXMEM_SIZE(i))[0]))
                print("rx_size[%d]: %d, Sn_RXMEM_SIZE = %d" % (i, self.rxsize[i],
                        self.__IINCHIP_READ(self.Sn_RXMEM_SIZE(i))[0]))
            self.SSIZE[i] = 0
            self.RSIZE[i] = 0
            if ssum <= self.MAX_SOCK_SIZE:  self.SSIZE[i] = self.txsize[i]*1024
            if rsum <= self.MAX_SOCK_SIZE:  self.RSIZE[i] = self.rxsize[i]*1024
            ssum += self.SSIZE[i]
            rsum += self.RSIZE[i]

    def sendto(self, socket_num, buf, addr, port):
        global ret
        ret = 0
        if len(buf) > self.getIINCHIP_TxMAX(socket_num):
            ret = self.getIINCHIP_TxMAX(socket_num)
        else:   ret = len(buf)

        if (addr[0] == 0x00 and addr[1] == 0x00 and addr[2] == 0x00 and addr[3] == 0x00) or \
                (port == 0x00):
            ret = 0
        else:
            self.IINCHIP_WRITE(self.SN_DIPR0(socket_num), addr[0])
            self.IINCHIP_WRITE(self.SN_DIPR1(socket_num), addr[1])
            self.IINCHIP_WRITE(self.SN_DIPR2(socket_num), addr[2])
            self.IINCHIP_WRITE(self.SN_DIPR3(socket_num), addr[3])
            self.IINCHIP_WRITE(self.SN_DPORT0(socket_num), ((port & 0xff00) >> 8))
            self.IINCHIP_WRITE(self.SN_DPORT1(socket_num), (port & 0x00ff))
            # copy data
            self.send_data_processing(socket_num, buf)
            self.IINCHIP_WRITE(self.SN_CR(socket_num), self.SN_CR_SEND)
            while (self.IINCHIP_READ(self.SN_CR(socket_num))):
                pass
            while ((self.IINCHIP_READ(self.SN_IR(socket_num)) & self.SN_IR_SEND_OK) != self.SN_IR_SEND_OK):
                if self.IINCHIP_READ(self.SN_IR(socket_num)) & self.SN_IR_TIMEOUT:
                    self.IINCHIP_WRITE(self.SN_IR(socket_num), (self.SN_IR_SEND_OK | self.SN_IR_TIMEOUT))
                    return 0
            self.IINCHIP_WRITE(self.SN_IR(socket_num), self.SN_IR_SEND_OK)

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
            ptr = self.IINCHIP_READ(self.SN_RX_RD0(socket_num))
            ptr = ((ptr & 0x00ff) << 8) + self.IINCHIP_READ(self.SN_RX_RD1(socket_num))
            addrbsb = (ptr << 8) + (socket_num << 5) + 0x18

            ret = self.IINCHIP_READ(self.SN_MR(socket_num)) & 0x07

            if ret == self.SN_MR_UDP:
                head = self.wiz_readBuff(addrbsb, 0x08)
                ptr += 8
                addr[0:4] = head[0:4]       # # IP address to receive.
                port[0] = head[4]
                port[1] = (port[0] << 8) + head[5]
                datalen = (head[6] << 8) + head[7]

                addrbsb = (ptr << 8) + (socket_num << 5) + 0x18
                buf = self.wiz_readBuff(addrbsb, datalen)
                ptr += datalen

                self.IINCHIP_WRITE(self.SN_RX_RD0(socket_num), ((ptr & 0xff00) >> 8))
                self.IINCHIP_WRITE(self.SN_RX_RD1(socket_num), (ptr & 0x00ff))
            elif ret == self.SN_MR_IPRAW:
                head = self.wiz_readBuff(addrbsb, 0x06)
                ptr += 6
                addr[0:4] = head[0:4]
                datalen = (head[4] << 8) + head[5]

                addrbsb = (ptr << 8) + (socket_num << 5) + 0x18
                buf = self.wiz_readBuff(addrbsb, datalen)
                ptr += datalen

                self.IINCHIP_WRITE(self.SN_RX_RD0(socket_num), ((ptr & 0xff00) >> 8))
                self.IINCHIP_WRITE(self.SN_RX_RD1(socket_num), (ptr & 0x00ff))
                print("socket Status: SN_MR_IPRAW")
            elif ret == self.SN_MR_MACRAW:
                head = self.wiz_readBuff(addrbsb, 0x02)
                ptr += 2
                datalen = (head[0] << 8) + head[1] - 2
                if datalen > 1514:
                    print("data_len over 1514")
                    while True:
                        pass

                addrbsb = (ptr << 8) + (socket_num << 5) + 0x18
                buf = self.wiz_readBuff(addrbsb, datalen)
                ptr += datalen

                self.IINCHIP_WRITE(self.SN_RX_RD0(socket_num), ((ptr & 0xff00) >> 8))
                self.IINCHIP_WRITE(self.SN_RX_RD1(socket_num), (ptr & 0x00ff))
            self.IINCHIP_WRITE(self.SN_CR(socket_num), self.SN_CR_RECV)
            while (self.IINCHIP_READ(self.SN_CR(socket_num))):
                pass

        print("Recv Datalen: %d" % datalen)
        return [datalen, buf]

    def getIINCHIP_TxMAX(self, socket_num):
        '''
        此函数用于获取要接收的最大大小
        :param socket_num:
        :return:
        '''
        return self.SSIZE[socket_num]


class myPing(mysocket):
    def __init__(self):
        super().__init__()
        self.pingType = 8
        self.pingCode = 0
        self.pingCheckNum = 0
        self.pingID = 0x1234
        self.pingSeqNum = 0x4321
        self.BUF_LEN = 128

        self.req = 0
        self.rep = 0

        self.PING_REPLY = 0
        self.PING_REQUEST = 8

        self.ping_reply_received = False

    def __swaps(self, data):
        '''
        大小端交换
        :param data:
        :return:
        '''
        ret = struct.pack('<H', data)
        return (ret)

    def pingCmd(self, socket_num, addr):
        global cnt
        cnt = 0
        for i in range(5):  # ping 5次
            time.sleep_ms(1000)
            SOCK_STATE = self.getSn_SR(socket_num)
            if SOCK_STATE == self.SOCKET_CLOSED:
                self.close(socket_num)
                self.IINCHIP_WRITE(self.SN_PROTO(socket_num), self.IPPROTO_ICMP)    # 设置 ICMP 协议
                if self.socket(socket_num, self.SN_MR_IPRAW, 3000, 0) != 0:
                    pass
                while (self.getSn_SR(socket_num) != self.SOCK_IPRAW):
                    pass
                time.sleep_ms(2)
            elif SOCK_STATE == self.SOCK_IPRAW:
                self.ping_request(socket_num, addr)
                self.req += 1
                while True:
                    rlen = self.getSN_RX_RSR(socket_num)
                    if rlen > 0:
                        self.ping_reply(socket_num, addr, rlen)
                        time.sleep_us(500)
                        self.rep += 1
                        # 回复完成
                        if self.ping_reply_received:
                            break
                    elif cnt > 1000:
                        print("Request Time out.")
                        cnt = 0
                        break
                    else:
                        cnt += 1
                        time.sleep_ms(1)
            else:
                print("socket_state Error!")
            if self.rep != 0:
                print("Ping Request = %d, PING_Reply = %d" % (self.req, self.rep))
                if self.rep == self.req:
                    print("PING SUCCESS")
                else:
                    print("REPLY_ERROR")

    def ping_request(self, socket_num, addr):
        buf = bytearray(136)

        self.ping_reply_received = False

        self.pingID += 1
        self.pingSeqNum += 1

        buf[0] = self.pingType      # 类型
        buf[1] = self.pingCode      # 代码
        buf[2] = 0x00               # 检验和
        buf[3] = 0x00               # 检验和
        ''' 这4个字节取决于 ICMP 报文的类型 '''
        buf[4] = self.__swaps(self.pingID)[0]
        buf[5] = self.__swaps(self.pingID)[1]
        buf[6] = self.__swaps(self.pingSeqNum)[0]
        buf[7] = self.__swaps(self.pingSeqNum)[1]

        for i in range(self.BUF_LEN):
            buf[i+8] = i % 8

        ''' 计算响应次数 '''
        temp = self.checkNum(buf)
        buf[2] = (temp >> 8) & 0xff
        buf[3] = temp & 0xff
        print('Send Data: ', end='')
        for i in buf:
            print(hex(i), end= " ")
        print("")
        if self.sendto(socket_num, buf, addr, 3000) == 0:
            print("Fail to send ping-reply packet")
        else:
            print("\n正在 Ping： %d.%d.%d.%d" % (addr[0], addr[1], addr[2], addr[3]))
        return 0

    def ping_reply(self, socket_num, addr, rlen):
        '''
        解析 Ping 回复
        :param socket_num:
        :param addr:
        :param rlen:
        :return:
        '''
        global temp_checksum
        temp_checksum = 0
        port = 3000

        ret = self.recvfrom(socket_num, rlen, addr, port)
        PingReplay = bytearray(ret[0])  # 获取长度
        databuf = ret[1]                # 获取数据

        ''' 输出数据 '''
        print("Recv Data: ", end='')
        for i in databuf:
            print(hex(i), end=' ')
        print('')
        ''' 输出数据 '''

        PingReplay[0:] = databuf[0:]    # 数据拷贝
        del databuf # 清缓存
        if PingReplay[0] == self.PING_REPLY:
            temp_checksum = ~self.checkNum(PingReplay) & 0xffff
            # 检查 ping 回复次数
            if temp_checksum != 0xffff:
                print("tmp_checksum = %x" % temp_checksum)
            else:
                print("来自 %d.%d.%d.%d 的回复 ：ID=%x 字节=%d" %
                       (addr[0], addr[1], addr[2], addr[3],
                        (PingReplay[4] << 8 | PingReplay[5]),
                        (rlen + 6)))
                self.ping_reply_received = True
        elif PingReplay[0] == self.PING_REQUEST:
            temp_checksum = PingReplay[3] << 8 + PingReplay[2]
            # TODO [此处与原函数有区别，后续开发注意]
            print("Request from %d.%d.%d.%d  ID:%x SeqNum:%x  :data size %d bytes" %
                  (addr[0], addr[1], addr[2], addr[3],
                   PingReplay[4] << 8 | PingReplay[5],
                   PingReplay[6] << 8 | PingReplay[7],
                   (rlen + 6)))
            self.ping_reply_received = True
        else:
            print("Unkonwn msg.")



if __name__ == '__main__':
    mac = bytearray([0x00,0x08,0xdc,0x11,0x11,0x11])    # mac 地址
    subnet = bytearray([255,255,255,0])                 # 子网掩码
    gateway = bytearray([192,168,5,1])                  # 网关
    local_ip = bytearray([192,168,5,5])                 # ip地址

    # myW5500 = w5500()
    # SOCKET = mysocket(TINCHIP_DBG=True)
    myping = myPing()

    myping.reset_w5500()
    myping.W5500_setMac(mac)
    myping.W5500_setIP(subnet, gateway, local_ip)
    myping.W5500_getIP()

    myping.socketBuf_Init()

    while True:
        print("------正在执行ping-----")
        time.sleep_ms(2000)
        myping.pingCmd(0, bytearray([192, 168, 5, 20]))
        pass

