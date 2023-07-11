from machine import SPI
from fpioa_manager import fm
from Maix import GPIO
import time
import struct
import sensor, image

''' SPI 类 '''
class spiChat():
    def __init__(self, baudrate=10000000):
        fm.register(27, fm.fpioa.GPIOHS10, force=True)  # cs
        self.cs = GPIO(GPIO.GPIOHS10, GPIO.OUT)    # cs 脚
        self.cs.value(1)
        self.spi = SPI(SPI.SPI1, mode=SPI.MODE_MASTER, baudrate=baudrate, polarity=0, phase=0, bits=8, firstbit=SPI.MSB, sck=28, mosi=29, miso=30)

''' W5500类 '''
class w5500(spiChat):
    def __init__(self):
        super().__init__()
        fm.register(31, fm.fpioa.GPIOHS11, force=True)  # reset
        self.reset = GPIO(GPIO.GPIOHS11, GPIO.OUT)  # reset 脚
        self.reset.value(1)

        self.SHAR0 = 0x000900
        self.SUBR0 = 0x000500
        self.SIPR0 = 0x000F00
        self.GAR0  = 0x000100

        ''' socket 端口选择 '''
        self.SOCK_TCPS = 0
        self.SOCK_TCPC = 1

        self.SOCKET_CLOSED = 0x00    # closed
        self.SOCK_IPRAW = 0x32      # ip raw mode socket

        ''' SN_SR values '''
        self.SOCK_CLOSED = 0x00     # closed
        self.SOCK_INIT = 0x13       # init state
        self.SOCK_SYNSENT = 0x15    # connection state
        self.SOCK_ESTABLISHED = 0x17# success to connect
        self.SOCK_CLOSE_WAIT = 0x1C # closing state

        ''' SN_IR value '''
        self.SN_IR_SEND_OK = 0x10   # complete sending
        self.SN_IR_TIMEOUT = 0x08   # assert timeout
        self.SN_IR_CON  = 0x01      # established connection

        ''' SN_MR values '''
        self.SN_MR_TCP = 0x01       # tcp
        self.SN_MR_UDP = 0x02       # udp
        self.SN_MR_IPRAW = 0x03     # IP LAYER RAW SOCK
        self.SN_MR_MACRAW = 0x04    # MAC LAYER RAW SOCK
        self.SN_MR_PPPOE = 0x05     # PPPoE
        self.SN_MR_ND = 0x20

        ''' SN_CR values '''
        self.SN_CR_OPEN = 0x01      # initialize or open socket
        self.SN_CR_LISTEN = 0x02    # wait connection request in tcp mode(Server mode)
        self.SN_CR_CONNECT = 0x04  # send connection request in tcp mode(Client mode)
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

    ''' 传输可用内存大小寄存器 '''
    @staticmethod
    def SN_TX_FSR0(ch):
        return (0x002008 + (ch << 5))
    @staticmethod
    def SN_TX_FSR1(ch):
        return (0x002108 + (ch << 5))

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
        return local_ip

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

    def __getSn_IR(self, addr):
        '''
        此功能用于读取中断和 SOCKET 状态寄存器
        :param socket_num:
        :return:
        '''
        return (self.IINCHIP_READ(self.SN_IR(addr)))

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

    def getSn_IR(self, addr):
        '''
        对外提供函数
        :param addr:
        :return:
        '''
        return (self.__getSn_IR(addr))

    def setSn_IR(self, addr, val):
        '''
        此功能用于读取中断和 SOCKET 状态寄存器
        :param addr:
        :return:
        '''
        return (self.IINCHIP_WRITE(self.SN_IR(addr), val))

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
        ip = self.__getSIPR()        # 获取ip
        self.__getSUBR()        # 获取子网掩码
        self.__getGAR()         # 获取网关
        return ip

    def getSN_TX_FSR(self, socket_num):
        '''
        获取发送缓冲区大小
        :param socket_num:
        :return:
        '''
        global val, val1
        val = 0
        val1 = 0
        while True:
            val1 = self.IINCHIP_READ(self.SN_TX_FSR0(socket_num))
            val1 = (val1 << 8) + self.IINCHIP_READ(self.SN_TX_FSR1(socket_num))
            if val1 != 0:
                val = self.IINCHIP_READ(self.SN_TX_FSR0(socket_num))
                val = (val << 8) + self.IINCHIP_READ(self.SN_TX_FSR1(socket_num))
            if val == val1:
                break
        return val

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

    def recv_data_processing(self, socket_num, len):
        '''
        此函数读取Rx读取指针寄存器,并且在从接收缓冲器复制数据之后更新Rx写入指针寄存器。
        用户应该先读取高位字节，然后再读取低位字节，以获得正确的值。
        :param socket_num:
        :param buf:
        :return:
        '''
        if len == 0:
            print("CH: %d Unexpected2 length 0" % socket_num)
            return

        ptr = self.IINCHIP_READ(self.SN_RX_RD0(socket_num))
        ptr = ((ptr & 0x00ff) << 8) + self.IINCHIP_READ(self.SN_RX_RD1(socket_num))

        addrbsb = (ptr << 8) + (socket_num << 5) + 0x18
        buf = self.__wiz_readBuff(addrbsb, len)

        ptr += len
        self.IINCHIP_WRITE(self.SN_RX_RD0(socket_num), ((ptr & 0xff00) >> 8))
        self.IINCHIP_WRITE(self.SN_RX_RD1(socket_num), (ptr & 0x00ff))

        return buf


''' socket 类 '''
class mysocket(w5500):
    def __init__(self, local_port=5000, quality=80, K210_DEBUG=False):
        super().__init__()
        self.local_port = local_port
        self.MAX_SOCK_NUM = 8
        self.MAX_SOCK_SIZE = 16384
        self.SSIZE = [0] * self.MAX_SOCK_NUM
        self.RSIZE = [0] * self.MAX_SOCK_NUM
        self.txsize = [2,2,2,2,2,2,2,2]
        self.rxsize = [2,2,2,2,2,2,2,2]

        self.quality = quality
        self.K210_DEBUG = K210_DEBUG

        self.beforeStatus = None
        self.outlineFlag = True

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
            if self.K210_DEBUG:
                print("tx_size[%d]: %d, Sn_TXMEM_SIZE = %d" % (i, self.txsize[i],
                        self.__IINCHIP_READ(self.Sn_TXMEM_SIZE(i))))
                print("rx_size[%d]: %d, Sn_RXMEM_SIZE = %d" % (i, self.rxsize[i],
                        self.__IINCHIP_READ(self.Sn_RXMEM_SIZE(i))))
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

    def listen(self, socket_num):
        '''
        此功能为处于被动（服务器）模式的通道建立了连接。
        此函数等待来自对等方的请求。
        :param socket_num:
        :return:
        '''
        if self.IINCHIP_READ(self.SN_SR(socket_num)) == self.SOCK_INIT:
            self.IINCHIP_WRITE(self.SN_CR(socket_num), self.SN_CR_LISTEN)
            while self.IINCHIP_READ(self.SN_CR(socket_num)):
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
            self.IINCHIP_WRITE(self.SN_DIPR0(socket_num), addr[0])
            self.IINCHIP_WRITE(self.SN_DIPR1(socket_num), addr[1])
            self.IINCHIP_WRITE(self.SN_DIPR2(socket_num), addr[2])
            self.IINCHIP_WRITE(self.SN_DIPR3(socket_num), addr[3])
            self.IINCHIP_WRITE(self.SN_DPORT0(socket_num), ((port & 0xff00) >> 8))
            self.IINCHIP_WRITE(self.SN_DPORT1(socket_num), (port & 0x00ff))
            self.IINCHIP_WRITE(self.SN_CR(socket_num), self.SN_CR_CONNECT)
            while self.IINCHIP_READ(self.SN_CR(socket_num)):
                pass
            while (self.IINCHIP_READ(self.SN_SR(socket_num)) != self.SOCK_SYNSENT):
                if self.IINCHIP_READ(self.SN_SR(socket_num)) == self.SOCK_ESTABLISHED:
                    break
                if (self.getSn_IR(socket_num) & self.SN_IR_TIMEOUT):
                    self.IINCHIP_WRITE(self.SN_IR(socket_num), self.SN_IR_TIMEOUT)
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

        if len > self.getIINCHIP_TxMAX(socket_num):
            ret = self.getIINCHIP_TxMAX(socket_num)
        else:
            ret = len

        while True:
            freesize = self.getSN_TX_FSR(socket_num)
            status = self.IINCHIP_READ(self.SN_SR(socket_num))
            if (status != self.SOCK_ESTABLISHED) and (status != self.SOCK_CLOSE_WAIT):
                ret = 0
                break
            if freesize >= ret:
                break
        # copy data
        self.send_data_processing(socket_num, buf[:len])
        self.IINCHIP_WRITE(self.SN_CR(socket_num), self.SN_CR_SEND)
        # wait to process the command...
        while self.IINCHIP_READ(self.SN_CR(socket_num)):
            pass
        while (self.IINCHIP_READ(self.SN_IR(socket_num)) & self.SN_IR_SEND_OK) != self.SN_IR_SEND_OK:
            status = self.IINCHIP_READ(self.SN_SR(socket_num))
            if ((status != self.SOCK_ESTABLISHED) and (status != self.SOCK_CLOSE_WAIT)):
                print("SEND_OK Problem!!")
                self.close(socket_num)
                return 0
        self.IINCHIP_WRITE(self.SN_IR(socket_num), self.SN_IR_SEND_OK)
        # TODO [原函数 __DEF_IINCHIP_INT__ 未知]
        self.IINCHIP_WRITE(self.SN_IR(socket_num), self.SN_IR_SEND_OK)

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
            buf = self.recv_data_processing(socket_num, len)
            self.IINCHIP_WRITE(self.SN_CR(socket_num), self.SN_CR_RECV)
            while self.IINCHIP_READ(self.SN_CR(socket_num)):
                pass
            return buf
        else:
            return 0

    def do_tcp_server(self):
        '''
        tcp 服务端函数
        :return:
        '''
        SOCKET_STATUS = self.getSn_SR(self.SOCK_TCPS)   # 获取 socket 状态
        if SOCKET_STATUS == self.SOCK_CLOSED:   # 关闭状态
            self.socket(self.SOCK_TCPS, self.SN_MR_TCP, self.local_port, self.SN_MR_ND) # 打开 SOCKET
        elif SOCKET_STATUS == self.SOCK_INIT:   # SOCKET 已初始化
            self.listen(self.SOCK_TCPS) # 建立监听
        elif SOCKET_STATUS == self.SOCK_ESTABLISHED:    # SOCKET 处于链接建立状态
            if self.getSn_IR(self.SOCK_TCPS) & self.SN_IR_CON:
                self.setSn_IR(self.SOCK_TCPS, self.SN_IR_CON)   # 清除接收中断标志位
            length = self.getSN_RX_RSR(self.SOCK_TCPS)  # 定义 length 为已接收数据的长度
            if length > 0:
                recvBuff = self.recv(self.SOCK_TCPS, length) # 接收来自 Client 的数据
                recvBuff.append(0x00) # 添加字符串结束符
                print("SOCKET Recv-> %s" % recvBuff[:length])
                self.send(self.SOCK_TCPS, recvBuff, length) # 向 Client 发送数据
        elif SOCKET_STATUS == self.SOCK_CLOSE_WAIT: # SOCKET 处于等待关闭状态
            self.close(self.SOCK_TCPS)

    def image_send(self):
        '''
        发送图像
        :return:
        '''
        clock.tick()
        start_time = time.ticks_cpu()   # 获取时间
        img = sensor.snapshot()         # 拍摄一张图片
        img = img.compress(quality=self.quality).to_bytes()     # 压缩图像
        total_size = len(img) # 计算图像总字节
        int_pieces = total_size // 1024
        end_pieces = total_size % 1024
        head_buf = str(start_time).encode() + ".".encode() + str(total_size).encode()
        # 发送数据头
        self.send(self.SOCK_TCPC, head_buf, len(head_buf))
        while True:
            ''' 等待服务端处理完成 '''
            if (self.getSn_IR(self.SOCK_TCPC) & self.SN_IR_CON):
                self.setSn_IR(self.SOCK_TCPC, self.SN_IR_CON)  # 清除接收中断标志位
            length = self.getSN_RX_RSR(self.SOCK_TCPC)  # 定义 length 为已接收数据的长度
            if length > 0:
                recvBuff = self.recv(self.SOCK_TCPC, length)  # 接收来自 Server 的数据
                if recvBuff.decode() == 'ok':   # 收到服务端的完成信号
                    ''' 分包发送图像 '''
                    # 整包
                    for i in range(int_pieces):
                        self.send(self.SOCK_TCPC, img[i*1024:(i+1)*1024], 1024)
                    # 剩下的包
                    self.send(self.SOCK_TCPC, img[int_pieces*1024:], end_pieces)
                    break
        while True:
            ''' 等待服务端处理完成 '''
            if (self.getSn_IR(self.SOCK_TCPC) & self.SN_IR_CON):
                self.setSn_IR(self.SOCK_TCPC, self.SN_IR_CON)  # 清除接收中断标志位
            length = self.getSN_RX_RSR(self.SOCK_TCPC)  # 定义 length 为已接收数据的长度
            if length > 0:
                recvBuff = self.recv(self.SOCK_TCPC, length)  # 接收来自 Server 的数据
                if recvBuff.decode() == 'ok':  # 收到服务端的完成信号
                    print(" [W550]@Client -> 处理时间: {} FPS: {}".format(time.ticks_cpu() - start_time, clock.fps()))
                    break
        # DEBUG 信息输出
        if self.K210_DEBUG:
            print(" [W550]@Client -> {}.{}".format(start_time, total_size))


    def do_tcp_client(self, remote_ip, remote_port):
        '''
        tcp 客户端函数
        :return:
        '''
        SOCKET_STATUS = self.getSn_SR(self.SOCK_TCPC)   # 获取 socket 状态

        if SOCKET_STATUS == self.SOCK_CLOSED:   # socket 处于关闭状态
            if SOCKET_STATUS != self.beforeStatus and self.outlineFlag:
                print("-> Now Status: SOCK_CLOSED")
            self.local_port += 1
            self.socket(self.SOCK_TCPC, self.SN_MR_TCP, self.local_port, self.SN_MR_ND)
        elif SOCKET_STATUS == self.SOCK_INIT:   # socket 处于初始化状态
            if SOCKET_STATUS != self.beforeStatus and self.outlineFlag:
                print("-> Now Status: SOCK_INIT")
            self.connect(self.SOCK_TCPC, remote_ip, remote_port)    # socket 连接服务器
        elif SOCKET_STATUS == self.SOCK_ESTABLISHED:    # socket 处于连接状态
            self.outlineFlag = True
            if SOCKET_STATUS != self.beforeStatus and self.outlineFlag:
                print("-> Now Status: SOCK_ESTABLISHED ...... ❤")
                print("-> Console log: {}\n".format(self.K210_DEBUG))
            ''' 发送图像 '''
            self.image_send()
            ''' 发送图像 '''
            # if (self.getSn_IR(self.SOCK_TCPC) & self.SN_IR_CON):
            #     self.setSn_IR(self.SOCK_TCPC, self.SN_IR_CON)   #清除接收中断标志位
            # length = self.getSN_RX_RSR(self.SOCK_TCPC)  # 定义 length 为已接收数据的长度
            # if length > 0:
            #     recvBuff = self.recv(self.SOCK_TCPC, length)    # 接收来自 Server 的数据
            #     recvBuff.append(0x00)   # 添加字符串结束符
            #     print("\nrecvBuff-> %s" % (recvBuff.decode()))
            #     self.send(self.SOCK_TCPC, recvBuff, length) # 向 Server 发送数据
        elif SOCKET_STATUS == self.SOCK_CLOSE_WAIT: # socket 处于等待关闭状态
            if SOCKET_STATUS != self.beforeStatus and self.outlineFlag:
                print("-> Now Status: SOCK_CLOSE_WAIT")
            self.close(self.SOCK_TCPC)
        elif SOCKET_STATUS == self.SOCK_SYNSENT:
            if SOCKET_STATUS != self.beforeStatus and self.outlineFlag:
                self.outlineFlag = False
                print("-> Now Status: Server outline ...... ✖")
        self.beforeStatus = SOCKET_STATUS   # 更新上次输出信息


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


def ping_function(me, ping_ip):
    '''
    ping 功能
    :param me:
    :param ping_ip:
    :return:
    '''
    print("------正在执行ping-----")
    time.sleep_ms(2000)
    me.pingCmd(0, ping_ip)


if __name__ == '__main__':
    mac = bytearray([0x00,0x08,0xdc,0x11,0x11,0x11])    # mac 地址
    subnet = bytearray([255,255,255,0])                 # 子网掩码
    gateway = bytearray([192,168,5,1])                  # 网关
    local_ip = bytearray([192,168,5,5])                 # ip地址

    remote_ip = bytearray([192, 168, 5, 20])            # 服务端ip
    remotr_host = 5500                                  # 服务端端口

    time.sleep_ms(10000)

    # myW5500 = w5500()
    # SOCKET = mysocket(TINCHIP_DBG=True)
    myping = myPing()

    myping.reset_w5500()
    myping.W5500_setMac(mac)
    myping.W5500_setIP(subnet, gateway, local_ip)
    ip = myping.W5500_getIP()

    if ip != local_ip:
        while True:
            time.sleep_ms(2000)
            myping.reset_w5500()
            myping.W5500_setMac(mac)
            myping.W5500_setIP(subnet, gateway, local_ip)
            ip = myping.W5500_getIP()
            if ip == local_ip:
                break

    myping.socketBuf_Init()

    print('W5500 为客户端')
    print("连接到服务端 %d.%d.%d.%d:%d" % (remote_ip[0], remote_ip[1], remote_ip[2],
                                          remote_ip[3], remotr_host))

    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=2000)

    clock = time.clock()

    while True:
        myping.do_tcp_client(remote_ip, remotr_host)
        pass

