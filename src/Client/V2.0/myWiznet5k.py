'''
    此文件是 W5500 的寄存器相关配置文件
'''
from myWiznet5k_spi import *
from micropython import const
import time
import machine
from fpioa_manager import fm
from Maix import GPIO

# 网关 IP 寄存器地址
GAR0 = const(0x000100)
# 子网掩码 寄存器地址
SUBR0 = const(0x000500)
# MAC 地址寄存器
SHAR0 = const(0x000900)
# 源 IP 寄存器地址
SIPR0 = const(0x000F00)

''' SN_MR values '''
SN_MR_TCP    = const(0x01)      # tcp
SN_MR_UDP    = const(0x02)      # udp
SN_MR_IPRAW  = const(0x03)      # IP LAYER RAW SOCK
SN_MR_MACRAW = const(0x04)      # MAC LAYER RAW SOCK
SN_MR_PPPOE  = const(0x05)      # PPPoE
SN_MR_ND     = const(0x20)      # No Delayed Ack(TCP) flag

''' SN_CR values '''
SN_CR_OPEN    = const(0x01)     # initialize or open socket
SN_CR_LISTEN  = const(0x02)     # wait connection request in tcp mode(Server mode)
SN_CR_CONNECT = const(0x04)     # send connection request in tcp mode(Client mode)
SN_CR_CLOSE   = const(0x10)     # close socket
SN_CR_SEND    = const(0x20)     # update txbuf pointer, send data
SN_CR_RECV    = const(0x40)     # update rxbuf pointer, recv data

''' SN_IR value '''
SN_IR_SEND_OK = const(0x10)     # complete sending
SN_IR_TIMEOUT = const(0x08)     # assert timeout
SN_IR_RECV    = const(0x04)     # receiving data
SN_IR_CON     = const(0x01)     # established connection

''' SN_SR values '''
SOCK_CLOSED      = const(0x00)  # closed
SOCK_INIT        = const(0x13)  # init state
SOCK_SYNSENT     = const(0x15)  # connection state
SOCK_ESTABLISHED = const(0x17)  # success to connect
SOCK_CLOSE_WAIT  = const(0x1C)  # closing state
SOCK_UDP         = const(0x22)  # udp socket
SOCK_IPRAW       = const(0x32)  # ip raw mode socket

''' IP PROTOCOL '''
IPPROTO_ICMP = const(1)         # Control message protocol

class myWiznet5k:
    '''
    W5500 对象
    '''
    def __init__(self, spi_port, config=None, reset_pin=None, mac=None, local_ip=None, subnet=None,
                 gateway=None, is_reset=True, is_debug=False):
        self.__device = spi_port    # spi 对象
        self.__reset = reset_pin    # 复位脚

        if mac is not None and local_ip is not None and subnet is not None and gateway is not None:
            self.mac = mac
            self.local_ip = local_ip
            self.subnet = subnet
            self.gateway = gateway
        else:
            assert type(config) is tuple, errorStr + 'config set error, must be tuple!'
            self.mac = config[0]             # mac 地址
            self.local_ip = config[1]   # 本地 ip
            self.subnet = config[2]       # 子网掩码
            self.gateway = config[3]     # 网关

        self.MAX_SOCK_NUM = 8
        self.MAX_SOCK_SIZE = 16384
        self.SSIZE = [0] * self.MAX_SOCK_NUM
        self.RSIZE = [0] * self.MAX_SOCK_NUM
        self.txsize = [2, 2, 2, 2, 2, 2, 2, 2]
        self.rxsize = [2, 2, 2, 2, 2, 2, 2, 2]

        self.__is_debug = is_debug  # 是否打印信息

        self.__init_function(is_reset)  # 系统初始化设置

    def __init_function(self, is_reset):
        '''
        系统初始化操作
        :param is_reset: 是否复位
        :return: None
        '''
        # 进行复位
        if is_reset:
            assert self.__reset is not None, errorStr + 'please set reset pin!!!'
            if self.__is_debug:
                print(debugStr + "_debug: reset now ...")
            self.reset_w5500()  # 复位
            if self.__is_debug:
                print(debugStr + "_debug: reset succeed !")

            if self.mac is not None and self.local_ip is not None and self.subnet is not None and self.gateway is not None:
                if self.__is_debug:
                    print(debugStr + "_debug: set Info now ...")
                self.set_mac()  # 设置 mac
                self.set_ip()  # 设置 ip 以及其他信息
                if self.__is_debug:
                    print(debugStr + "_debug: set Info succeed !")

            time.sleep_ms(500)

            ret = self.get_ip()  # 获取设置信息
            if ret != self.local_ip:
                print(debugStr + "_System: system will reset in 1 second ... ㊥")
                time.sleep_ms(1000)
                machine.reset()  # 系统复位
            else:
                print(debugStr + 'Setting succeed! Start progarm ... ☯')

            self.socketBuf_Init()   # 发送、接收缓冲区初始化

    @staticmethod
    def SN_MR(ch):  # 套接字模式寄存器
        return (0x000008 + (ch << 5))
    @staticmethod
    def SN_CR(ch):  # 通道Sn_CR寄存器
        return (0x000108 + (ch << 5))
    @staticmethod
    def SN_IR(ch):  # 通道中断寄存器
        return (0x000208 + (ch << 5))
    @staticmethod
    def SN_SR(ch):  # 通道状态寄存器
        return (0x000308 + (ch << 5))

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

    ''' 接收数据内存大小寄存器 '''
    @staticmethod
    def Sn_TXMEM_SIZE(ch):
        return (0x001F08 + (ch << 5))

    ''' 发送数据内存大小寄存器 '''
    @staticmethod
    def Sn_RXMEM_SIZE(ch):
        return (0x001E08 + (ch << 5))

    ''' 传输可用内存大小寄存器 '''
    @staticmethod
    def SN_TX_FSR0(ch):
        return (0x002008 + (ch << 5))
    @staticmethod
    def SN_TX_FSR1(ch):
        return (0x002108 + (ch << 5))

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

    def __set_information(self, info, addr, length, str):
        '''
        设置参数信息
        :param info: 设置对象
        :param addr: 设置对象的地址
        :param length: 设置对象本来需要的长度
        :param str: 设置对象的字符
        :return: None
        '''
        assert len(info) == length, errorStr + '{} length error!!!'.format(str)
        self.__device.wiz_writeBuff(addr, info)
        if self.__is_debug:
            if length == 4:
                print(debugStr + '_debug: Set %s: %d.%d.%d.%d' % (str, info[0], info[1], info[2], info[3]))

    def __get_information(self, addr, str):
        '''
        获取参数信息
        :param addr: 获取的参数的地址
        :return: 返回参数信息
        '''
        time.sleep_ms(1)
        dateRecv = self.__device.wiz_readBuff(addr, 4)
        print(debugStr + '%s: %d.%d.%d.%d' % (str, dateRecv[0], dateRecv[1], dateRecv[2], dateRecv[3]))

        return dateRecv

    def getSn_SR(self, addr):
        '''
        获取 socket 状态
        '''
        return self.__device.IINCHIP_READ(self.SN_SR(addr))

    def getSn_IR(self, addr):
        '''
        读取 socket 中断状态寄存器
        '''
        return  self.__device.IINCHIP_READ(self.SN_IR(addr))

    def setSn_IR(self, addr, val):
        '''
        设置 socket 中断状态寄存器
        '''
        self.__device.IINCHIP_WRITE(self.SN_IR(addr), val)

    def reset_w5500(self):
        '''
        软件复位 w5500
        :return: None
        '''
        self.__reset.value(0)
        time.sleep_us(2)
        self.__reset.value(1)
        time.sleep_ms(1600)

    def set_mac(self, mac=None):
        '''
        设置 mac 地址
        :param mac: mac 地址
        :return: None
        '''
        if mac is not None:
            self.__set_information(mac, SHAR0, 6, 'mac')  # 设置 mac 地址
        else:
            self.__set_information(self.mac, SHAR0, 6, 'mac')   # 设置 mac 地址

    def set_ip(self, local_ip=None, subnet=None, gateway=None):
        '''
        设置 ip、子网掩码以及网关
        :param local_ip: ip 地址
        :param subnet: 子网掩码
        :param gateway: 网关
        :return: None
        '''
        if local_ip is not None:
            self.__set_information(local_ip, SIPR0, 4, 'local_ip')  # 设置本地 ip
        else:
            self.__set_information(self.local_ip, SIPR0, 4, 'local_ip') # 设置本地 ip

        if subnet is not None:
            self.__set_information(subnet, SUBR0, 4, 'subnet')  # 设置子网掩码
        else:
            self.__set_information(self.subnet, SUBR0, 4, 'subnet')     # 设置子网掩码

        if gateway is not None:
            self.__set_information(gateway, GAR0, 4, 'gateway')  # 设置网关
        else:
            self.__set_information(self.gateway, GAR0, 4, 'gateway')    # 设置网关

    def get_ip(self, str=None):
        '''
        获取网络信息
        :param str: self.local_ip / self.subnet / self.gateway
        :return: local_ip
        '''
        local_ip = None
        if str is None:
            local_ip = self.__get_information(SIPR0, 'ip 地址')   # 获取IP地址
            self.__get_information(SUBR0, '子网掩码')    # 获取子网掩码
            self.__get_information(GAR0, '网关')    # 获取网关
        elif str == self.local_ip:
            local_ip = self.__get_information(SIPR0, 'ip 地址')  # 获取IP地址
        elif str == self.subnet:
            self.__get_information(SUBR0, '子网掩码')  # 获取子网掩码
        elif str == self.gateway:
            self.__get_information(GAR0, '网关')    # 获取网关

        return local_ip # 返回 ip

    def getSN_TX_FSR(self, socket_num):
        '''
        获取发送缓冲区大小
        :param socket_num: socket 端口
        :return: 发送缓冲区空闲部分大小
        '''
        global val, val1
        val = 0
        val1 = 0
        while True:
            val1 = self.__device.IINCHIP_READ(self.SN_TX_FSR0(socket_num))
            val1 = (val1 << 8) + self.__device.IINCHIP_READ(self.SN_TX_FSR1(socket_num))
            if val1 != 0:
                val = self.__device.IINCHIP_READ(self.SN_TX_FSR0(socket_num))
                val = (val << 8) + self.__device.IINCHIP_READ(self.SN_TX_FSR1(socket_num))
            if val == val1:
                break
        return val

    def getSN_RX_RSR(self, socket_num):
        '''
        给出接收缓冲区中接收数据的大小
        :param socket_num: socket 端口
        :return: 接收缓冲区空闲部分大小
        '''
        global val, val1
        val = 0
        val1 = 0
        while True:
            val1 = self.__device.IINCHIP_READ(self.SN_RX_RSR0(socket_num))
            val1 = (val1 << 8) + self.__device.IINCHIP_READ(self.SN_RX_RSR1(socket_num))
            if val1 != 0:
                val = self.__device.IINCHIP_READ(self.SN_RX_RSR0(socket_num))
                val = (val << 8) + self.__device.IINCHIP_READ(self.SN_RX_RSR1(socket_num))
            if val == val1:
                break
        return val

    def getIINCHIP_RxMAX(self, socket_num):
        '''
        此函数用于获取要发送的最大大小
        :param socket_num:
        :return:
        '''
        return self.RSIZE[socket_num]

    def getIINCHIP_TxMAX(self, socket_num):
        '''
        此函数用于获取要接收的最大大小
        :param socket_num:
        :return:
        '''
        return self.SSIZE[socket_num]

    def socketBuf_Init(self):
        '''
        此功能根据使用的信道设置发送和接收缓冲区大小
        :return:
        '''
        ssum = 0
        rsum = 0
        for i in range(self.MAX_SOCK_NUM):
            self.__device.IINCHIP_WRITE(self.Sn_TXMEM_SIZE(i), self.txsize[i])
            self.__device.IINCHIP_WRITE(self.Sn_RXMEM_SIZE(i), self.rxsize[i])
            self.SSIZE[i] = 0
            self.RSIZE[i] = 0
            if ssum <= self.MAX_SOCK_SIZE:  self.SSIZE[i] = self.txsize[i]*1024
            if rsum <= self.MAX_SOCK_SIZE:  self.RSIZE[i] = self.rxsize[i]*1024
            ssum += self.SSIZE[i]
            rsum += self.RSIZE[i]
        if self.__is_debug:
            print(debugStr + '_debug: socketBuf Init succeed ! tx_buf & rx_buf set size:{}/{}'.
                  format(self.__device.IINCHIP_READ(self.Sn_TXMEM_SIZE(0)),
                         self.__device.IINCHIP_READ(self.Sn_RXMEM_SIZE(0))))

    def send_data_processing(self, socket_num, buf):
        '''
        此函数读取Tx写入指针寄存器，并在复制缓冲区中的数据后更新Tx写入指示器寄存器。
        用户应该先读取高位字节，然后再读取低位字节，以获得正确的值
        :param socket_num: socket number
        :param buf: data buffer to send
        :return: None
        '''
        if len(buf) == 0:
            print("CH: %d Unexpected1 length 0" % socket_num)
            return

        ptr = self.__device.IINCHIP_READ(self.SN_TX_WR0(socket_num))
        ptr = ((ptr & 0x00ff) << 8) + self.__device.IINCHIP_READ(self.SN_TX_WR1(socket_num))

        addrbsb = (ptr << 8) + (socket_num << 5) + 0x10
        self.__device.wiz_writeBuff(addrbsb, buf)

        ptr += len(buf)
        self.__device.IINCHIP_WRITE(self.SN_TX_WR0(socket_num), ((ptr & 0xff00) >> 8))
        self.__device.IINCHIP_WRITE(self.SN_TX_WR1(socket_num), (ptr & 0x00ff))

    def recv_data_processing(self, socket_num, length):
        '''
        此函数读取Rx读取指针寄存器,并且在从接收缓冲器复制数据之后更新Rx写入指针寄存器。
        用户应该先读取高位字节，然后再读取低位字节，以获得正确的值。
        :param socket_num: socket number
        :param buf: data buffer to send
        :return: 接收到的数据 bytearry
        '''
        if length == 0:
            print("CH: %d Unexpected2 length 0" % socket_num)
            return

        ptr = self.__device.IINCHIP_READ(self.SN_RX_RD0(socket_num))
        ptr = ((ptr & 0x00ff) << 8) + self.__device.IINCHIP_READ(self.SN_RX_RD1(socket_num))

        addrbsb = (ptr << 8) + (socket_num << 5) + 0x18
        buf = self.__device.wiz_readBuff(addrbsb, length)

        ptr += length
        self.__device.IINCHIP_WRITE(self.SN_RX_RD0(socket_num), ((ptr & 0xff00) >> 8))
        self.__device.IINCHIP_WRITE(self.SN_RX_RD1(socket_num), (ptr & 0x00ff))

        return buf

if __name__ == '__main__':
    # 配置信息
    config = (
        bytearray([0x00, 0x08, 0xdc, 0x11, 0x11, 0x11]),  # mac 地址
        bytearray([192, 168, 5, 5]),  # ip地址
        bytearray([255, 255, 255, 0]),  # 子网掩码
        bytearray([192, 168, 5, 1])  # 网关
    )

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