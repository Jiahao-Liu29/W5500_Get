from machine import SPI
from fpioa_manager import fm
from Maix import GPIO
import time

''' SPI 类 '''
class spiChat():
    def __init__(self, baudrate=10000000):
        fm.register(27, fm.fpioa.GPIOHS10, force=True)  # cs
        self.cs = GPIO(GPIO.GPIOHS10, GPIO.OUT)    # cs 脚
        self.spi = SPI(SPI.SPI1, mode=SPI.MODE_MASTER, baudrate=baudrate, polarity=0, phase=0, bits=8, firstbit=SPI.MSB, sck=28, mosi=29, miso=30)

    def spiSendByte(self, sbyte):
        ''' 读写一字节 '''
        readBuff = bytearray(1)
        self.spi.write_readinto(sbyte, readBuff)   # 全双工

        return readBuff



class w5500(spiChat):
    def __init__(self):
        super().__init__()
        fm.register(31, fm.fpioa.GPIOHS11, force=True)  # reset
        self.reset = GPIO(GPIO.GPIOHS11, GPIO.OUT)  # reset 脚

        self.SHAR0 = 0x000900
        self.SUBR0 = 0x000500
        self.SIPR0 = 0x000F00
        self.GAR0  = 0x000100

    def reset_w5500(self):
        ''' 复位操作 '''
        self.reset.value(0)
        time.sleep_us(2)
        self.reset.value(1)
        time.sleep_ms(1600)

    def __wiz_writeBuff(self, addr, buf):
        '''
        写数据
        :param addr:
        :param buf:
        :return:
        '''
        if len == 0:    print("write datalen Error!")
        self.cs.value(0)
        self.spi.write((addr & 0x00FF0000) >> 16)
        self.spi.write((addr & 0x0000FF00) >> 8)
        self.spi.write((addr & 0x000000F8) + 4)
        self.spi.write(buf)
        self.cs.value(1)

    def __wiz_readBuff(self, addr, len):
        '''
        读数据
        :param addr:
        :param len:
        :return: bytearray 数组
        '''
        dataBuff = bytearray(len)
        if len == 0:    print("read datalen Error!")
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
        print("W5500 IP地址： %d.%d.%d.%d\n" % (local_ip[0], local_ip[1], local_ip[2], local_ip[3]))

    def __getSUBR(self):
        '''
        读取 SUBR 寄存器
        :return:
        '''
        subnet = self.__wiz_readBuff(self.SUBR0, 4)
        print("W5500 子网掩码： %d.%d.%d.%d\n" % (subnet[0], subnet[1], subnet[2], subnet[3]))

    def __getGAR(self):
        '''
        读取 GAR 寄存器
        :return:
        '''
        gateway = self.__wiz_readBuff(self.GAR0, 4)
        print("W5500 网关： %d.%d.%d.%d\n" % (gateway[0], gateway[1], gateway[2], gateway[3]))

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


if __name__ == '__main__':

    myW5500 = w5500()
    myW5500.reset_w5500()
    myW5500.W5500_setMac(bytearray([0x00,0x08,0xdc,0x11,0x11,0x11]))
    myW5500.W5500_setIP(bytearray([255,255,255,0]), bytearray([192,168,1,1]), bytearray([192,168,1,88]))
    myW5500.W5500_getIP()


