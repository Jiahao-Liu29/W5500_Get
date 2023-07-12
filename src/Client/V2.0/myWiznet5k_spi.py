'''
    此文件用于 W5500 的 spi 初始化
'''
from machine import SPI
from fpioa_manager import fm
from Maix import GPIO
import time

debugStr = ' [W5500]@winznet5k:~$ '
errorStr = ' [error]: '

class mySpi:
    def __init__(self, id, baudrate=10000000, sck=28, mosi=29, miso=30, cs=27):
        fm.register(cs, fm.fpioa.GPIOHS10, force=True)  # cs
        self.cs = GPIO(GPIO.GPIOHS10, GPIO.OUT)  # cs 脚
        self.cs.value(1)
        self.spi = SPI(id, mode=SPI.MODE_MASTER, baudrate=baudrate, polarity=0, phase=0, bits=8, firstbit=SPI.MSB,
                       sck=sck, mosi=mosi, miso=miso)

    def __systemDebugOut(self):
        '''
        debug 输出函数
        '''
        str1 = """\n\n\n
            -----------------------------------------------------------------------------
            ------------------------- MaixPy For W5500 Init -----------------------------

                 ┌─────────────────────────────────────────────────────────────┐
                 │┌───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┬───┐│
                 ││Esc│!1 │@2 │#3 │$4 │%5 │^6 │&7 │*8 │(9 │)0 │_- │+= │|\ │`~ ││
                 │├───┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴───┤│
                 ││ Tab │ Q │ W │ E │ R │ T │ Y │ U │ I │ O │ P │{[ │}] │ BS  ││
                 │├─────┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴┬──┴─────┤│"""
        str2 = """                 ││ Ctrl │ A │ S │ D │ F │ G │ H │ J │ K │ L │: ;│" '│ Enter  ││
                 │├──────┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴─┬─┴────┬───┤│
                 ││ Shift  │ Z │ X │ C │ V │ B │ N │ M │< ,│> .│? /│Shift │Fn ││
                 │└─────┬──┴┬──┴──┬┴───┴───┴───┴───┴───┴──┬┴───┴┬──┴┬─────┴───┘│
                 │      │Fn │ Alt │         Space         │ Alt │Win│   HHKB   │
                 │      └───┴─────┴───────────────────────┴─────┴───┘          │
                 └─────────────────────────────────────────────────────────────┘

            -----------------------------------------------------------------------------
            -------- Prower By: Liu Ziqian  ------------- Using K210 for W5500 ----------
            -----------------------------------------------------------------------------"""
        print(str1)
        time.sleep_ms(500)
        print(str2)
        print("\n\n")

    def wiz_writeBuff(self, addr, buf):
        '''
        spi 发送 buf
        :param addr: 发送地址
        :param buf: 发送数据
        :return: None
        '''
        assert len(buf) != 0, errorStr + 'spi write buf length is null'
        self.cs.value(0)
        self.spi.write((addr & 0x00FF0000) >> 16)
        self.spi.write((addr & 0x0000FF00) >> 8)
        self.spi.write((addr & 0x000000F8) + 4)
        self.spi.write(buf)
        self.cs.value(1)

    def wiz_readBuff(self, addr, length):
        '''
        从 addr 中读取 length 个数据
        :param addr: 目的地址
        :param length: 读取数据的长度
        :return: bytearray 数组
        '''
        assert length != 0, errorStr + 'spi read buf length must not null'
        dataBuff = bytearray(length)
        self.cs.value(0)
        self.spi.write((addr & 0x00FF0000) >> 16)
        self.spi.write((addr & 0x0000FF00) >> 8)
        self.spi.write(addr & 0x000000F8)
        self.spi.readinto(dataBuff, write=0x00)
        self.cs.value(1)

        return dataBuff

    def IINCHIP_WRITE(self, addr, data):
        '''
        写一个字节的数据
        :param addr: 目的地址
        :param data: 一字节数据
        :return: None
        '''
        self.wiz_writeBuff(addr, bytearray([data]))

    def IINCHIP_READ(self, addr):
        '''
        读一个字节数据
        :param addr: 目的地址
        :return: 一个字节数据
        '''
        return self.wiz_readBuff(addr, 1)[0]