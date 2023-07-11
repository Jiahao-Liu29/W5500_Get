import myWiznet5k_spi as wiznet5k_spi
import myWiznet5k as wiznet5k
from machine import SPI
from fpioa_manager import fm
from Maix import GPIO
import time

# 配置信息
config = (
    bytearray([0x00, 0x08, 0xdc, 0x11, 0x11, 0x11]),  # mac 地址
    bytearray([255, 255, 255, 0]),  # 子网掩码
    bytearray([192, 168, 5, 1]),  # 网关
    bytearray([192, 168, 5, 5])  # ip地址
)

# reset 脚
fm.register(31, fm.fpioa.GPIOHS11, force=True)  # reset
reset = GPIO(GPIO.GPIOHS11, GPIO.OUT)  # reset 脚
reset.value(1)

# 初始化 mySpi 类
spi = wiznet5k_spi.mySpi(SPI.SPI1)

# 延时，等待稳定
for i in range(1, 11):
    print(wiznet5k_spi.debugStr + 'wait for start ....... have {} second!'.format(10-i))
    time.sleep_ms(1000)

# w5500 初始化
w5500 = wiznet5k.myWinznet5k(spi, config=config, reset_pin=reset, is_debug=True)
w5500.init_function(True)

while True:
    pass

