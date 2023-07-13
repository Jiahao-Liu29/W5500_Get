from myWiznet5k_spi import *
from myWiznet5k import *
from myWiznet5k_socket import mySocket
from fpioa_manager import fm
from Maix import GPIO
import time
import sensor, image

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

