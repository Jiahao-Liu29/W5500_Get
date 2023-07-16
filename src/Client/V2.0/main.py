from myWiznet5k_spi import *
from myWiznet5k import *
from myWiznet5k_socket import mySocket
from fpioa_manager import fm
from Maix import GPIO
import time
import sensor, image
import lcd

def lcd_show_info(object, remote_ip, remote_port, string='Client'):
    '''
    lcd 显示字符信息
    '''
    strRemote_ip = str(remote_ip[0])+'.'+str(remote_ip[1])+'.'+str(remote_ip[2])+'.'+str(remote_ip[3])

    img = image.Image(size=(320, 240))
    img.draw_string(0, 10, 'ip: ' + object.getIp, lcd.WHITE, scale=3)
    img.draw_string(0, 50, 'net: ' + object.getSubnet, lcd.WHITE, scale=3)
    img.draw_string(0, 90, 'gate: ' + object.getGateway, lcd.WHITE, scale=3)
    img.draw_string(40, 130, 'W5500 is ' + string, lcd.GREEN, scale=3)
    img.draw_string(30, 170, strRemote_ip + ':' + str(remote_port), lcd.GREEN, scale=3)
    lcd.display(img)

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

    # 初始化lcd
    lcd.init()

    time.sleep_ms(1000)
    # w5500 初始化
    w5500 = myWiznet5k(spi, config=config, reset_pin=reset, is_debug=True)

    # 初始化 socket
    socket = mySocket(spi, w5500, local_port=5000, img_pieces=2040)

    print(debugStr + ' W5500 为客户端')
    print(debugStr + "连接到服务端 %d.%d.%d.%d:%d" % (remote_ip[0], remote_ip[1], remote_ip[2],
                                          remote_ip[3], remote_port))

    # lcd 显示信息
    lcd_show_info(w5500, remote_ip, remote_port, string='Client')

    # 摄像头初始化
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=2000)

    lcd.clear() # lcd清屏

    clock = time.clock()

    while True:
        SOCK_STATUS = socket.do_tcp_client(remote_ip, remote_port)
        if SOCK_STATUS == SOCK_ESTABLISHED:
            img, deltime, fps = socket.image_send()
            # 添加字符信息
            img.draw_string(0, 10, 'deltime: ' + str(deltime) + 'ms ' + 'FPS: ' + '{:.2f}'.format(fps),
                            lcd.RED, scale=1)
            # lcd 显示
            lcd.display(img)

