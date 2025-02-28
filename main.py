import sys
import serial
import cv2
import time
from struct import *
import TinyFrame as TF
from maix import camera, display, image, nn, app,pinmap,touchscreen
import configparser

# 创建配置解析器
config = configparser.ConfigParser()

config.read('config.ini')
exposure = int(config.get('Settings', 'exposure'))
gain = int(config.get('Settings', 'gain'))

detector = nn.YOLOv5(model="/root/models/maixhub/172648/model_172648.mud")
cam = camera.Camera(detector.input_width(), detector.input_height(), detector.input_format())
cam.exposure(exposure)  #曝光时间
cam.gain(gain)       #增益
cam.luma(50)
cam.constrast(50)
cam.saturation(20)

dis = display.Display()
img_back = image.load("/maixapp/share/icon/ret.png")
back_rect = [0, 0, 32, 32]
back_rect_disp = image.resize_map_pos(cam.width(), cam.height(), dis.width(), dis.height(), image.Fit.FIT_CONTAIN, back_rect[0], back_rect[1], back_rect[2], back_rect[3])
ts = touchscreen.TouchScreen()

def is_in_button(x, y, btn_pos):
    return x > btn_pos[0] and x < btn_pos[0] + btn_pos[2] and y > btn_pos[1] and y < btn_pos[1] + btn_pos[3]

def init_uart():
    pinmap.set_pin_function("A18", "UART1_RX")
    pinmap.set_pin_function("A19", "UART1_TX")

    device = "/dev/ttyS1"
    serial0 = serial.Serial(device, 115200)
    return serial0

def adjust(x,y):
    global exposure,gain
    if 0<exposure<40000:
        if 110<x<210 and 305<y<360:
            exposure = exposure - 1000
            gain = gain - 100
            cam.exposure(exposure)  #曝光时间
            cam.gain(gain)       #增益
        if 385<x<470 and 305<y<360:
            exposure = exposure + 1000
            gain = gain + 100
            cam.exposure(exposure)  #曝光时间
            cam.gain(gain)       #增益
        config.set('Settings', 'exposure', str(exposure))
        config.set('Settings', 'gain', str(gain))
    if 0<exposure<40000:
        with open('config.ini', 'w') as config_file:
            config.write(config_file)


class Comm:
    def __init__(self, uart):
        self.uart = uart
        self.tf = TF.TinyFrame()
        self.tf.ID_BYTES =1
        self.tf.LEN_BYTES = 1
        self.tf.TYPE_BYTES = 1
        self.tf.CKSUM_TYPE = 'crc16'
        self.tf.SOF_BYTE = 0x55
        self.tf.write = self.uart.write

    def send_detect_result(self,objs):
        global lt
        i = 0
        nx,ny,nw,nh=0,0,0,0,
        has = False
        for obj in objs:
            if obj.class_id == 1 or obj.class_id == 2:
                nx,ny,nw,nh=obj.x, obj.y, obj.w, obj.h
                self.tf.send(1, pack('<HHHHH', obj.class_id,obj.x, obj.y, obj.w, obj.h), i)
                has = True
                lt = time.time()
                i=i+1
        for obj in objs:
            if has == True or time.time()-lt<0.5:
                if obj.class_id == 0 or obj.class_id == 3 or obj.class_id == 4 or obj.class_id == 5:
                    if nx<obj.x+obj.w//2 < nx+nw and ny<obj.y+obj.h//2 < ny+nh:
                        pass
                    else:
                        if obj.score >0.75:
                            idx = obj.class_id
                            self.tf.send(1, pack('<HHHHH',idx,obj.x, obj.y, obj.w, obj.h), i)
                            i=i+1
            if has == False and time.time()-lt>0.5:
                if obj.class_id == 0 or obj.class_id == 3 or obj.class_id == 4 or obj.class_id == 5:
                    if obj.score >0.75:
                        idx = obj.class_id
                        self.tf.send(1, pack('<HHHHH',idx,obj.x, obj.y, obj.w, obj.h), i)
                        i=i+1
            
lt = time.time()
uart = init_uart()
comm = Comm(uart)
last_send_time = time.time()
s_t = time.time()
last_x,last_y = 0,0
while not app.need_exit():
    try:
        x,y = 0,0
        img = cam.read()
        objs = detector.detect(img, conf_th = 0.55, iou_th = 0.6)
        FPS = "FPS:"+str(int(1/(time.time()-s_t)))
        s_t = time.time()
        img.draw_string(220, 10, FPS, color = image.COLOR_GREEN,scale=1.7)
        elapsed_time = time.time() - last_send_time
        if elapsed_time >= 0.020:
            comm.send_detect_result(objs)
            last_send_time = time.time()
        for obj in objs:
            img.draw_rect(obj.x, obj.y, obj.w, obj.h, color = image.COLOR_RED)
            msg = f'{detector.labels[obj.class_id]}: {obj.score:.2f}'
            img.draw_string(obj.x, obj.y, msg, color = image.COLOR_RED)

        img.draw_image(0, 0, img_back)
        img.draw_rect(0,270, 75, 50, color = image.COLOR_RED,thickness=3)
        img.draw_rect(245, 270, 75,50, color = image.COLOR_BLUE,thickness=3)
        img.draw_string(22,282, '-', color = image.COLOR_GREEN,scale=2.5,thickness=2)
        img.draw_string(268, 282, '+', color = image.COLOR_GREEN,scale=2.5,thickness=2)
        dis.show(img)
        x, y, preesed = ts.read()
        if is_in_button(x, y, back_rect_disp):
            app.set_exit_flag(True)
        if last_x != x and last_y != y:
            adjust(x,y)
            time.sleep(0.01)
        last_x,last_y = x,y
    except:
        time.sleep(0.001)
    

