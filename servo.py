# Untitled - By: Devin - 周三 1月 15 2020

import sensor, image, time, math
from pyb import Servo

green_threshold   = ( 85, 90,-75,-55, 5, 35)
s1 = Servo(1) #aileron 1
s2 = Servo(2) #aileron 2
s3 = Servo(3) #rudder

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.skip_frames(time = 10)
clock = time.clock()

K = 50 #the value should be measured
X = 160 #total pixels in x
Y = 120 #total pixels in y
KP = 1 #the value should be measured

while(True):
    clock.tick()

    img = sensor.snapshot()

    blobs = img.find_blobs([green_threshold])

    if blobs:
        for b in blobs:
            ROI = (b[0],b[1],b[2],b[3])
            img.draw_rectangle(b[0:4]) # rect
            img.draw_cross(b[5], b[6]) # cx, cy
            x_pix = b[5] - X/2
            x_angle = arctan(K * x_pix)
            y_pix = b[6] - Y/2
            y_angle = arctan(K * y_pix)

            s1.angle(KP * y_angle)
            s2.angle(- KP * y_angle)
            s3.angle(KP * x_angle)

    print(clock.fps())

