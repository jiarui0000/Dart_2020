# missile - By: Jiarui - 周三 1月 15 2020

import sensor, image, time, math, imu
from pyb import Servo

attitude_origional, acc_origional = ((), ())
time_shut_down = 7 #the time to shut down under no condition (in unit of s)
green_threshold   = ( 85, 90,-75,-55, 5, 35)
s1 = Servo(1) #aileron 1
s2 = Servo(2) #aileron 2
s3 = Servo(3) #aileron 3
K = 50 #the value should be measured
X = 160 #total pixels in x
Y = 120 #total pixels in y
KP = 1 #the value should be measured
IMU = MPU6050


def initialize_sensor():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time = 100)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    return


def readin_imu():
    acceleration, attitude = IMU.sensors()
    acceleration_overall = 0.0
    for acc in acceleration:
        acceleration_overall += acc * acc
    acceleration_overall = acceleration_overall ** 0.5
    return gyro, acceleration_overall


def initialize_imu():
    IMU.wake()
    return


def readin_sensor():
    img = sensor.snapshot()

    blobs = img.find_blobs([green_threshold])
    area_total, x_pix, y_pix = (0.0, 0.0, 0.0)

    if blobs:
        for b in blobs:
            ROI = (b[0],b[1],b[2],b[3])
            img.draw_rectangle(b[0:4]) # rect
            img.draw_cross(b[5], b[6]) # cx, cy
            area = b[2] * b[3]
            x_pix += b[5]*area
            y_pix += b[6]*area
            area_total += area

    x_pix /= area_total
    y_pix /= area_total
    x_angle = math.atan(K * x_pix)
    y_angle = math.atan(K * y_pix)
    angle = (x_angle, y_angle)

    if len(blobs) == 1:
        b = blobs[0]
        Lm = (b[2]+b[3])/2
        distance = K/Lm

    return distance, angle


def initialize_servo():
    s1.angle(KP * y_angle)
    s2.angle(- KP * y_angle)
    s3.angle(KP * x_angle)
    return


def servo_change_camera_correction(differ, flydata)
    x_angle, y_angle = differ

##差从希望变化的角度到舵机转动角度之间的关系

    s1.angle(KP * y_angle)
    s2.angle(- KP * y_angle)
    s3.angle(KP * x_angle)
    return


def imu_stablize():
    return


def target_found():
    img = sensor.snapshot()
    blobs = img.find_blobs([green_threshold])
    green_block_found = ( len(blobs)>0 )
    return green_block_found


def imu_correction():
    flydata = readin_imu()
    servo_change_imu_stabilize(differ, flydata)
    return


def camera_correction():
    target_position = readin_sensor()
    flydata = readin_imu()
    differ = target_position[1]
    servo_change_camera_correction(differ, flydata)
    return


def main():
    initialize_imu()
    initialize_servo()

#待机状态 以每秒30帧检测加速度 判断是否起飞
    flydata = readin_imu()
    while (flydata[1] < 0.5): ## acceleration < 0.5g
        clock.tick(30)
        flydata = readin_imu()

#起飞后 未找到目标 以大约每秒10帧画面搜索目标
    initialize_sensor()
    t = time.clock()
    while (!target_found()):
        time.sleep_ms(100)

#第一次找到目标后 以每秒30帧更新目标状态并调整舵机
    while (time.clock() < time_shut_down):
        clock.tick(30)
        if (target_found()):
            camera_correction()
        else:
            imu_correction()
    initialize_servo()
    return


main()
