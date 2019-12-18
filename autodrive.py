#!/usr/bin/env python

import rospy, time

from linedetector import LineDetector
from motordriver import MotorDriver
from obstacledetector import ObstacleDetector

class AutoDrive:

    def __init__(self):
        self.start_time = time.time()
        self.angle = 0
        self.speed = 0
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.driver = MotorDriver('/xycar_motor_msg')
        self.obstacle_detector = ObstacleDetector('/ultrasonic')



    def trace(self):
        obs_l, obs_m, obs_r = self.obstacle_detector.get_distance()
        r, l = self.line_detector.detect_lines()
        self.line_detector.show_images()
        self.angle = self.steer(l, r, self.angle)
        self.speed = self.accelerate(self.angle, obs_m)
        self.driver.drive(self.angle + 90, self.speed + 90)

    def steer(self, l, r, angle):
        mid = (l + r) // 2
        angle = -((320 - mid) // 1.6)

        if mid < 320:
            l = -1
        elif mid > 320:
            r = -1
        else:
            angle = 0
        return angle

    def accelerate(self, angle, l, r):
        speed = 40
        if l == -1 and r == -1:
            speed = 0
        return speed

    def exit(self):
        print('finished')


if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        car.trace()
        rate.sleep()
    rospy.on_shutdown(car.exit)