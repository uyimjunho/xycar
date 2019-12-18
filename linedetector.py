import cv2
import numpy as np

try:
    import rospy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
except:
    pass


class LineDetector:
    def __init__(self, topic, ros_node=True):
        self.ros_node = ros_node
        self.image_width = 640
        self.scan_width, self.scan_height = 600, 40  # 300
        self.area_width, self.area_height = 30, 10
        area = self.area_width * self.area_height
        self.pxl_cnt_threshold = area * 0.16
        self.linescan_offset = 15
        self.roi_vertical_pos = 200
        self.left, self.right = -1, -1
        self.color3 = (255, 255, 255);
        self.color1 = 255
        self.lmid, self.rmid = self.scan_width, self.image_width - self.scan_width
        self.row_begin = (self.scan_height - self.area_height) // 2
        self.row_end = self.row_begin + self.area_height
        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.ROI_image = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.mask = np.zeros(shape=(self.scan_height, self.image_width), dtype=np.uint8)
        self.edge = np.zeros(shape=(self.scan_height, self.image_width), dtype=np.uint8)

        if self.ros_node:
            self.bridge = CvBridge()
            rospy.Subscriber(topic, Image, self.conv_image)
            self.recorder = cv2.VideoWriter('/home/nvidia/xycar/src/auto_drive/record.avi',
                                            cv2.VideoWriter_fourcc(*'MJPG'), 30, (640, 480))

    def __del__(self):
        if self.ros_node:
            self.recorder.release()
        cv2.destroyAllWindows()

    def conv_image(self, data):
        if self.ros_node:
            self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            self.recorder.write(self.cam_img)
        else:
            self.cam_img = data
        gray = cv2.cvtColor(self.cam_img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edge = cv2.Canny(blur, 100, 150)
        mask = np.zeros_like(edge)
        if len(edge.shape) > 2:
            self.color = self.color3
        else:
            self.color = self.color1

        roi = self.cam_img[self.roi_vertical_pos:self.roi_vertical_pos + self.scan_height, :]
        self.cam_img = cv2.rectangle(self.cam_img, (0, self.roi_vertical_pos),
                                     (self.image_width - 1, self.roi_vertical_pos + self.scan_height),
                                     (255, 0, 0), 3)

        v = self.roi_vertical_pos
        roi = self.cam_img[v:v + self.scan_height, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        avg_value = np.average(hsv[:, :, 2])
        value_threshold = avg_value * 1.0

        lbound = np.array([0, 0, 200], dtype=np.uint8)
        ubound = np.array([100, 255, 255], dtype=np.uint8)
        self.edge = cv2.inRange(hsv, lbound, ubound)

    def detect_lines(self):

        self.left, self.right = -1, -1

        bin = self.edge

        self.view = cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR)

        for l in range(self.area_width + 30, self.lmid):
            area = bin[self.row_begin:self.row_end, l - self.area_width:l]
            if cv2.countNonZero(area) > self.pxl_cnt_threshold:
                self.left = l
                break

        for r in range(self.image_width - 30 - self.area_width, self.rmid, -1):
            area = bin[self.row_begin:self.row_end, r:r + self.area_width]
            if cv2.countNonZero(area) > self.pxl_cnt_threshold:
                self.right = r
                break

        if self.left != -1:
            lsquare = cv2.rectangle(self.view, (self.left - self.area_width, self.row_begin), (self.left, self.row_end),
                                    (0, 255, 0), 3)
        else:
            print("Lost left line")

        if self.right != -1:
            rsquare = cv2.rectangle(self.view, (self.right, self.row_begin),
                                    (self.right + self.area_width, self.row_end), (0, 255, 0), 3)
        else:
            print("Lost right line")
        return self.left, self.right

    def show_images(self):
        cv2.imshow("test", self.view)
        cv2.imshow("origin", self.cam_img)
        # Display images for debugging purposes;
        cv2.waitKey(1)
        pass