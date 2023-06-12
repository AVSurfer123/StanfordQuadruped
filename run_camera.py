#!/usr/bin/env python3

import numpy as np
import cv2
import depthai as dai

def imshow(name, img):
    mode = 0
    if mode == 0:
        cv2.imshow(name, img)
    elif mode == 1:
        cv2.imwrite(name, img)
    else:
        return

class OakCamera:

    def __init__(self):
                
        # Create pipeline
        pipeline = dai.Pipeline()

        # Define source and output
        camRgb = pipeline.create(dai.node.ColorCamera)
        xoutVideo = pipeline.create(dai.node.XLinkOut)

        xoutVideo.setStreamName("video")

        # Properties
        camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setVideoSize(960, 540)

        xoutVideo.input.setBlocking(False)
        xoutVideo.input.setQueueSize(1)

        # Linking
        camRgb.video.link(xoutVideo.input)

        # Connect to device and start pipeline
        self.device = dai.Device(pipeline)
        self.video = self.device.getOutputQueue(name="video", maxSize=1, blocking=False)

    def get_frame(self):
        return self.video.get().getCvFrame()

    def run(self):
        while True:
            frame = self.get_frame()

            frame_blur = cv2.GaussianBlur(frame, (5, 5), 0)
            imshow("blur.png", frame_blur)
            # frame_opening =  cv2.morphologyEx(frame_blur, cv2.MORPH_OPEN, (5, 5))
            # imshow("opening.png", frame_opening)
            frame_hsv = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)
            imshow("camera_hsv.png", frame_hsv)

            mask = cv2.inRange(frame_hsv, (13, 100, 64), (35, 255, 255))
            imshow("mask.png", mask)
            filtered = cv2.bitwise_and(frame, frame, mask=mask)
            imshow("filtered.png", filtered)

            canny = cv2.Canny(mask, 100, 100)
            imshow("canny.jpg", canny)

            contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contour_img = cv2.drawContours(frame.copy(), contours, -1, (0, 255, 0), 3)
            imshow("contours.png", contour_img)

            if len(contours) == 0:
                if cv2.waitKey(1) == ord('q'):
                    break
                continue

            max_contour = max(contours, key=lambda c: cv2.contourArea(c))
            mu = cv2.moments(max_contour)
            center = (mu['m10'] / (mu['m00'] + 1e-5), mu['m01'] / (mu['m00'] + 1e-5))
            print("Max contour center:", center)
            max_contour_img = cv2.drawContours(frame.copy(), [max_contour], -1, (255, 0, 0), 3)
            cv2.circle(max_contour_img, (int(center[0]), int(center[1])), 10, (0, 255, 0), -1)
            imshow("filtered_contour.png", max_contour_img)

            img_center = np.array([960, 540]) / 2.0

            yaw_diff = (center[0] - img_center[0]) / img_center[0]
            print(yaw_diff)

            # horizontal_diff = (720 - center[1]) / 720
            # print(horizontal_diff)

            cv2.imwrite("filtered_contour.png", max_contour_img)

            if cv2.waitKey(1) == ord('q'):
                break


if __name__ == '__main__':
    cam = OakCamera()
    cam.run()


# HSV Thresholds
# H 13-34
# S 100-255
# V 64-255 