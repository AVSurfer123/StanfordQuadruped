#!/usr/bin/env python3

import numpy as np
import cv2
import depthai as dai

def imshow(name, img, mode=2):
    if mode == 0:
        cv2.imshow(name, img)
    elif mode == 1:
        cv2.imwrite(name, img)
    else:
        return

class OakCamera:

    SHAPE = np.array([960, 540])

    def __init__(self):
                
        # Create pipeline
        pipeline = dai.Pipeline()

        # Define source and output
        camRgb = pipeline.create(dai.node.ColorCamera)
        # left = pipeline.create(dai.node.MonoCamera)
        # right = pipeline.create(dai.node.MonoCamera)
        # stereo = pipeline.create(dai.node.StereoDepth)
        xoutVideo = pipeline.create(dai.node.XLinkOut)

        xoutVideo.setStreamName("video")

        # Properties
        camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setVideoSize(*self.SHAPE)

        xoutVideo.input.setBlocking(False)
        xoutVideo.input.setQueueSize(1)

        # Linking
        camRgb.video.link(xoutVideo.input)

        # Connect to device and start pipeline
        self.device = dai.Device(pipeline)
        self.video = self.device.getOutputQueue(name="video", maxSize=1, blocking=False)
        self.calib = self.device.readCalibration()
        self.K_left = np.array(self.calib.getCameraIntrinsics(dai.CameraBoardSocket.LEFT, self.SHAPE[0], self.SHAPE[1]))

    def get_frame(self):
        return self.video.get().getCvFrame()
    
    def run_once(self):
        frame = self.get_frame()
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        imshow("camera_hsv.png", frame_hsv)

        mask = cv2.inRange(frame_hsv, (13, 100, 64), (35, 255, 255))
        imshow("mask.png", mask)
        filtered = cv2.bitwise_and(frame, frame, mask=mask)
        imshow("filtered.png", filtered)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour_img = cv2.drawContours(frame.copy(), contours, -1, (0, 255, 0), 3)
        imshow("contours.png", contour_img)

        if len(contours) == 0:
            return None

        max_contour = max(contours, key=lambda c: cv2.contourArea(c))
        mu = cv2.moments(max_contour)
        center = (mu['m10'] / (mu['m00'] + 1e-5), mu['m01'] / (mu['m00'] + 1e-5))
        print("Max contour center:", center)
        max_contour_img = cv2.drawContours(frame.copy(), [max_contour], -1, (255, 0, 0), 3)
        cv2.circle(max_contour_img, (int(center[0]), int(center[1])), 10, (0, 255, 0), -1)
        imshow("filtered_contour.png", max_contour_img)

        img_center = self.SHAPE / 2.0
        yaw_diff, pitch_diff = (center - img_center) / img_center
        print(yaw_diff, -pitch_diff)

        imshow("filtered_contour.png", max_contour_img, 0)
        return yaw_diff, -pitch_diff

    def run(self):
        while True:
            self.run_once()
            if cv2.waitKey(1) == ord('q'):
                break


if __name__ == '__main__':
    cam = OakCamera()
    cam.run()


# HSV Thresholds
# H 13-34
# S 100-255
# V 64-255 