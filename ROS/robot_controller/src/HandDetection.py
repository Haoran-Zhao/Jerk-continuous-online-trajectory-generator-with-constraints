#!/usr/bin/env python3
import rospy
import roslib
import cv2, cv_bridge
import mediapipe as mp
import numpy as np
from scipy.spatial import ConvexHull
from scipy.ndimage.interpolation import rotate
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image


class Detection:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw',
                                      Image, self.image_callback)
        self.pub = rospy.Publisher("HandPosition", Point, queue_size = 1)

    def minimum_bounding_rectangle(self, points):
        """
        https://stackoverflow.com/questions/13542855/algorithm-to-find-the-minimum-area-rectangle-for-given-points-in-order-to-comput
        Find the smallest bounding rectangle for a set of points.
        Returns a set of points representing the corners of the bounding box.

        :param points: an nx2 matrix of coordinates
        :rval: an nx2 matrix of coordinates
        """
        pi2 = np.pi/2.
        # get the convex hull for the points
        hull_points = points[ConvexHull(points).vertices]

        # calculate edge angles
        edges = np.zeros((len(hull_points)-1, 2))
        edges = hull_points[1:] - hull_points[:-1]

        angles = np.zeros((len(edges)))
        angles = np.arctan2(edges[:, 1], edges[:, 0])

        angles = np.abs(np.mod(angles, pi2))
        angles = np.unique(angles)

        # find rotation matrices
        # XXX both work
        rotations = np.vstack([
            np.cos(angles),
            np.cos(angles-pi2),
            np.cos(angles+pi2),
            np.cos(angles)]).T
        #     rotations = np.vstack([
        #         np.cos(angles),
        #         -np.sin(angles),
        #         np.sin(angles),
        #         np.cos(angles)]).T
        rotations = rotations.reshape((-1, 2, 2))

        # apply rotations to the hull
        rot_points = np.dot(rotations, hull_points.T)

        # find the bounding points
        min_x = np.nanmin(rot_points[:, 0], axis=1)
        max_x = np.nanmax(rot_points[:, 0], axis=1)
        min_y = np.nanmin(rot_points[:, 1], axis=1)
        max_y = np.nanmax(rot_points[:, 1], axis=1)

        # find the box with the best area
        areas = (max_x - min_x) * (max_y - min_y)
        best_idx = np.argmin(areas)
        area = areas[best_idx]
        # return the best box
        x1 = max_x[best_idx]
        x2 = min_x[best_idx]
        y1 = max_y[best_idx]
        y2 = min_y[best_idx]
        r = rotations[best_idx]

        rval = np.zeros((4, 2))
        rval[0] = np.dot([x1, y2], r)
        rval[1] = np.dot([x2, y2], r)
        rval[2] = np.dot([x2, y1], r)
        rval[3] = np.dot([x1, y1], r)

        return rval, area

    def handdetection(self, image):
        # For webcam input:
        mp_drawing = mp.solutions.drawing_utils
        mp_hands = mp.solutions.hands
        with mp_hands.Hands(
        static_image_mode=True,
        max_num_hands=1,
        min_detection_confidence=0.5) as hands:
        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
            image = cv2.flip(image, 1)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = hands.process(image)

            # Draw the hand annotations on the image.
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            h, w, c = image.shape
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    points = np.array([[res.x, res.y] for res in hand_landmarks.landmark])
                    rect, area = self.minimum_bounding_rectangle(points)
                    center = np.sum(points, axis=0)/21

                    mp_drawing.draw_landmarks(
                        image,
                        hand_landmarks,
                        mp_hands.HAND_CONNECTIONS)
                    cv2.drawMarker(image, (int(center[0]*w), int(center[1]*h)), color=(52, 91, 235), markerType=cv2.MARKER_CROSS, thickness=2)
                    self.x = center[0]*w
                    self.y = center[1]*h
                    for i in range(0, rect.shape[0]):
                        cv2.line(image, (int(rect[i-1][0]*w), int(rect[i-1][1]*h)), (int(rect[i][0]*w), int(rect[i][1]*h)), color=(235, 52, 225), thickness=2)
            p = Point()
            p.x = self.x
            p.y = self.y
            self.pub.publish(p)
            # Flip the image horizontally for a selfie-view display.
            cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))
            cv2.waitKey(1)

    def image_callback(self, msg):
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        self.handdetection(image)

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    rospy.init_node("handdetection")
    detection = Detection()
    rospy.spin()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
