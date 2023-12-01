# Hand Gesture Volume Control
# What it does
# Controls the master volume of the computer
#
# How it works
# Detects the Index & Thumb fingers
# Calculates the distance between both the fingers and controls the volume accordingly
# Displays the Output in 30-40 frames per seconds
# How is it built
# Hand Detection Module : Class containing the methods to detect hands & hand's postion.
# Python's cv2 and mediapipe libraries : Provide the modules and methods to get hand landmarks and draw shapes to mark them.

"""
    Use Index & Thumb finger tips' movement to control the volume of the system
    Made with cv2, pycaw & numpy library
"""

import cv2
import numpy as np
import math
from ctypes import cast, POINTER
from comtypes import CLSCTX_ALL
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume
from ctypes import windll
"""
    Detect hands using cv2 & mediapipe library
    Draw lines to connect the landmarks of the detected hands
"""
import cv2
import mediapipe as mp

icon_path = 'cam.png'
class handDetector():
    def __init__(self, mode=False, maxHands=2, detectConfidence=0.75, trackConfidence=0.75):
        self.mode = mode
        self.maxHands = maxHands
        self.detectConfidence = detectConfidence
        self.trackConfidence = trackConfidence
        self.mpHands = mp.solutions.hands
        # process a RGB image & return the hand landmarks of detected hands
        self.hands = self.mpHands.Hands()
        self.mpDraw = mp.solutions.drawing_utils

    def findHands(self, frame, draw=True):
        # convert img from BGR to RGB for hands object to process
        rgbImg = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.result = self.hands.process(rgbImg)
        if self.result.multi_hand_landmarks:
            for handLmarks in self.result.multi_hand_landmarks:
                if draw:
                    # draw landmarks & connections for them
                    self.mpDraw.draw_landmarks(
                        frame, handLmarks, self.mpHands.HAND_CONNECTIONS)
        return frame

    def findPosition(self, frame, handNo=0, draw=True):
        self.LmarkList = []
        rgbImg = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.result = self.hands.process(rgbImg)

        if self.result.multi_hand_landmarks:
            # processed landmarks from Hands() method for 1 hand
            myHand = self.result.multi_hand_landmarks[handNo]
            # get land mark ids
            for LmarkId, lmk in enumerate(myHand.landmark):
                h, w, c = frame.shape  # get o/p window dimension
                # coordinates for landmarks of detected hands
                cx, cy = int(lmk.x * w), int(lmk.y * h)
                self.LmarkList.append([LmarkId, cx, cy])

                if draw:
                    if LmarkId in [4, 8, 12, 16, 20]:  # ids of all finger tips
                        cv2.circle(frame, (cx, cy), 15,
                                   (84, 245, 66), cv2.FILLED)
        return self.LmarkList


def main():
    video = cv2.VideoCapture(0)  # open camera to capture image frame
    detect = handDetector()

if __name__ == "__main__":
    main()

# access system's volume control
devices = AudioUtilities.GetSpeakers()
interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
volume = cast(interface, POINTER(IAudioEndpointVolume))
# volume.GetMute()
# volume.GetMasterVolumeLevel()
volRange = volume.GetVolumeRange()
volMin = volRange[0]
volMax = volRange[1]

def set_window_icon(window_name, icon_path):
    hwnd = windll.user32.FindWindowW(None, window_name)
    if hwnd != 0:
        icon = windll.user32.LoadImageW(0, icon_path, 3, 0, 0, 0x40 | 0x1)
        windll.user32.SendMessageW(hwnd, 0x80, 0, icon)

def main():
    # turn on the camera (0) 7 start capture
    video = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    # setting the captured video's dimensions
    # wCam, hCam = 1920, 1080
    # video.set(3, wCam)
    # video.set(4, hCam)

    detect = handDetector()  # make an object for hand detection module

    while True:
        check, frame = video.read()  # check & capture the frame
        # flip the frame for a mirror image like o/p
        frame = cv2.flip(frame, 1)
        # get landmarks & store in a list
        LmarkList = detect.findPosition(frame, draw=False)
        if len(LmarkList) != 0:
            print(LmarkList[4], LmarkList[8])

            # coordinates for landmarks of Thumb & Index fingers
            x1, y1 = LmarkList[4][1], LmarkList[4][2]
            x2, y2 = LmarkList[8][1], LmarkList[8][2]

            cv2.circle(frame, (x1, y1), 15, (0, 255, 0),
                       cv2.FILLED)  # draw circle @ thumb tip
            cv2.circle(frame, (x2, y2), 15, (0, 255, 0),
                       cv2.FILLED)  # draw circle @ index tip
            cv2.line(frame, (x1, y1), (x2, y2), (7, 0, 212), 3, 5)

            # midpt. for thumb-index joining line
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

            # draw circle @ midpt. of the thumb-index joing line
            cv2.circle(frame, (cx, cy), 15, (255, 255, 255), cv2.FILLED)

            # joining line length
            length = math.hypot(x2 - x1, y2 - y1)

            # setting the volume
            # hand range : 50 to 200
            # volume range : -65 to 0
            vol = np.interp(length, [50, 300], [volMin, volMax])
            print(int(length), vol)
            volume.SetMasterVolumeLevel(vol, None)

            # changing colors for respective lengths of joining line
            if length <= 50:
                cv2.circle(frame, (cx, cy), 15, (237, 250, 0),
                           cv2.FILLED)  # cyan circle

            elif length >= 200:
                cv2.circle(frame, (cx, cy), 15, (250, 0, 0),
                           cv2.FILLED)  # blue circle

        cv2.putText(frame, "Press 'q' to exit", (25, 450), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (255, 255, 0), 2)  # display quit key on o/p window
        cv2.imshow('HVC',frame)  # open window for showing the o/p

        # escape key (q)
        if cv2.waitKey(1) == ord('q'):
            break

    video.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()