#!/usr/bin/env python3
"""
Camera module helper functions.
"""

import cv2
import time
from threading import Thread

class FPS:
    def __init__(self):
        self._start = None
        self._end = None
        self._lastUpdate = None
        self._lastElasped = None
        self._numFrames = 0

    def start(self):
        self._start = time.time()
        self._lastUpdate = self._start

    def stop(self):
        self._end = time.time()

    def update(self):
        self._numFrames += 1
        self._lastElapsed = time.time() - self._lastUpdate
        self._lastUpdate += self._lastElapsed

    def currentFps(self):
        return 1.0 / self._lastElapsed

    def elapsed(self):
        if self._end is None:
            self.stop()
        return self._end - self._start

    def frames(self):
        return self._numFrames

    def fps(self):
        return self._numFrames / self.elapsed()

    def wait(self, millis):
        time.sleep((millis - self._lastElapsed)*0.001)

class USBVideoStream:
    def __init__(self, src=0):
        # initialize the video camera stream and read the first frame
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        print("Framerate", self.stream.get(cv2.CAP_PROP_FPS))
        self.grabbed, self.frame = self.stream.read()
        # initialize the indicate value
        self.stopped = False
        self._update_flag = False

    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while True:
            if self.stopped:
                return
            # grab frame from stream
            self.grabbed, self.frame = self.stream.read()
            self._update_flag = True
    
    def read(self):
        while not self._update_flag:
            pass
        
        self._update_flag = False
        return self.grabbed, self.frame
    
    def stop(self):
        self.stopped = True
        self.stream.release()

