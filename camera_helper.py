#!/usr/bin/env python3
"""
Camera module helper functions.
"""

import cv2
import os
import time
import glob
from threading import Thread, Lock
from queue import Queue

class FPS:
    def __init__(self):
        self._start = None
        self._end = None
        self._lastUpdate = None
        self._lastElapsed = 0 
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
        time.sleep((millis - self._lastElapsed) * 0.001)


class PicVideoStream:

    def __init__(self, path, prefix, filetype="jpg", sync=False, framerate=40):
        self.path = path
        self.prefix = prefix
        self.filetype = filetype
        self._sync = sync
        self._stopped = False
        self._update_flag = False
        self.fps_counter = FPS()
        self._buffer = Queue() 
        if self._sync:
            self.wait_time = 1000 / framerate
        
    def start(self, offset=None):
        if offset == None:
            # get all filenames that meets the standard
            l = glob.glob(os.path.join(self.path, "".join((self.prefix, "*.", self.filetype))))
            # convert to numbers
            l = [int(x[len(os.path.join(self.path, self.prefix)): -1 - len(self.filetype)]) for x in l]
            # get the latest value
            self.num = max(l)
        else:
            self.num = offset
        print(f"Starting from {self.num}")
        Thread(target=self.update, args=()).start()
        return self

    def get_filename(self, offset = 0):
        return os.path.join(self.path, "".join((self.prefix, str(self.num + offset), '.', self.filetype)))

    def update(self):
        self.fps_counter.start()
        while True:
            if self._sync:
                self.fps_counter.wait(self.wait_time)
            self.fps_counter.update()
            
            # place offset=1 here
            # to avoid incomplete picture
            fn = self.get_filename(1)
            while not os.path.exists(fn):
                time.sleep(0.001)

            self._buffer.put((self.num, cv2.imread(self.get_filename())))
            self.num += 1
    
    def read(self):
        return self._buffer.get()

    def stop(self):
        self._stopped = True
        self.fps_counter.stop()

    def get_fps(self):
        return self.fps_counter.currentFps()


class USBVideoStream:
    def __init__(self, src=0, sync=False, framerate=30):
        # initialize the video camera stream and read the first frame
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        print("Framerate", self.stream.get(cv2.CAP_PROP_FPS))
        self.grabbed, self.frame = self.stream.read()
        # initialize the indicate value
        self.stopped = False
        self._update_flag = False
        self._update_lock = Lock()
        self._sync = sync
        self.fps_counter = FPS()
        if self._sync:
            # synchronized update
            self.wait_time = 1 / framerate

    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        self.fps_counter.start()

        while not self._stopped:
            if self.stopped:
                return
            if self._sync:
                self.fps_counter.wait(self.wait_time)
            self.fps_counter.update()

            # grab frame from stream
            self.grabbed, self.frame = self.stream.read()
            self._update_flag = True
    
    def read(self):
        while not self._update_flag:
            pass
        
        ret = self.grabbed, self.frame
        self._update_flag = False
        return ret

    
    def stop(self):
        self.stopped = True
        self.fps_counter.stop()
        self.stream.release()

    def get_fps(self):
        return self.fps_counter.currentFps()
