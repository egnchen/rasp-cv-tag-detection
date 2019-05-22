from camera_helper import USBVideoStream, FPS

fps_counter = FPS()
stream = USBVideoStream().start()

fps_counter.start()
while (fps_counter.frames() < 500):
    stream.read()
    fps_counter.update()
    print(fps_counter.currentFps())

fps_counter.stop()
stream.stop()
print("elapsed time", fps_counter.elapsed())
print("average fps", fps_counter.fps())
