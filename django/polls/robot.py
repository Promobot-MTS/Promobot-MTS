import cv2
import threading
import time

class RobotWork(object):
    def __init__(self):
        self.video = cv2.VideoCapture(0)
        (self.grabbed, self.frame) = self.video.read()
        t = threading.Thread(target=self.update, args=())
        # t.daemon=True
        t.start()

    def __del__(self):
        self.video.release()

    def get_frame(self):
        # self.grabbed, self.frame = self.video.read()
        image = self.frame
        # image = cv2.Canny(image, 50, 100)
        # cv2.imshow("1", image)
        # cv2.waitKey(1)
        ret, jpeg = cv2.imencode('.jpg', image)
        return jpeg.tobytes()

    def update(self):
        while True:
            (self.grabbed, self.frame) = self.video.read()
            time.sleep(0.01)

            cv2.imshow("1", self.frame)
            cv2.waitKey(1)

