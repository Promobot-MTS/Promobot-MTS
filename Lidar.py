import rplidar as rp
from PIL import Image as img
import cv2
import numpy as np
import time
import threading as tr

class PromobotLidar():

    LeftSectBord = [285, 335]
    ForwardSectBord = [335, 25]
    RightSectBord = [25, 75]
    img = np.zeros((600, 800, 3))

    PORT1 = "COM3"
    flagdraw = False

    def __init__(self, PORT):
        # self.lidar = rp.RPLidar(PORT, 115200)
        while 1:
            try:
                self.lidar = rp.RPLidar(PORT, 115200)
                self.PORT1 = PORT
                break
            except:
                print("\n\n    mb lidar has no connected\n    or mb wrong port\n    IDK ^_^  X_X  \\_(*_*)_/")

    def restart(self):
        self.lidar.disconnect()
        self.__init__(self.PORT1)
        pass

    def lookforward(self, interval):
        AngDist = []
        distances = []
        self.lidar.clear_input()
        for scan in self.lidar.iter_scans():
            try:
                for i in range(len(scan)):
                    if scan[i][1] >= 360-(interval/2) or scan[i][1] <= interval/2:
                        AngDist.append((scan[i][1], scan[i][2]))
                        distances.append(scan[i][2])
                minD = min(distances)
                maxD = max(distances)
                return minD, maxD, AngDist
            except Exception as E:
                print(E)


    def looksectors(self):
        LeftDistSects = []
        ForwardDistSects = []
        RightDistSects = []
        try:
            for scan in self.lidar.iter_scans():
                try:
                    for i in range(len(scan)):
                        if scan[i][1] >= self.LeftSectBord[0] and scan[i][1] <= self.LeftSectBord[1]:
                            LeftDistSects.append(scan[i][2])
                        elif scan[i][1] > self.ForwardSectBord[0] or scan[i][1] < self.ForwardSectBord[1]:
                            ForwardDistSects.append(scan[i][2])
                        elif scan[i][1] >= self.RightSectBord[0] or scan[i][1] <= self.RightSectBord[1]:
                            RightDistSects.append(scan[i][2])
                        else:
                            continue
                    return min(LeftDistSects), min(ForwardDistSects), min(RightDistSects)
                except Exception as E:
                    print(E)
        except:
            self.lidar.stop()
            self.restart()


    def startdrawmap(self, width, height):
        if self.flagdraw == False:
            self.flagdraw = True
        pass


    def stopdrawmap(self):
        self.flagdraw = False


if __name__ == '__main__':
    li = PromobotLidar("COM3")
    while 1:
        r = li.looksectors()
        if r == None:
            li.restart()
        else:
            break

    print(r)