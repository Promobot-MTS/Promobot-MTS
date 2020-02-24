import rplidar as rp
from PIL import Image as img
import numpy as np
import time
import threading as tr

class PromobotLidar():

    LeftSectBord = [285, 335]
    ForwardSectBord = [335, 25]
    RightSectBord = [25, 75]

    flagdraw = False

    def __init__(self, PORT):
        self.lidar = rp.RPLidar(PORT, 115200)

    def searchforward(self, interval):
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


    def startdrawmap(self, width, height):
        if self.flagdraw == False:
            self.flagdraw = True
        map = img.new('RGB', (width, height), (0, 0, 120))
        centerX = int(width/2)
        centerY = int(height/2)
        while self.flagdraw:
            map.show()

    def stopdrawmap(self):
        self.flagdraw = False


if __name__ == '__main__':
    lidar1 = PromobotLidar("COM3")
    r = lidar1.looksectors()
    print(r)