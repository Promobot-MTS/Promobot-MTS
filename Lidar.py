import rplidar as rp
import cv2
import numpy as np
import math as m
import threading
import time

class PromobotLidar():

    PORT = "COM3"

    def __init__(self, PORT):
        while 1:
            try:
                self.lidar = rp.RPLidar(PORT, 115200)
                self.PORT = PORT
                break
            except:
                pass

    def restart(self):
        self.lidar.disconnect()
        self.__init__(self.PORT)

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


    def looksectors(self, LeftSectBord=[285, 335], ForwardSectBord=[335, 25], RightSectBord = [25, 75]):
        LeftDistSects = []
        ForwardDistSects = []
        RightDistSects = []
        try:
            for scan in self.lidar.iter_scans():
                try:
                    for i in range(len(scan)):
                        if scan[i][1] >= LeftSectBord[0] and scan[i][1] <= LeftSectBord[1]:
                            LeftDistSects.append(scan[i][2])
                        elif scan[i][1] > ForwardSectBord[0] or scan[i][1] < ForwardSectBord[1]:
                            ForwardDistSects.append(scan[i][2])
                        elif scan[i][1] >= RightSectBord[0] or scan[i][1] <= RightSectBord[1]:
                            RightDistSects.append(scan[i][2])
                        else:
                            continue
                    return min(LeftDistSects), min(ForwardDistSects), min(RightDistSects)
                except Exception as E:
                    print(E)
        except:
            self.lidar.stop()
            self.restart()


    def startdrawmap(self, width=640, height=480, sectors=[[285, 335], [335, 25], [25, 75]]):
        LeftSectBord = sectors[0]
        ForwardSectBord = sectors[1]
        RightSectBord = sectors[2]
        X = int(width/2)
        Y = int(height/2)
        frame = np.ones((height, width, 3), dtype=np.uint8)
        cv2.circle(frame, (0, 0), 3, (0, 255, 0), -1)
        # cv2.circle(frame, (width, 0), 3, (0, 255, 0), -1)
        # cv2.circle(frame, (0, height), 3, (0, 255, 0), -1)
        # cv2.circle(frame, (width, height), 3, (0, 255, 0), -1)
        cv2.circle(frame, (X, Y), 3, (0, 0, 255), -1)
        try:
            for scan in self.lidar.iter_scans():
                try:
                    for i in range(len(scan)):
                        xo = scan[i][2] * m.sin(scan[i][1])
                        yo = scan[i][2] * m.cos(scan[i][1])
                        x = int(X + xo)
                        y = int(Y + yo)
                        cv2.circle(frame, (x, y), 2, (0, 255, 0), -1)
                    return frame
                except Exception as E:
                    print(E)
        except Exception as e:
            print(e)
            self.lidar.stop()
            self.restart()