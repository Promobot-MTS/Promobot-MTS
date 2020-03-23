import RobotAPI as rapi
import rplidar as rp
import cv2
import numpy as np
import math as m

class PromobotLidar():

    img = np.zeros((600, 800, 3))
    PORT1 = "COM3"
    flagdraw = False

    def __init__(self, PORT):
        while 1:
            try:
                self.lidar = rp.RPLidar(PORT, 115200)
                self.PORT1 = PORT
                break
            except:
                pass

    def restart(self):
        self.lidar.disconnect()
        self.__init__(self.PORT1)

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


    def startdrawmap(self, width=480, height=640, sectors=[[285, 335], [335, 25], [25, 75]]):
        LeftSectBord = sectors[0]
        ForwardSectBord = sectors[1]
        RightSectBord = sectors[2]
        X = int(width/2)
        Y = int(height/2)
        frame = np.ones((width, height, 3), dtype=np.uint8)
        cv2.circle(frame, (X, Y), 3, (0,0,255),-1)
        try:
            for scan in self.lidar.iter_scans():
                try:
                    for i in range(len(scan)):
                        xo = scan[i][2] * m.sin(scan[i][1])
                        yo = scan[i][2] * m.cos(scan[i][1])
                        x = X + xo
                        y = Y + yo
                        cv2.circle(frame, (x, y), 2, (255, 0, 0), -1)
                except Exception as E:
                    print(E)
        except:
            self.lidar.stop()
            self.restart()
        return frame



bot = PromobotLidar("/dev/ttyUSB0")
robot = rapi.RobotAPI()
ang = 0
pos = 0

while 1:
    while 1:
        dist = bot.looksectors()
        if dist == None:
            bot.restart()
        else:
            break
    # print(dist)
    # if min(dist) == dist[0]:
    #     robot.serv(-60)
    # elif min(dist) == dist[2]:
    #     robot.serv(60)
    # else:
    #     robot.serv(0)

    frame = bot.startdrawmap(480, 640)
    robot.set_frame(frame, 30)


