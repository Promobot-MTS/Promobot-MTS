import rplidar as rp
import cv2
import numpy as np

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


    def startdrawmap(self, width=480, height=640):
        frame = np.ones((width, height, 3), dtype=np.uint8)
        cv2.circle(frame, (100,100), 3, (0,0,255))
        return frame



if __name__ == '__main__':
    li = PromobotLidar("COM3")
    while 1:
        r = li.looksectors()
        if r == None:
            li.restart()
        else:
            break

    print(r)
