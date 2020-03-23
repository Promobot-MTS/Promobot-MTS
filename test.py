#import RobotAPI as rapi
import cv2
import Lidar

#bot = PromobotLidar("/dev/ttyUSB0")
#robot = rapi.RobotAPI()
bot = Lidar.PromobotLidar("COM3")
ang = 0
pos = 0
#cv2.startWindowThread()

while 1:
    '''while 1:
        dist = bot.looksectors()
        if dist == None:
            bot.restart()
        else:
            break'''
    # print(dist)
    # if min(dist) == dist[0]:
    #     robot.serv(-60)
    # elif min(dist) == dist[2]:
    #     robot.serv(60)
    # else:
    #     robot.serv(0)

    frame = bot.startdrawmap(640, 480)
    cv2.imshow("frame", frame)
    cv2.waitKey(0)
    #robot.set_frame(fame, 30)