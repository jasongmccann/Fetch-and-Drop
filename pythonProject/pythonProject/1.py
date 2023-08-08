from naoqi import ALProxy
import time

IP = "10.0.126.29"
PORT = 9559

redBallProxy = ALProxy("ALRedBallDetection", IP, PORT)
camProxy = ALProxy("ALVideoDevice", IP, PORT)
memoryProxy = ALProxy("ALMemory", IP, PORT)
camProxy.setActiveCamera(1)
period = 500
redBallProxy.subscribe("Test_RedBall", period, 0.0)
memValue = "redBallDetected"
time.sleep(0.5)
val = memoryProxy.getData(memValue)
if val and isinstance(val, list) and len(val) >= 2:
    timeStamp = val[0]
    ballInfo = val[1]
    try:
        print "centerX=", ballInfo[0], "centerY=", ballInfo[1]
        print "sizeX=", ballInfo[2], "sizeY=", ballInfo[3]
    except Exception, e:
        print "ALValue=", val, "error msg ", (str(e))
else:
    print "Error"
redBallProxy.unsubscribe("Test_RedBall")
