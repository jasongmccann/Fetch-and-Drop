# time: 11/8/2021 12:14 PM
from naoqi import ALProxy
import time

IP1 = "10.0.126.144"
IP2 = "10.0.126.29"
'''
tts = ALProxy("ALTextToSpeech", IP1, 9559)
tts.say("hello")


markProxy = ALProxy("ALLandMarkDetection", IP2, 9559)
period = 500
markProxy.subscribe("Test_Mark", period, 0.0)
# Create a proxy to ALMemory.
time.sleep(10.0)


memProxy = ALProxy("ALMemory", IP2, 9559)
# Get data from landmark detection (assuming face detection has been activated).
data = memProxy.getData("LandmarkDetected")
print (data)
'''
# Replace this with your robot's IP address
IP = "10.0.126.29"
PORT = 9559

# Create a proxy to ALLandMarkDetection
try:
    landMarkProxy = ALProxy("ALLandMarkDetection", IP2, PORT)
except Exception, e:
    print "Error when creating landmark detection proxy:"
    print str(e)
    exit(1)


period = 500
landMarkProxy.subscribe("Test_LandMark", period, 0.0)

memValue = "LandmarkDetected"


time.sleep(5.0)

try:
    memoryProxy = ALProxy("ALMemory", IP2, PORT)
except Exception, e:
    print "Error when creating memory proxy:"
    print str(e)
    exit(1)

print "Creating landmark detection proxy"


for i in range(0, 20):
    time.sleep(0.5)
    val = memoryProxy.getData(memValue, 0)
    print ""
    print "\*****"
    print ""

if val and isinstance(val, list) and len(val) == 2:

    timeStamp = val[0]

    markInfoArray = val[1]

    try:
        for markInfo in markInfoArray:

            markShapeInfo = markInfo[0]

            markExtraInfo = markInfo[1]

            print "mark  ID: %d" % (markExtraInfo[0])
            print "  alpha %.3f - beta %.3f" % (markShapeInfo[1], markShapeInfo[2])
            print "  width %.3f - height %.3f" % (markShapeInfo[3], markShapeInfo[4])
    except Exception, e:
        print "Naomarks detected, but it seems getData is invalid. ALValue ="
        print val
        print "Error msg %s" % (str(e))
else:
    print "Error with getData. ALValue = %s" % (str(val))


landMarkProxy.unsubscribe("Test_LandMark")
print "Test terminated successfully."
