from naoqi import ALProxy

IP = "10.0.126.29"
PORT = 9559

landmark = ALProxy("ALLandMarkDetection", IP, PORT)
memory = ALProxy("ALMemory", IP, PORT)

period = 500
event = landmark.subscribe("Test_Mark", period, 0.0)

#landmark
markData = memory.getData("LandmarkDetected")
print (markData)
#
#
'''
try:
    landMarkProxy = ALProxy("ALLandMarkDetection", IP1, PORT)
except Exception, e:
    print "Error when creating landmark detection proxy:"
    print str(e)
    exit(1)


period = 500
landMarkProxy.subscribe("Test_LandMark", period, 0.0)

memValue = "LandmarkDetected"


time.sleep(5.0)

try:
    memoryProxy = ALProxy("ALMemory", IP1, PORT)
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
'''
