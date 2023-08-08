from naoqi import ALProxy
import math
import almath
import time

IP = "10.0.126.29"

landmarkTheoreticalSize = 0.06
currentCamera = "CameraTop"
memoryProxy = ALProxy("ALMemory", IP, 9559)
landmarkProxy = ALProxy("ALLandMarkDetection", IP, 9559)
landmarkProxy.subscribe("landmarkTest")
markData = memoryProxy.getData("LandmarkDetected")
motion = ALProxy("ALMotion", IP, 9559)


while 1:
    while markData is None or len(markData) == 0:
        markData = memoryProxy.getData("LandmarkDetected")
    wzCamera = markData[1][0][0][1]
    wyCamera = markData[1][0][0][2]
    angularSize = markData[1][0][0][3]
    markID = markData[1][0][1][0]
    distanceFromCameraToLandmark = landmarkTheoreticalSize / (2 * math.tan(angularSize / 2))
    transform = motion.getTransform(currentCamera, 2, True)
    transformList = almath.vectorFloat(transform)
    robotToCamera = almath.Transform(transformList)
    cameraToLandmarkRotationTransform = almath.Transform_from3DRotation(0, wyCamera, wzCamera)
    cameraToLandmarkTranslationTransform = almath.Transform(distanceFromCameraToLandmark, 0, 0)
    robotToLandmark = robotToCamera * cameraToLandmarkTranslationTransform * cameraToLandmarkTranslationTransform
    x = robotToLandmark.r1_c4
    y = robotToLandmark.r2_c4
    z = robotToLandmark.r3_c4
    print markID
    print "x " + str(robotToLandmark.r1_c4) + "(in meters)"
    print "y " + str(robotToLandmark.r2_c4) + "(in meters)"
    print "z " + str(robotToLandmark.r3_c4) + "(in meters)"
    # landmarkProxy.unsubscribe("landMarkTest")

    if markID == 119:
        print "I See Mark 119, I will move to it"
        tts.say("I See Mark 119, I will move to it ")
        motion.post.moveTo(x, y, 0)
        time.sleep(10.0)
        tts.say("Arriver")
    else:
        print "I See Mark " + str(markID) + ", I only flow Mark 119 "
        tts.say("I See Mark " + str(markID) + ", I only flow Mark 119 ")
        time.sleep(5.0)
    markData = None
