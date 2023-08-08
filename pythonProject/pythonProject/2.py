# encoding=utf-8

from __future__ import division
from naoqi import ALProxy
from PIL import Image
import motion
import cv2
import math
import vision_definitions
import numpy as np
import almath
import time

IP = "10.0.126.29"  # ROBOT IP
PORT = 9559
motionProxy = ALProxy("ALMotion", IP, PORT)  # motion
names = list()
times = list()
keys = list()
x = 0
y = 0
flag = None


def getHeadAngle(IP, PORT):
    # motionProxy = ALProxy("ALMotion",IP,PORT)
    actuator = ["HeadYaw", "HeadPitch"]  # HeadYaw HeadPitch
    useSensor = False
    headAngle = motionProxy.getAngles(actuator, useSensor)
    return headAngle


def getHeadPitchAngle(IP, PORT):
    # motionProxy = ALProxy("ALMotion",IP,PORT)
    actuator = "HeadPitch"
    useSensor = False
    headAngle = motionProxy.getAngles(actuator, useSensor)
    return headAngle


def setHeadAngle(alpha, beta):
    # motionProxy = ALProxy("ALMotion", IP, PORT)
    motionProxy.setStiffnesses("Head", 1.0)
    maxSpeedFraction = 0.3
    names = ["HeadYaw", "HeadPitch"]
    angles = [alpha, beta]
    motionProxy.angleInterpolationWithSpeed(names, angles, maxSpeedFraction)

    motionProxy.setStiffnesses("Head", 0.0)


def getImage(IP, PORT, cameraID):  # cameraID=1
    camProxy = ALProxy("ALVideoDevice", IP, PORT)  # 申请video的代理
    # vision_definitions.kCameraSelectID = vision_definitions.kBottomCamera

    if (cameraID == 0):  # Bottom Camera
        camProxy.setCameraParameter("test", 18, 0)  # const std::string& Handle, const int& Id, const int& NewValue
    elif (cameraID == 1):  # Top Camera
        camProxy.setCameraParameter("test", 18, 1)

    resolution = 2
    colorSpace = vision_definitions.kRGBColorSpace
    fps = 15

    nameId = camProxy.subscribe("test", resolution, colorSpace, fps)
    naoImage = camProxy.getImageRemote(nameId)
    imageWidth = naoImage[0]
    imageHeight = naoImage[1]
    array = naoImage[6]
    im = Image.frombytes("RGB", (imageWidth, imageHeight), array)

    im.save("camImage.png", "PNG")
    camProxy.unsubscribe(nameId)


def Barbarization(image, pattern="red"):

    # Setting the pattern
    lower = []
    upper = []
    if pattern == "red":
        lower = np.array([0, 120, 120])  # HSV0-127 0-10 156-180
        upper = np.array([10, 255, 255])
    elif pattern == "yellow":  # 26-34
        lower = np.array([20, 100, 100])
        upper = np.array([34, 255, 255])
    elif pattern == "blue":
        lower = np.array([110, 70, 70])
        upper = np.array([124, 255, 255])

    # BGR to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Binarization
    mask = cv2.inRange(hsv, lower, upper)


    # Opened the image
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # cv2.imshow("Barbarization", opened)
    return opened


def calcTheLocate(img):

    col = np.ones(640)
    row = np.ones(480)
    colsum = []
    rowsum = []
    x = 0
    xw = 0  # w:west
    xe = 0  # e:est
    y = 0
    yn = 0  # n:north
    ys = 0  # s:south
    for i in range(0, 480):
        product = np.dot(col, img[i][:])
        colsum.append(product)
    for i in range(0, 480):
        if colsum[i] == max(colsum):
            y = i
            val = max(colsum) / 255
            yn = i - val
            ys = i + val
            break
    for i in range(0, 640):
        product = np.dot(row, img[:, i])
        rowsum.append(product)
    for i in range(0, 640):
        if rowsum[i] == max(rowsum):
            x = i
            val = max(colsum) / 255
            xw = i - val
            xe = i + val
            break
    print("locate  x: ", x, xw, xe, "........ locate y :", y, yn, ys)

    x = int(x)
    y = int(y)
    xw = int(xw)
    xe = int(xe)
    yn = int(yn)
    ys = int(ys)

    cv2.circle(img, (x, y), 5, (55, 255, 155), -1)
    cv2.circle(img, (xw, y), 5, (55, 255, 155), -1)
    cv2.circle(img, (xe, y), 5, (55, 255, 155), -1)
    cv2.circle(img, (x, yn), 5, (55, 255, 155), -1)
    cv2.circle(img, (x, ys), 5, (55, 255, 155), -1)
    cv2.putText(img, "center", (x - 20, y - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (55, 255, 155), 2)

    # cv2.imshow("two", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return x, y


def getDistance(x, y, cameraID):
    """

    :param x: x
    :param y: y
    :param cameraID:
    :return:
    """
    x = x - 320
    y = y - 240
    alpha = ((-x / 640) * 60.97) * math.pi / 180  # rads
    beta = ((y / 480) * 47.64) * math.pi / 180  # rads
    headAngle = getHeadAngle(IP, PORT)
    alpha = alpha + headAngle[0]
    beta = beta + headAngle[1]

    print("alpha", alpha, "beta", beta)
    print("alpha", alpha / math.pi * 180, "beta", beta / math.pi * 180)

    setHeadAngle(alpha, beta)
    motionProxy.setStiffnesses("Head", 0.0)
    H = 495
    cameraAngle = 1.2 * math.pi / 180
    if cameraID == 0:
        H = 495
        cameraAngle = 1.2 * math.pi / 180
    elif cameraID == 1:
        H = 477.33
        cameraAngle = 39.7 * math.pi / 180

    #  h = H - 210 - 105 / 2  ################## the height and the diam
    h = H - 155 - 40 / 2
    headPitchAngle = getHeadPitchAngle(IP, PORT)
    # s = (h-100)/math.tan(cameraAngle + headPitchAngle[0])
    s = h / math.tan(cameraAngle + headPitchAngle[0])
    # s = h/math.tan(cameraAngle +beta)
    x = s * math.cos(alpha) / 1000
    y = s * math.sin(alpha) / 1000
    return x, y, alpha


def getDistanceBottom(x, y):
    x = x - 320
    y = y - 240
    alpha = ((-x / 640) * 60.97) * math.pi / 180  # rads
    beta = ((y / 480) * 47.64) * math.pi / 180  # rads
    headAngle = getHeadAngle(IP, PORT)
    alpha = alpha + headAngle[0]
    beta = beta + headAngle[1]
    setHeadAngle(alpha, beta)
    motionProxy.setStiffnesses("Head", 0.0)

    print("Use bottom camera : ")
    print("alpha", alpha, "beta", beta)
    print("alpha", alpha / math.pi * 180, "beta", beta / math.pi * 180)

    H = 477.33
    cameraAngle = 39.7 * math.pi / 180
    h = H - 210 - 105 / 2
    s = h / math.tan(cameraAngle + beta)
    x = s * math.cos(alpha) / 1000
    y = s * math.sin(alpha) / 1000
    z = 30 / 2
    return x, y, z



def head(motionProxy):
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([1.96, 3.92, 5.96])
    keys.append([0.261799, 0.274544, 0.269942])

    names.append("HeadYaw")
    times.append([1.96, 3.92, 5.96])
    keys.append([0.387463, -0.659662, -0.0123138])

    motionProxy.setMoveArmsEnabled(False, False)
    motionProxy.angleInterpolation(names, keys, times, True)



def searchLandmark(motionProxy):
    getImage(IP, PORT, 0)
    tts = ALProxy("ALTextToSpeech", IP, PORT)
    landmarkTheoreticalSize = 0.06
    currentCamera = "CameraTop"
    memoryProxy = ALProxy("ALMemory", IP, 9559)
    landmarkProxy = ALProxy("ALLandMarkDetection", IP, 9559)
    landmarkProxy.subscribe("landmarkTest")
    markData = memoryProxy.getData("LandmarkDetected")
    motion = ALProxy("ALMotion", IP, 9559)
    time.sleep(1)
    markData = memoryProxy.getData("LandmarkDetected")
    motionProxy.setMoveArmsEnabled(False, False)
    setHeadAngle(0, 0)
    head(motionProxy)
    while markData is None or len(markData) == 0:
        motionProxy.moveTo(-0.01, 0.01, math.pi / 8)
        print("search.........")
        tts.say("searching mark. ")
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
    tts.say("I see " + str(markID))
    motion.post.moveTo(0, y, 0)
    motion.post.moveTo(x - 0.4, 0, 0)
    time.sleep(10.0)
    return markID


# ('lwx : ', -1.627792477607727, 'lwy : ', 0.47067877650260925, 'lwz : ', -0.4202498197555542)
# ('rwx : ', 1.8139231204986572, 'rwy : ', 0.49902573227882385, 'rwz : ', 0.371066689491272)


def grabball(IP, PORT):
    # Choregraphe bezier export in Python.
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append(
        [[0.12728, [3, -0.8, 0], [3, 0.386667, 0]], [0.118076, [3, -0.386667, 0.00712914], [3, 0.306667, -0.00565415]],
         [0.0889301, [3, -0.306667, 0], [3, 0.506667, 0]], [0.0889301, [3, -0.506667, 0], [3, 0.346667, 0]],
         [0.322098, [3, -0.346667, 0], [3, 0.786667, 0]], [0.0873961, [3, -0.786667, 0], [3, 0.426667, 0]],
         [0.0873961, [3, -0.426667, 0], [3, 0.693333, 0]], [-0.153442, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("HeadYaw")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append([[0.0137641, [3, -0.8, 0], [3, 0.386667, 0]], [-0.0153821, [3, -0.386667, 0], [3, 0.306667, 0]],
                 [-0.0153821, [3, -0.306667, 0], [3, 0.506667, 0]], [-0.0153821, [3, -0.506667, 0], [3, 0.346667, 0]],
                 [-0.0153821, [3, -0.346667, 0], [3, 0.786667, 0]], [0.0199001, [3, -0.786667, 0], [3, 0.426667, 0]],
                 [0.0199001, [3, -0.426667, 0], [3, 0.693333, 0]], [-0.01845, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("LAnklePitch")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append([[-1.22264, [3, -0.8, 0], [3, 0.386667, 0]],
                 [-1.22111, [3, -0.386667, -0.000570326], [3, 0.306667, 0.000452328]],
                 [-1.21957, [3, -0.306667, 0], [3, 0.506667, 0]], [-1.21957, [3, -0.506667, 0], [3, 0.346667, 0]],
                 [-1.2073, [3, -0.346667, 0], [3, 0.786667, 0]], [-1.2165, [3, -0.786667, 0], [3, 0.426667, 0]],
                 [-1.2165, [3, -0.426667, 0], [3, 0.693333, 0]], [0.0904641, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("LAnkleRoll")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append([[0.0859461, [3, -0.8, 0], [3, 0.386667, 0]], [0.073674, [3, -0.386667, 0], [3, 0.306667, 0]],
                 [0.0798099, [3, -0.306667, 0], [3, 0.506667, 0]], [0.0798099, [3, -0.506667, 0], [3, 0.346667, 0]],
                 [0.0844119, [3, -0.346667, -0.000625653], [3, 0.786667, 0.00141975]],
                 [0.0859461, [3, -0.786667, 0], [3, 0.426667, 0]], [0.0859461, [3, -0.426667, 0], [3, 0.693333, 0]],
                 [-0.102736, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("LElbowRoll")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append(
        [[-1.04001, [3, -0.8, 0], [3, 0.386667, 0]], [-1.02467, [3, -0.386667, -0.00855502], [3, 0.306667, 0.00678501]],
         [-0.99399, [3, -0.306667, -0.015231], [3, 0.506667, 0.0251643]],
         [-0.903484, [3, -0.506667, -0.0388613], [3, 0.346667, 0.0265893]],
         [-0.797638, [3, -0.346667, 0], [3, 0.786667, 0]], [-0.839057, [3, -0.786667, 0], [3, 0.426667, 0]],
         [-0.839057, [3, -0.426667, 0], [3, 0.693333, 0]], [-0.424876, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("LElbowYaw")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append([[-0.81613, [3, -0.8, 0], [3, 0.386667, 0]], [-0.80846, [3, -0.386667, 0], [3, 0.306667, 0]],
                 [-0.809994, [3, -0.306667, 0.00153398], [3, 0.506667, -0.0025344]],
                 [-0.857548, [3, -0.506667, 0.0112334], [3, 0.346667, -0.007686]],
                 [-0.866752, [3, -0.346667, 0.00920415], [3, 0.786667, -0.0208863]],
                 [-1.44354, [3, -0.786667, 0], [3, 0.426667, 0]], [-1.44354, [3, -0.426667, 0], [3, 0.693333, 0]],
                 [-1.18122, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("LHand")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append([[0.0140001, [3, -0.8, 0], [3, 0.386667, 0]], [0.0224, [3, -0.386667, 0], [3, 0.306667, 0]],
                 [0.0152, [3, -0.306667, 0], [3, 0.506667, 0]],
                 [0.0272, [3, -0.506667, -0.0107667], [3, 0.346667, 0.00736666]],
                 [0.0696, [3, -0.346667, -0.00070509], [3, 0.786667, 0.00160001]],
                 [0.0712, [3, -0.786667, 0], [3, 0.426667, 0]], [0.0712, [3, -0.426667, 0], [3, 0.693333, 0]],
                 [0.2996, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("LHipPitch")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append(
        [[-0.7102, [3, -0.8, 0], [3, 0.386667, 0]], [-0.704064, [3, -0.386667, -0.00427751], [3, 0.306667, 0.00339251]],
         [-0.68719, [3, -0.306667, -0.0092543], [3, 0.506667, 0.0152897]],
         [-0.630432, [3, -0.506667, 0], [3, 0.346667, 0]],
         [-0.707132, [3, -0.346667, 0.0132947], [3, 0.786667, -0.0301687]],
         [-0.760822, [3, -0.786667, 0], [3, 0.426667, 0]], [-0.760822, [3, -0.426667, 0], [3, 0.693333, 0]],
         [0.142704, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("LHipRoll")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append([[-0.075124, [3, -0.8, 0], [3, 0.386667, 0]], [-0.0659201, [3, -0.386667, 0], [3, 0.306667, 0]],
                 [-0.079726, [3, -0.306667, 0], [3, 0.506667, 0]], [-0.05825, [3, -0.506667, 0], [3, 0.346667, 0]],
                 [-0.101202, [3, -0.346667, 0], [3, 0.786667, 0]], [-0.075124, [3, -0.786667, 0], [3, 0.426667, 0]],
                 [-0.075124, [3, -0.426667, 0], [3, 0.693333, 0]], [0.107422, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("LHipYawPitch")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append([[-0.245398, [3, -0.8, 0], [3, 0.386667, 0]], [-0.243864, [3, -0.386667, 0], [3, 0.306667, 0]],
                 [-0.243864, [3, -0.306667, 0], [3, 0.506667, 0]],
                 [-0.276078, [3, -0.506667, 0.0103225], [3, 0.346667, -0.00706279]],
                 [-0.29602, [3, -0.346667, 0], [3, 0.786667, 0]], [-0.259204, [3, -0.786667, 0], [3, 0.426667, 0]],
                 [-0.259204, [3, -0.426667, 0], [3, 0.693333, 0]], [-0.16563, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("LKneePitch")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append([[2.1583, [3, -0.8, 0], [3, 0.386667, 0]], [2.15063, [3, -0.386667, 0], [3, 0.306667, 0]],
                 [2.1583, [3, -0.306667, 0], [3, 0.506667, 0]], [2.1583, [3, -0.506667, 0], [3, 0.346667, 0]],
                 [2.1583, [3, -0.346667, 0], [3, 0.786667, 0]], [2.15676, [3, -0.786667, 0], [3, 0.426667, 0]],
                 [2.15676, [3, -0.426667, 0], [3, 0.693333, 0]], [-0.09515, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("LShoulderPitch")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append([[1.42965, [3, -0.8, 0], [3, 0.386667, 0]], [1.41891, [3, -0.386667, 0], [3, 0.306667, 0]],
                 [1.42504, [3, -0.306667, 0], [3, 0.506667, 0]],
                 [1.41431, [3, -0.506667, 0.010738], [3, 0.346667, -0.00734705]],
                 [1.27164, [3, -0.346667, 0], [3, 0.786667, 0]], [1.28852, [3, -0.786667, 0], [3, 0.426667, 0]],
                 [1.28852, [3, -0.426667, 0], [3, 0.693333, 0]], [1.44652, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("LShoulderRoll")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append(
        [[0.164096, [3, -0.8, 0], [3, 0.386667, 0]], [0.153358, [3, -0.386667, 0.00193415], [3, 0.306667, -0.00153398]],
         [0.151824, [3, -0.306667, 0.00153398], [3, 0.506667, -0.0025344]],
         [0.0950661, [3, -0.506667, 0], [3, 0.346667, 0]], [0.136484, [3, -0.346667, 0], [3, 0.786667, 0]],
         [-0.151908, [3, -0.786667, 0], [3, 0.426667, 0]], [-0.151908, [3, -0.426667, 0], [3, 0.693333, 0]],
         [0.200912, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("LWristYaw")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append([[0.130348, [3, -0.8, 0], [3, 0.386667, 0]], [0.130348, [3, -0.386667, 0], [3, 0.306667, 0]],
                 [0.138018, [3, -0.306667, 0], [3, 0.506667, 0]], [0.138018, [3, -0.506667, 0], [3, 0.346667, 0]],
                 [0.138018, [3, -0.346667, 0], [3, 0.786667, 0]], [0.193243, [3, -0.786667, 0], [3, 0.426667, 0]],
                 [0.193243, [3, -0.426667, 0], [3, 0.693333, 0]], [0.124212, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("RAnklePitch")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append([[-1.21642, [3, -0.8, 0], [3, 0.386667, 0]], [-1.20875, [3, -0.386667, 0], [3, 0.306667, 0]],
                 [-1.21335, [3, -0.306667, 0], [3, 0.506667, 0]], [-1.21335, [3, -0.506667, 0], [3, 0.346667, 0]],
                 [-1.21642, [3, -0.346667, 0], [3, 0.786667, 0]], [-1.21642, [3, -0.786667, 0], [3, 0.426667, 0]],
                 [-1.21642, [3, -0.426667, 0], [3, 0.693333, 0]], [0.0828779, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("RAnkleRoll")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append([[-0.078192, [3, -0.8, 0], [3, 0.386667, 0]], [-0.078192, [3, -0.386667, 0], [3, 0.306667, 0]],
                 [-0.082794, [3, -0.306667, 0], [3, 0.506667, 0]], [-0.0720561, [3, -0.506667, 0], [3, 0.346667, 0]],
                 [-0.076658, [3, -0.346667, 0], [3, 0.786667, 0]], [-0.076658, [3, -0.786667, 0], [3, 0.426667, 0]],
                 [-0.076658, [3, -0.426667, 0], [3, 0.693333, 0]], [0.105888, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("RElbowRoll")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append([[1.03089, [3, -0.8, 0], [3, 0.386667, 0]], [0.768576, [3, -0.386667, 0], [3, 0.306667, 0]],
                 [1.29474, [3, -0.306667, 0], [3, 0.506667, 0]], [1.12293, [3, -0.506667, 0], [3, 0.346667, 0]],
                 [1.21804, [3, -0.346667, -0.0153279], [3, 0.786667, 0.0347825]],
                 [1.27326, [3, -0.786667, 0], [3, 0.426667, 0]], [1.27326, [3, -0.426667, 0], [3, 0.693333, 0]],
                 [0.412688, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("RElbowYaw")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append([[0.806842, [3, -0.8, 0], [3, 0.386667, 0]], [1.00013, [3, -0.386667, 0], [3, 0.306667, 0]],
                 [-0.105888, [3, -0.306667, 0.176603], [3, 0.506667, -0.291779]],
                 [-0.405018, [3, -0.506667, 0], [3, 0.346667, 0]],
                 [-0.3636, [3, -0.346667, -0.0414178], [3, 0.786667, 0.0939867]],
                 [0.246933, [3, -0.786667, 0], [3, 0.426667, 0]], [0.246933, [3, -0.426667, 0], [3, 0.693333, 0]],
                 [1.20108, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("RHand")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append([[0.2704, [3, -0.8, 0], [3, 0.386667, 0]], [0.3268, [3, -0.386667, 0], [3, 0.306667, 0]],
                 [0.2708, [3, -0.306667, 0], [3, 0.506667, 0]],
                 [0.8952, [3, -0.506667, -0.0912], [3, 0.346667, 0.0624]],
                 [0.9576, [3, -0.346667, -0.0106855], [3, 0.786667, 0.0242478]],
                 [1, [3, -0.786667, 0], [3, 0.426667, 0]], [0, [3, -0.426667, 0], [3, 0.693333, 0]],
                 [0.3116, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("RHipPitch")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append([[-0.702614, [3, -0.8, 0], [3, 0.386667, 0]],
                 [-0.696478, [3, -0.386667, -0.00199619], [3, 0.306667, 0.00158319]],
                 [-0.691876, [3, -0.306667, -0.00460194], [3, 0.506667, 0.00760321]],
                 [-0.627448, [3, -0.506667, 0], [3, 0.346667, 0]],
                 [-0.698012, [3, -0.346667, 0.0139203], [3, 0.786667, -0.0315884]],
                 [-0.763974, [3, -0.786667, 0], [3, 0.426667, 0]], [-0.763974, [3, -0.426667, 0], [3, 0.693333, 0]],
                 [0.131882, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("RHipRoll")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append([[0.0752079, [3, -0.8, 0], [3, 0.386667, 0]], [0.067538, [3, -0.386667, 0], [3, 0.306667, 0]],
                 [0.0782759, [3, -0.306667, 0], [3, 0.506667, 0]],
                 [0.0552659, [3, -0.506667, 0.00224198], [3, 0.346667, -0.00153398]],
                 [0.0537319, [3, -0.346667, 0], [3, 0.786667, 0]], [0.0690719, [3, -0.786667, 0], [3, 0.426667, 0]],
                 [0.0690719, [3, -0.426667, 0], [3, 0.693333, 0]], [-0.110406, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("RHipYawPitch")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append([[-0.245398, [3, -0.8, 0], [3, 0.386667, 0]], [-0.243864, [3, -0.386667, 0], [3, 0.306667, 0]],
                 [-0.243864, [3, -0.306667, 0], [3, 0.506667, 0]],
                 [-0.276078, [3, -0.506667, 0.0103225], [3, 0.346667, -0.00706279]],
                 [-0.29602, [3, -0.346667, 0], [3, 0.786667, 0]], [-0.259204, [3, -0.786667, 0], [3, 0.426667, 0]],
                 [-0.259204, [3, -0.426667, 0], [3, 0.693333, 0]], [-0.16563, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("RKneePitch")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append([[2.15224, [3, -0.8, 0], [3, 0.386667, 0]], [2.15531, [3, -0.386667, 0], [3, 0.306667, 0]],
                 [2.14764, [3, -0.306667, 0], [3, 0.506667, 0]], [2.15838, [3, -0.506667, 0], [3, 0.346667, 0]],
                 [2.15378, [3, -0.346667, 0], [3, 0.786667, 0]], [2.15838, [3, -0.786667, 0], [3, 0.426667, 0]],
                 [2.15838, [3, -0.426667, 0], [3, 0.693333, 0]], [-0.0935321, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("RShoulderPitch")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append(
        [[1.4374, [3, -0.8, 0], [3, 0.386667, 0]], [0.467912, [3, -0.386667, 0.283171], [3, 0.306667, -0.224584]],
         [-0.0858622, [3, -0.306667, 0], [3, 0.506667, 0]],
         [0.234744, [3, -0.506667, -0.040356], [3, 0.346667, 0.027612]],
         [0.262356, [3, -0.346667, -0.027612], [3, 0.786667, 0.0626579]],
         [0.716419, [3, -0.786667, 0], [3, 0.426667, 0]], [0.716419, [3, -0.426667, 0], [3, 0.693333, 0]],
         [1.43126, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("RShoulderRoll")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append([[-0.16418, [3, -0.8, 0], [3, 0.386667, 0]], [-0.467912, [3, -0.386667, 0], [3, 0.306667, 0]],
                 [0.08126, [3, -0.306667, 0], [3, 0.506667, 0]],
                 [-0.0138481, [3, -0.506667, 0.0346109], [3, 0.346667, -0.0236811]],
                 [-0.093616, [3, -0.346667, 0.00946402], [3, 0.786667, -0.0214761]],
                 [-0.115092, [3, -0.786667, 0], [3, 0.426667, 0]], [-0.115092, [3, -0.426667, 0], [3, 0.693333, 0]],
                 [-0.200996, [3, -0.693333, 0], [3, 0, 0]]])

    names.append("RWristYaw")
    times.append([2.36, 3.52, 4.44, 5.96, 7, 9.36, 10.64, 12.72])
    keys.append([[-0.116626, [3, -0.8, 0], [3, 0.386667, 0]], [-0.857548, [3, -0.386667, 0], [3, 0.306667, 0]],
                 [0.122678, [3, -0.306667, 0], [3, 0.506667, 0]],
                 [-0.107422, [3, -0.506667, 0.0807587], [3, 0.346667, -0.055256]],
                 [-0.285366, [3, -0.346667, 0.0547427], [3, 0.786667, -0.124224]],
                 [-0.644321, [3, -0.786667, 0], [3, 0.426667, 0]], [-0.644321, [3, -0.426667, 0], [3, 0.693333, 0]],
                 [0.0858622, [3, -0.693333, 0], [3, 0, 0]]])

    try:
        # uncomment the following line and modify the IP if you use this script outside Choregraphe.
        motion = ALProxy("ALMotion", IP, 9559)
        motion.angleInterpolationBezier(names, times, keys)
    except BaseException, err:
        print err


def putball(IP, strength=8.0):
    names = list()
    times = list()
    keys = list()

    if strength < 0.0:
        strength = -4.0
    elif strength > 20.0:
        strength = 16.0
    else:
        strength = strength * 1.0 - 4.0

    names.append("RElbowRoll")
    times.append([0.40000, 1.20000, 2.50000])
    keys.append([[0.98640, [3, -0.13333, 0.00000], [3, 0.26667, 0.00000]],
                 [0.35081, [3, -0.26667, 0.00000], [3, 0.20000, 0.00000]],
                 [0.57072, [3, -0.20000, 0.00000], [3, 0.00000, 0.00000]]])

    names.append("RElbowYaw")
    times.append([0.40000])
    keys.append([[1.38669, [3, -0.13333, 0.00000], [3, 0.00000, 0.00000]]])

    names.append("RHand")
    times.append([0.40000, 1.20000, 2.50000])
    keys.append([[0.00450, [3, -0.13333, 0.00000], [3, 0.26667, 0.00000]],
                 [0.00489, [3, -0.26667, 0.00000], [3, 0.20000, 0.00000]],
                 [0.00454, [3, -0.20000, 0.00000], [3, 0.00000, 0.00000]]])

    names.append("RShoulderPitch")
    times.append([0.40000, 0.80000, 1.20000, 2.50000])
    keys.append([[1.41439, [3, -0.13333, 0.00000], [3, 0.13333, 0.00000]],
                 [0.87790, [3, -0.13333, 0.20461], [3, 0.13333, -0.20461]],
                 [0.18675, [3, -0.13333, 0.00000], [3, 0.20000, 0.00000]],
                 [0.33161, [3, -0.20000, 0.00000], [3, 0.00000, 0.00000]]])

    names.append("RShoulderRoll")
    times.append([0.40000, 0.80000, 1.20000, 3.00000])
    keys.append([[-0.05236, [3, -0.13333, 0.00000], [3, 0.13333, 0.00000]],
                 [-0.96866, [3, -0.13333, 0.00000], [3, 0.13333, 0.00000]],
                 [-0.68591, [3, -0.13333, -0.12566], [3, 0.09333, 0.08796]],
                 [strength, [3, strength / 2, 0.00000], [3, 0.00000, 0.00000]]])

    names.append("RWristYaw")
    times.append([0.40000, 1.20000, 3.00000])
    keys.append([[-0.00925, [3, -0.13333, 0.00000], [3, 0.26667, 0.00000]],
                 [-0.69639, [3, -0.26667, 0.21931], [3, 0.20000, -0.16449]],
                 [-1.16064, [3, -0.20000, 0.00000], [3, 0.00000, 0.00000]]])

    motion = ALProxy("ALMotion", IP, 9559)
    postureProxy = ALProxy("ALRobotPosture", IP, 9559)
    postureProxy.goToPosture("StandInit", 0.5)

    motion.angleInterpolationBezier(names, times, keys);
    motion.openHand("RHand")
    time.sleep(1.0)
    postureProxy.goToPosture("StandInit", 0.5)


if __name__ == '__main__':

    tts = ALProxy("ALTextToSpeech", IP, PORT)
    memory = ALProxy("ALMemory", IP, PORT)
    setHeadAngle(0, 0.25)
    motionProxy.setStiffnesses("Head", 0.0)
    tts.say("Touch my head , let me find the red ball")
    Seeflag = 1
    while Seeflag:
        if memory.getData('MiddleTactilTouched', 0) == 1:
            getImage(IP, PORT, 0)
            img = cv2.imread("camImage.png")
            af = Barbarization(img, "red")
            x, y = calcTheLocate(af)
            print("find center: ", x, y)
            if x == 0 & y == 0:
                tts.say("I didn't see the red ball!")
            else:
                arrive = 1
                tts.say("I see the red ball! moving to there. ")
                while arrive:
                    x, y, theta = getDistance(x, y, 0)
                    moveConfig = [["MaxStepFrequency", 0]]
                    motionProxy.moveTo(0.1, y + 0.06 * math.tan(math.radians(12)), theta, moveConfig)
                    print("walk 0:", x, y, theta)
                    time.sleep(1)
                    setHeadAngle(0, 0.25)
                    motionProxy.setStiffnesses("Head", 0.0)
                    getImage(IP, PORT, 0)
                    img = cv2.imread("camImage.png")
                    af = Barbarization(img, "red")
                    x, y = calcTheLocate(af)
                    print("find center: ", x, y)
                    if x == 0 & y == 0:
                        arrive = 0
                setHeadAngle(0, 0.25)
                motionProxy.setStiffnesses("Head", 0.0)
                getImage(IP, PORT, 1)
                img = cv2.imread("camImage.png")
                af = Barbarization(img, "red")
                x, y = calcTheLocate(af)
                if x == 0 & y == 0:
                    tts.say("Error ,I didn't see the red ball!")
                    tts.say("Touch my head , let me find the red ball")
                else:
                    print("find cinter: ", x, y)
                    x, y, theta = getDistance(x, y, 1)
                    print("walk 2:", x, y, theta)
                    motionProxy.moveTo(x - 0.12, y - (x - 0.12) * math.tan(math.radians(12)), theta, moveConfig)
                    # motionProxy.walkTo(x - 0.15, y - 0.1, theta)
                    tts.say("I have arrived!")
                    grabball(IP, PORT)
                    searchLandmark(motionProxy)
                    putball(IP, PORT)
'''
    setHeadAngle(0, 0.25)
    motionProxy.setStiffnesses("Head", 0.0)  

    getImage(IP, PORT, 0)  
    img = cv2.imread("camImage.png")  
    af = Barbarization(img, "red")  
    x, y = calcTheLocate(af)  
    print("find center: ", x, y)  
    af = Barbarization(img, "red")
    x, y = calcTheLocate(af)
    x, y, theta = getDistance(x, y, 0)
    print("walk 0:", x, y, theta)

    moveConfig = [["MaxStepFrequency", 0]]
    motionProxy.moveTo(x - 0.16, y - (x - 0.16) * math.tan(math.radians(12)), theta, moveConfig)
    time.sleep(1)

    tts.say("I have arrived!")

    setHeadAngle(0, 0)
    motionProxy.setStiffnesses("Head", 0.0)  

    getImage(IP, PORT, 1)
    img = cv2.imread("camImage.png")
    af = Barbarization(img, "red")
    x, y = calcTheLocate(af)

    print("find cinter: ", x, y)  
    x, y, theta = getDistance(x, y, 1)
    print("walk 1:", x, y, theta)
    # motionProxy.moveTo(x - 0.1, y - (x - 0.1) * math.tan(math.radians(12)), theta, moveConfig)
    motionProxy.walkTo(x - 0.04, y, theta)

    
    tts = ALProxy("ALTextToSpeech", IP, PORT)
    tts.say("I will grab")
    grabball(IP, PORT)

    markId = None
    # markId = getMarkId(motionProxy)
    while True:
        if markId == None:
            markId = searchLandmark(motionProxy)  ###
        else:
            break

    landmark()
    putball(IP, PORT)
    motion1 = ALProxy("ALMotion", IP, 9559)
    postureProxy1 = ALProxy("ALRobotPosture", IP, 9559)
    postureProxy1.goToPosture("StandInit", 0.5)
    motionProxy.rest()
    '''
