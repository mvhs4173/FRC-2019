#!/usr/bin/env python3
import cv2
import json
import sys
import numpy as np
import time

from cscore import CameraServer, VideoSource, CvSource, VideoMode, CvSink, UsbCamera
from networktables import NetworkTablesInstance

#Camera resolution
cameraWidth = 160
cameraHeight = 120

########Modes##########
disableVisionProcessing = False
colorPickerMode = False #Set this to true if color thresholds need to be changed
testingMode = False

configFile = "/boot/frc.json"

class CameraConfig: pass

team = None
server = False
cameraConfigs = []

"""Report parse error."""
def parseError(str):
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

"""Read single camera configuration."""
def readCameraConfig(config):
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    cam.config = config

    cameraConfigs.append(cam)
    return True

"""Read configuration file."""
def readConfig():
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False

    return True

readConfig()
#Set up the camera
camServer = CameraServer.getInstance()
camServer.enableLogging()
camera = UsbCamera("rPi Camera 0", 0)
camera.setFPS(30)
camera.setResolution(cameraWidth, cameraHeight)
camServer.addCamera(camera)

cvSink = camServer.getVideo()
outputStream = camServer.putVideo("Processed Frames", cameraWidth, cameraHeight)

ntinst = NetworkTablesInstance.getDefault()

print("Setting up NetworkTables client for team {}".format(team))
ntinst.startClientTeam(team)
dashboard = ntinst.getTable('SmartDashboard')

#outputStream = camServer.putVideo("Vision", cameraWidth, cameraHeight)


#camera = camServer.startAutomaticCapture();
#camera.setResolution(cameraWidth, cameraHeight)
#cvSink = camServer.getVideo()


#camera = cv2.VideoCapture(0)
#camera.set(cv2.CAP_PROP_FRAME_WIDTH, cameraWidth);
#camera.set(cv2.CAP_PROP_FRAME_HEIGHT, cameraHeight);


#Gets the distance between the points in pixels
def distanceBetweenPoints(p1, p2):
    return int(np.sqrt([abs(p2[0] - p1[0])**2 + abs(p2[1] - p1[1])**2])[0])

#Orders the points given and return them in an array in the following order
#Index 0: Top Left Point
#Index 1: Top Right Point
#Index 2: Bottom Left Point
#Index 3: Bottom Right Point
def order_points(pts):
    centerPosition = getCenterPositionOfPoints(pts)
    topLeft = centerPosition
    topRight = centerPosition
    bottomLeft = centerPosition
    bottomRight = centerPosition

    closestDist = -1
    farthestDist = -1

    top1 = None
    top2 = None
    bottom1 = None
    bottom2 = None
    
    #Calculate the topLeft and topRight based on their distances from zero
    for point in pts:
        #If the point is above the center point then its one of the top two points
        if point[1] <= centerPosition[1]:
            if top1 is None:
                top1 = point
            else:
                top2 = point
        elif point[1] > centerPosition[1]:
            if bottom1 is None:
                bottom1 = point
            else:
                bottom2 = point
    if top1 is not None and bottom1 is not None and top2 is not None and bottom2 is not None:
        if top1[0] < top2[0]:
            topLeft = top1
            topRight = top2
        else:
            topLeft = top2
            topRight = top1

        if bottom1[0] < bottom2[0]:
            bottomLeft = bottom1
            bottomRight = bottom2
        else:
            bottomLeft = bottom2
            bottomRight = bottom1
    else:
        return [centerPosition, centerPosition, centerPosition, centerPosition]
                
    return [topLeft, topRight, bottomLeft, bottomRight];

def getCenterPositionOfPoints(points):
    totalX = 0
    totalY = 0

    for pt in points:
        totalX += pt[0]
        totalY += pt[1]

    return [int(totalX/len(points)), int(totalY/len(points))];







##############Vision Target Class##########################
#A class that describes information about a Vision target the camera may see on the field
class VisionTarget:
    #boundingBoxCoordinates - A TopLeft Corner and BottomRight Corner that encompasses the two contours
    #contour1 - A contour 
    def __init__(self, boundingBoxCoordinates, contour1, contour2):
        self.tiltAllowance = 5000#How small the difference between areas of both strips of tape have to be for the camera to be considered in front of it
        self.distFromCenterAllowance = 25#How many pixels away from the center that will still be considered "center"

        self.boundingBoxCoordinates = boundingBoxCoordinates

        #Figure out the left and the right contour and calculate bounding boxes
        rect1 = cv2.minAreaRect(contour1)
        rect2 = cv2.minAreaRect(contour2)
        
        rect1Points = order_points(Vision.getRotatedBoxPoints(rect1))
        rect2Points = order_points(Vision.getRotatedBoxPoints(rect2))

        #Set left objects to contour1
        if rect1Points[0][0] <= rect2Points[0][0]:
            self.leftContour = contour1
            self.leftRect = rect1
            self.leftRectPoints = rect1Points

            self.rightContour = contour2
            self.rightRect = rect2
            self.rightRectPoints = rect2Points
        else:#Set left objects to contour2
            self.leftContour = contour2
            self.leftRect = rect2
            self.leftRectPoints = rect2Points

            self.rightContour = contour1
            self.rightRect = rect1
            self.rightRectPoints = rect1Points

        #Find the center position of the vision target
        self.visionTargetPosition = [(boundingBoxCoordinates[0][0] + boundingBoxCoordinates[1][0])/2, (boundingBoxCoordinates[0][1] + boundingBoxCoordinates[1][1])/2]

    #Gets the position of the center of the vision target
    def getPosition(self):
        return self.visionTargetPosition;

    #Returns the coordinates
    #of the TopLeft Corner and BottomRight corner of the Vision Target
    def getBoundingBox(self):
        return self.boundingBoxCoordinates;

    def getLeftBoundingBox(self):
        return cv2.boundingRect(self.leftContour)

    def getRightBoundingBox(self):
        return cv2.boundingRect(self.rightContour);

    def getLeftBoxArea(self):
        x, y, w, h = self.getLeftBoundingBox()
        return w*h

    def getRightBoxArea(self):
        x, y, w, h = self.getRightBoundingBox()
        return w*h

    #Returns how many pixels to the left or right the robot is offset from the Vision Target
    #If the number is closer to zero then the camera is closer to facing the Target straight on
    def getCameraHorizontalOffset(self):
        leftArea = self.getLeftBoxArea()
        rightArea = self.getRightBoxArea()

        offset = rightArea - leftArea#The difference between widths of the boxes
        
        return offset

    #Returns true if the camera is aligned with target
    def isAlignedWithCamera(self):
        targetPosition = self.getPosition()
        offset = self.getCameraHorizontalOffset()

        #First make sure the camera isnt looking at the target from the side
        if abs(offset) <= self.tiltAllowance:
            #Now make sure the target is in the center of the screen
            screenCenter = [cameraWidth/2, cameraHeight/2]
            horizontalDist = abs(screenCenter[0] - targetPosition[0])#The distance between the two points on the screen in pixels
            #Check if the vision target is close enough to center of the screen to be considered centered
            if horizontalDist <= self.distFromCenterAllowance:
                return True;

        return False;#If the above conditions are false then the target is not centered






            
        
########################################################################
#################    MAIN VISION CLASS    ##############################
########################################################################
class Vision:
    def __init__(self):
        self.tapeLowerThresh = np.array([60, 30, 230])
        self.tapeUpperThresh = np.array([100, 120, 255])

    def getRotatedBoxPoints(rect):        
        boxPoints = cv2.boxPoints(rect)
        boxPoints = np.int0(boxPoints)
        return boxPoints;

    def getContourPosition(contour):
        M = cv2.moments(contour)
        x = int(M['m10']/max(M['m00'], 1))
        y = int(M['m01']/max(M['m00'], 1))

        return [x, y];

    def getRotatedBoxDimensions(box):
        (x, y), (width, height), angle = box
        return width, height;

    #Finds contours that resemble The strips of reflective tape
    #It may return contours for other things as well
    def recognizeReflectiveTape(self, image):
        thresholdMaxValue = 255
        thresholdValue = 231
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)#Convert the color space to HSV
        mask = cv2.inRange(hsv, self.tapeLowerThresh, self.tapeUpperThresh)#Search for only the color of what we are looking for
        #blur = cv2.blur(mask, (2, 2))#Blur the image
        ret, grayImage = cv2.threshold(mask, thresholdValue, thresholdMaxValue, cv2.THRESH_BINARY)#Get the image black and white for edge detection

        kernel = np.ones((10, 10), np.uint8)
        opening = cv2.morphologyEx(grayImage, cv2.MORPH_CLOSE, kernel)#Get rid of extra noise
        
        #Now find the contours in the image to find what we are looking for
        cntImage, contours, h = cv2.findContours(opening, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        return contours, cntImage

    def getTwoLargestContours(contourList):
        largestContour = []
        largestContourArea = 0
        indexOfLargestWhenFound = 0 #The index of the loop when the largest contour was found
        
        secondLargestContour = []
        secondLargestContourArea = 0

        #Find the largest box
        for cnt in contourList:
            area = cv2.contourArea(cnt)

            #Check the size of the boxes
            if len(largestContour) > 0 and area > largestContourArea: 
                largestContour = cnt
                largestContourArea = area
            elif len(largestContour) <= 0:
                largestContour = cnt
                largestContourArea = area

        #Find the second largest box
        for cnt in contourList:
            area = cv2.contourArea(cnt)
            
            if len(secondLargestContour) > 0 and secondLargestContourArea and area != largestContourArea:
                secondLargestContour = cnt
                secondLargestArea = area
            elif area != largestContourArea:
                secondLargestContour = cnt
                secondLargestArea = area

        return largestContour, secondLargestContour;

    #Calculates the TopLeft Corner and Bottom Right corner that will contain all the contours in the given list
    def getBoundingBoxOfContours(contours):
        topCorner = None
        bottomCorner = None

        #Check all of the contours
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)

            #Find top corner
            if topCorner is None:
                topCorner = [x, y]
            else:
                topCorner = [min(x, topCorner[0]), min(y, topCorner[1])]

            #Find the bottom corner
            if bottomCorner is None:
                bottomCorner = [x+w, y+h]
            else:
                bottomCorner = [max(x+w, bottomCorner[0]), max(y+h, bottomCorner[1])]
        

        return topCorner, bottomCorner;

    #Checks if the two boxes for reflective tape are paired together
    def checkRotatedBoxesPaired(box1, box2, image):
        b1 = Vision.getRotatedBoxPoints(box1)
        b2 = Vision.getRotatedBoxPoints(box2)
        box1Points = order_points(b1)
        box2Points = order_points(b2)
        
        box1TopLeft = box1Points[0]
        box2TopLeft = box2Points[0]

        leftBox = None
        rightBox = None

        #Determine which box is on the left
        if box1TopLeft[0] < box2TopLeft[0]:
            leftBox = box1Points
            rightBox = box2Points
        else:
            leftBox = box2Points
            rightBox = box1Points

        
        lx, ly, lw, lh = cv2.boundingRect(np.int0(leftBox))
        rx, ry, rw, rh = cv2.boundingRect(np.int0(rightBox))

        #Make sure the boxes are not overlapping
        if lx + lw < rx:
            #Make sure the boxes are facing towards each other
            if leftBox[0][1] < leftBox[1][1] and rightBox[0][1] > rightBox[1][1]:
                #Get the corners we will use to make the measurments
                leftBoxTopPoint = leftBox[0]
                rightBoxTopPoint = rightBox[0]

                leftBoxBottomPoint = leftBox[3]
                rightBoxBottomPoint = rightBox[3]


                topPointDistance = abs(leftBoxTopPoint[0] - rightBoxTopPoint[0])
                bottomPointDistance = abs(leftBoxBottomPoint[0] - rightBoxBottomPoint[0])
                
                #Check if the top corner distance is less than the bottom corner distance
                if topPointDistance < bottomPointDistance:
                    return True
                else:
                    return False;
        else:
            return False;

    def findNearestVisionTarget(self, image):
        contours, cntImage = self.recognizeReflectiveTape(image)#Find possible contours for reflective tape
        
        lastCnt = None
        contour1 = None
        contour2 = None

        index1 = -1
        index2 = -1
        #Compare each contour with all the other contours
        for cnt in contours:
            index1 += 1
            for cnt2 in contours:
                index2 += 1
                #Dont compare the same contour
                
                if index2 != index1:
                    box1 = cv2.minAreaRect(cnt)
                    box2 = cv2.minAreaRect(cnt2)
                    
                    width1, height1 = Vision.getRotatedBoxDimensions(box1)#Get the width and height of the box
                    width2, height2 = Vision.getRotatedBoxDimensions(box2)

                    box1Area = width1 * height1
                    box2Area = width2 * height2
                    
                    #Filter out small contours
                    if box1Area > 10 and box2Area > 10:
                        #Make sure the boxes resemble a rectangle because thats the shape we are looking for
                        #Now check if the two reflective tapes belong to the same target
                        
                        if (height1 > width1 * 2 or width1 > height1 * 2) and (height2 > width2 * 2 or width2 > height2 * 2):

                            #Check that these two contours are paired
                            if Vision.checkRotatedBoxesPaired(box1, box2, image):
                                #If this is first iteration
                                if contour1 is None and contour2 is None:
                                    contour1 = cnt
                                    contour2 = cnt2
                                else:
                                    contour1Area = cv2.contourArea(contour1)
                                    contour2Area = cv2.contourArea(contour2)
                                    cnt1Area = cv2.contourArea(cnt)
                                    cnt2Area = cv2.contourArea(cnt2)

                                    cntPosition = Vision.getContourPosition(cnt)
                                    cnt2Position = Vision.getContourPosition(cnt2)

                                    contour1Position = Vision.getContourPosition(contour1)
                                    contour2Position = Vision.getContourPosition(contour2)
                                    
                                    potentialTargetDistBetweenTape = abs(cntPosition[0] - cnt2Position[0])
                                    currentTargetDistBetweenTape = abs(contour1Position[0] - contour2Position[0])

                                    #Make sure the new contours are bigger and closer together to be considered targets
                                    if (cnt1Area + cnt2Area > contour1Area + contour2Area and potentialTargetDistBetweenTape < currentTargetDistBetweenTape) or potentialTargetDistBetweenTape < currentTargetDistBetweenTape:
                                        contour1 = cnt
                                        contour2 = cnt2
                

                    
        if contour1 is not None and contour2 is not None:
            return VisionTarget(Vision.getBoundingBoxOfContours([contour1, contour2]), contour1, contour2);
        else:
            return None;
                        
            

########################################################################
#################    MAIN PROGRAM LOOP    ##############################
########################################################################

vision = Vision()#Init vision processing object
frame = np.zeros(shape=(cameraHeight, cameraWidth, 3), dtype=np.uint8)

 
while True:
    gotFrame, frame = cvSink.grabFrame(frame)
    gotFrame = gotFrame != 0

    if not gotFrame:
        print(str(cvSink.getError()))

    if testingMode == True and gotFrame:
        colorPickerMode = False
        disableVisionProcessing = True
        cnts, cntImage = vision.recognizeReflectiveTape(frame)
        frame = cntImage
	
    if colorPickerMode == True and gotFrame:
        disableVisionProcessing = True
        
        #Get thresholds from dashboard
        hLow = dashboard.getNumber("HLOW", 0)
        sLow = dashboard.getNumber("SLOW", 0)
        vLow = dashboard.getNumber("VLOW", 0)

        hHigh = dashboard.getNumber("HHIGH", 180)
        sHigh = dashboard.getNumber("SHIGH", 255)
        vHigh = dashboard.getNumber("VHIGH", 255)

        lowerThresh = np.array([hLow, sLow, vLow])
        upperThresh = np.array([hHigh, sHigh, vHigh])

        #Filter the image
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        frame = cv2.inRange(hsv, lowerThresh, upperThresh)
        
        
	
    #Only run if vision is enabled
    if disableVisionProcessing == False and gotFrame:
        visionTarget, frame = vision.findNearestVisionTarget(frame)

        if visionTarget:
            boxCoordinates = visionTarget.getBoundingBox()

            font = cv2.FONT_HERSHEY_SIMPLEX
            
            #Detect if the robot is aligned with the VisionTarget
            if visionTarget.isAlignedWithCamera():
                cv2.rectangle(frame, (boxCoordinates[0][0], boxCoordinates[0][1]), (boxCoordinates[1][0], boxCoordinates[1][1]), (0, 255, 0), 2)
                #cv2.putText(frame,'Field Target - Aligned',(boxCoordinates[0][0], boxCoordinates[0][1] - 5), font, 0.2,(255,255,255),1,cv2.LINE_AA)
            else:
                cv2.rectangle(frame, (boxCoordinates[0][0], boxCoordinates[0][1]), (boxCoordinates[1][0], boxCoordinates[1][1]), (0, 0, 255), 2)
                #cv2.putText(frame,'Field Target - Unaligned',(boxCoordinates[0][0], boxCoordinates[0][1] - 5), font, 0.2,(255,255,255),1,cv2.LINE_AA)
	
    
    outputStream.putFrame(frame)
