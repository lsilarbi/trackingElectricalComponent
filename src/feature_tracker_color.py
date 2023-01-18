#!/usr/bin/env python3

import cv2
import depthai as dai
from collections import deque
import time

# Paramters
############################################
detection_intervale_x = [440, 840]
detection_intervale_y = [160, 560]

# a feature will be assiciated with an component(average) if the distance between both is +-borne_x on x and +-borne_y on y
borne_x = 50
borne_y = 50

# number of features to consider as a component
nbr_feat = 30
############################################

    
class FeatureTrackerDrawer:

    maxTrackedFeaturesPathLength = 30
    # for how many frames the feature is tracked
    trackedFeaturesPathLength = 4

    trackedIDs = None
    trackedFeaturesPath = None

    def onTrackBar(self, val):
        FeatureTrackerDrawer.trackedFeaturesPathLength = val
        pass

    def trackFeaturePath(self, features):

        newTrackedIDs = set()
        for currentFeature in features:
            currentID = currentFeature.id
            newTrackedIDs.add(currentID)

            if currentID not in self.trackedFeaturesPath:
                self.trackedFeaturesPath[currentID] = deque()

            path = self.trackedFeaturesPath[currentID]
            
            if (detection_intervale_x[0] < int(currentFeature.position.x) and int(currentFeature.position.x) < detection_intervale_x[1] and detection_intervale_y[0] < int(currentFeature.position.y) and int(currentFeature.position.y) < detection_intervale_y[1]):
                path.append(currentFeature.position)
                while(len(path) > max(1, FeatureTrackerDrawer.trackedFeaturesPathLength)):
                    path.popleft()

            self.trackedFeaturesPath[currentID] = path

        featuresToRemove = set()
        for oldId in self.trackedIDs:
            if oldId not in newTrackedIDs:
                featuresToRemove.add(oldId)

        for id in featuresToRemove:
            self.trackedFeaturesPath.pop(id)

        self.trackedIDs = newTrackedIDs

    # identify electrical component
    def componentDectector(self, img, show):
        circleRadius = 3
        retained_features = []
        component_centers = []
        condition = 0
        averages = [] # Array of averages corresponding to different components [sum x, sum y, value number in the sum]

        for i, featurePath in enumerate(self.trackedFeaturesPath.values()):
            path = featurePath
            if condition == 0 and len(path)-1 > 0:
                averages.append([path[0].x, path[0].y, 1])
                condition = 1

            for j in range(len(path) - 1):
                for k in range(len(averages)):
                    if path[j].x > averages[k][0]/averages[k][2] - borne_x and path[j].x < averages[k][0]/averages[k][2] + borne_x and path[j].y > averages[k][1]/averages[k][2]  - borne_y and path[j].y < averages[k][1]/averages[k][2] + borne_y:
                        averages[k][0] += path[j].x
                        averages[k][1] += path[j].y
                        averages[k][2] +=1
                        break

                    elif k == len(averages)-1:
                        averages.append([path[j].x, path[j].y, 1])
                        break
        for i, component_center in enumerate(averages):
            if component_center[2] >= nbr_feat:
                cx = component_center[0]/component_center[2]
                cy = component_center[1]/component_center[2]
                cv2.circle(img, (int(cx), int(cy)), circleRadius, color = (255, 0, 0), thickness = -1)
                cv2.putText(img, text = str(i), org = (int(cx-10), int(cy-11)), fontFace= 1, fontScale = 1, color = (0, 0, 255), thickness = 1)
                if show:
                    print("=================================================================")
                    print("Liste des composants détéctés:")
                    print("")
                    print("Composant " + str(i) + ":")
                    print("Coordonnées du centre en pixels: [" + str(int(cx)) + "," + str(int(cy)) + "]")
                    print("")
                    print("=================================================================")
                    print("")
        
        cv2.setTrackbarPos(self.trackbarName, self.windowName, FeatureTrackerDrawer.trackedFeaturesPathLength)

    def __init__(self, trackbarName, windowName):
        self.trackbarName = trackbarName
        self.windowName = windowName
        cv2.namedWindow(windowName)
        cv2.createTrackbar(trackbarName, windowName, FeatureTrackerDrawer.trackedFeaturesPathLength, FeatureTrackerDrawer.maxTrackedFeaturesPathLength, self.onTrackBar)
        self.trackedIDs = set()
        self.trackedFeaturesPath = dict()


# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
colorCam = pipeline.create(dai.node.ColorCamera)
featureTrackerColor = pipeline.create(dai.node.FeatureTracker)

xoutPassthroughFrameColor = pipeline.create(dai.node.XLinkOut)
xoutTrackedFeaturesColor = pipeline.create(dai.node.XLinkOut)
xinTrackedFeaturesConfig = pipeline.create(dai.node.XLinkIn)

xoutPassthroughFrameColor.setStreamName("passthroughFrameColor")
xoutTrackedFeaturesColor.setStreamName("trackedFeaturesColor")
xinTrackedFeaturesConfig.setStreamName("trackedFeaturesConfig")

# Properties
colorCam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

if 1:
    colorCam.setIspScale(2,3)
    colorCam.video.link(featureTrackerColor.inputImage)
else:
    colorCam.isp.link(featureTrackerColor.inputImage)

# Linking
featureTrackerColor.passthroughInputImage.link(xoutPassthroughFrameColor.input)
featureTrackerColor.outputFeatures.link(xoutTrackedFeaturesColor.input)
xinTrackedFeaturesConfig.out.link(featureTrackerColor.inputConfig)

# By default the least mount of resources are allocated
# increasing it improves performance
numShaves = 2
numMemorySlices = 2
featureTrackerColor.setHardwareResources(numShaves, numMemorySlices)
featureTrackerConfig = featureTrackerColor.initialConfig.get()

print("Press 's' to switch between Lucas-Kanade optical flow and hardware accelerated motion estimation!")

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    # Output queues used to receive the results
    passthroughImageColorQueue = device.getOutputQueue("passthroughFrameColor", 8, False)
    outputFeaturesColorQueue = device.getOutputQueue("trackedFeaturesColor", 8, False)

    inputFeatureTrackerConfigQueue = device.getInputQueue("trackedFeaturesConfig")

    colorWindowName = "color"
    colorFeatureDrawer = FeatureTrackerDrawer("Feature tracking duration (frames)", colorWindowName)

    while True:
        inPassthroughFrameColor = passthroughImageColorQueue.get()
        passthroughFrameColor = inPassthroughFrameColor.getCvFrame()
        colorFrame = passthroughFrameColor

        trackedFeaturesColor = outputFeaturesColorQueue.get().trackedFeatures
        colorFeatureDrawer.trackFeaturePath(trackedFeaturesColor)
        colorFeatureDrawer.componentDectector(colorFrame, show = False)

       # draw the detection zone
        cv2.line(colorFrame, (detection_intervale_x[0], detection_intervale_y[0]), (detection_intervale_x[0], detection_intervale_y[1]), color = (255, 0, 0), thickness = 1)
        cv2.line(colorFrame, (detection_intervale_x[1], detection_intervale_y[0]), (detection_intervale_x[1], detection_intervale_y[1]), color = (255, 0, 0), thickness = 1)
        cv2.line(colorFrame, (detection_intervale_x[0], detection_intervale_y[0]), (detection_intervale_x[1], detection_intervale_y[0]), color = (255, 0, 0), thickness = 1)
        cv2.line(colorFrame, (detection_intervale_x[0], detection_intervale_y[1]), (detection_intervale_x[1], detection_intervale_y[1]), color = (255, 0, 0), thickness = 1)
        cv2.putText(colorFrame, text = "Detection zone", org = (detection_intervale_x[0], detection_intervale_y[0]-3), fontFace= 1, fontScale = 1, color = (255, 0, 0), thickness = 1)

        # Show the frame
        cv2.imshow(colorWindowName, colorFrame)

        featureTrackerConfig.motionEstimator.type = dai.FeatureTrackerConfig.MotionEstimator.Type.LUCAS_KANADE_OPTICAL_FLOW

        cfg = dai.FeatureTrackerConfig()
        cfg.set(featureTrackerConfig)
        inputFeatureTrackerConfigQueue.send(cfg)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

