#!/usr/bin/env python3

from turtle import left
import cv2
import depthai as dai

# Paramters
############################################
detection_intervale_x = [160, 450]
# 340mm = 290
detection_intervale_y = [10, 380]

# permit to use the two camera as one
# depend of the distance between the comera and the belt, need to be reset if the distance change.
# to setup it, put a peace of paper in front of the camera, the offsets corespond the difference between
# position of the corner in the tow axis
offset_on_x = 244 - 165
offset_on_y = 52 - 54


# a feature will be assiciated with an component(average) if the distance between both is +-borne_x on x and +-borne_y on y
borne_x = 22
borne_y = 22

# number of features to consider as a component
nbr_feat = 5

# decalage dû à la vitesse d'avance du tapis
decalage = 20
############################################


# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
featureTrackerLeft = pipeline.create(dai.node.FeatureTracker)
featureTrackerRight = pipeline.create(dai.node.FeatureTracker)

xoutPassthroughFrameLeft = pipeline.create(dai.node.XLinkOut)
xoutTrackedFeaturesLeft = pipeline.create(dai.node.XLinkOut)
xoutPassthroughFrameRight = pipeline.create(dai.node.XLinkOut)
xoutTrackedFeaturesRight = pipeline.create(dai.node.XLinkOut)
xinTrackedFeaturesConfig = pipeline.create(dai.node.XLinkIn)

xoutPassthroughFrameLeft.setStreamName("passthroughFrameLeft")
xoutTrackedFeaturesLeft.setStreamName("trackedFeaturesLeft")
xoutPassthroughFrameRight.setStreamName("passthroughFrameRight")
xoutTrackedFeaturesRight.setStreamName("trackedFeaturesRight")
xinTrackedFeaturesConfig.setStreamName("trackedFeaturesConfig")

# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# Disable optical flow
featureTrackerLeft.initialConfig.setMotionEstimator(False)
featureTrackerRight.initialConfig.setMotionEstimator(False)#########


# Linking
monoLeft.out.link(featureTrackerLeft.inputImage)
featureTrackerLeft.passthroughInputImage.link(xoutPassthroughFrameLeft.input)
featureTrackerLeft.outputFeatures.link(xoutTrackedFeaturesLeft.input)
xinTrackedFeaturesConfig.out.link(featureTrackerLeft.inputConfig)

monoRight.out.link(featureTrackerRight.inputImage)
featureTrackerRight.passthroughInputImage.link(xoutPassthroughFrameRight.input)
featureTrackerRight.outputFeatures.link(xoutTrackedFeaturesRight.input)
xinTrackedFeaturesConfig.out.link(featureTrackerRight.inputConfig)

featureTrackerConfig = featureTrackerRight.initialConfig.get()

print("Press 's' to switch between Harris and Shi-Thomasi corner detector!")

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    # Output queues used to receive the results
    passthroughImageLeftQueue = device.getOutputQueue("passthroughFrameLeft", 8, False)
    outputFeaturesLeftQueue = device.getOutputQueue("trackedFeaturesLeft", 8, False)
    passthroughImageRightQueue = device.getOutputQueue("passthroughFrameRight", 8, False)
    outputFeaturesRightQueue = device.getOutputQueue("trackedFeaturesRight", 8, False)

    inputFeatureTrackerConfigQueue = device.getInputQueue("trackedFeaturesConfig")

    leftWindowName = "left"
    rightWindowName = "right"
    combineWindowName = "combination of left and right result"

# transform the right feature in a left feature to compare
    def transformation(features):
        for feature in features:
            feature.position.x += offset_on_x
            feature.position.y += offset_on_y
        return features

# identify electrical component
    def componentDectector(features):
        retained_features = []
        component_centers = []
        averages = [] #tab of averages corresponding to different components [sum x, sum y, value number in the sum]
        # First we retain only the feature in the chosen interval
        for feature in features:
            if (detection_intervale_x[0] < int(feature.position.x) and int(feature.position.x) < detection_intervale_x[1] and detection_intervale_y[0] < int(feature.position.y) and int(feature.position.y) < detection_intervale_y[1]):
                retained_features.append(feature)
        averages.append([retained_features[0].position.x, retained_features[0].position.y, 1])

        for feature in retained_features:
            for i in range(len(averages)):
                if feature.position.x > averages[i][0]/averages[i][2] - borne_x and feature.position.x < averages[i][0]/averages[i][2] + borne_x and feature.position.y > averages[i][1]/averages[i][2]  - borne_y and feature.position.y < averages[i][1]/averages[i][2] + borne_y:
                    averages[i][0] += feature.position.x
                    averages[i][1] += feature.position.y
                    averages[i][2] +=1
                    break

                elif i == len(averages)-1:
                    averages.append([feature.position.x, feature.position.y, 1])
                    break

        for compontent_center in averages:
            if compontent_center[2] >= nbr_feat:
                component_centers.append([compontent_center[0]/compontent_center[2], compontent_center[1]/compontent_center[2]])

        return component_centers

# transform the right feature in a left feature to compare
    def transformation_inverse(component_centers):
        for component_center in component_centers:
            component_center[0] -= offset_on_x
            component_center[1] -= offset_on_y
        return component_centers

    def drawFeatures(frame, component_centers, show):
        circleRadius = 1
        for i, component_center in enumerate(component_centers):
            # print(component_center[0], component_center[1])
            cv2.rectangle(frame, (int(component_center[0]+10), int(component_center[1]+decalage+10)), (int(component_center[0]-10), int(component_center[1]+decalage-10)), color = (0, 0, 255), thickness = 1)
            cv2.circle(frame, (int(component_center[0]), int(component_center[1])), circleRadius, color = (0, 0, 255))
            cv2.putText(frame, text = str(i), org = (int(component_center[0]-10), int(component_center[1]-11)), fontFace= 1, fontScale = 1, color = (0, 0, 255), thickness = 1)

        
        if show:
            for i, component_center in enumerate(component_centers):
                print("=================================================================")
                print("Liste des composants détéctés:")
                print("")
                print("Composant " + str(i) + ":")
                print("Coordonnées du centre en pixels: [" + str(round(int(component_center[0]-detection_intervale_x[0])*0.340/290, 3)) + "," + str(round(int(component_center[1]+decalage-detection_intervale_y[0])*0.340/290, 3)) + "]")
                print("")
            print("=================================================================")
            print("")

    while True:
        inPassthroughFrameLeft = passthroughImageLeftQueue.get()
        passthroughFrameLeft = inPassthroughFrameLeft.getFrame()
        leftFrame = cv2.cvtColor(passthroughFrameLeft, cv2.COLOR_GRAY2BGR)

        inPassthroughFrameRight = passthroughImageRightQueue.get()
        passthroughFrameRight = inPassthroughFrameRight.getFrame()
        rightFrame = cv2.cvtColor(passthroughFrameRight, cv2.COLOR_GRAY2BGR)

        inPassthroughFrameLeft = passthroughImageLeftQueue.get()
        passthroughFrameLeft = inPassthroughFrameLeft.getFrame()
        combineFrame = cv2.cvtColor(passthroughFrameLeft, cv2.COLOR_GRAY2BGR)

        try:
            trackedFeaturesLeft = outputFeaturesLeftQueue.get().trackedFeatures
            drawFeatures(leftFrame, componentDectector(trackedFeaturesLeft), show = False)

            trackedFeaturesRight = outputFeaturesRightQueue.get().trackedFeatures
            drawFeatures(rightFrame, transformation_inverse(componentDectector(transformation(trackedFeaturesRight))), show = False)

            drawFeatures(combineFrame, componentDectector(trackedFeaturesLeft + trackedFeaturesRight), show = True)
        except:
            pass

        # draw the detection zone
        cv2.line(combineFrame, (detection_intervale_x[0], detection_intervale_y[0]), (detection_intervale_x[0], detection_intervale_y[1]), color = (255, 0, 0), thickness = 1)
        cv2.line(combineFrame, (detection_intervale_x[1], detection_intervale_y[0]), (detection_intervale_x[1], detection_intervale_y[1]), color = (255, 0, 0), thickness = 1)
        cv2.line(combineFrame, (detection_intervale_x[0], detection_intervale_y[0]), (detection_intervale_x[1], detection_intervale_y[0]), color = (255, 0, 0), thickness = 1)
        cv2.line(combineFrame, (detection_intervale_x[0], detection_intervale_y[1]), (detection_intervale_x[1], detection_intervale_y[1]), color = (255, 0, 0), thickness = 1)
        cv2.putText(combineFrame, text = "Detection zone", org = (detection_intervale_x[0], detection_intervale_y[0]-3), fontFace= 1, fontScale = 1, color = (255, 0, 0), thickness = 1)


        # Show the frame
        #cv2.imshow(leftWindowName, leftFrame)
        #cv2.imshow(rightWindowName, rightFrame)
        cv2.imshow(combineWindowName, combineFrame)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        elif key == ord('s'):
            if featureTrackerConfig.cornerDetector.type == dai.FeatureTrackerConfig.CornerDetector.Type.HARRIS:
                featureTrackerConfig.cornerDetector.type = dai.FeatureTrackerConfig.CornerDetector.Type.SHI_THOMASI
                print("Switching to Shi-Thomasi")
            else:
                featureTrackerConfig.cornerDetector.type = dai.FeatureTrackerConfig.CornerDetector.Type.HARRIS
                print("Switching to Harris")

            cfg = dai.FeatureTrackerConfig()
            cfg.set(featureTrackerConfig)
            inputFeatureTrackerConfigQueue.send(cfg)