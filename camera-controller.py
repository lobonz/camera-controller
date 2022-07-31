#!/usr/bin/env python3

from pathlib import Path
# import sys
import cv2
import depthai as dai
import contextlib
# import numpy as np
import time

mydevices = { 1: { id: "194430103182F51200"}, 2: { id: "1944301031E0F41200"} }

stretch = True # set to false for Letterbox

showDepthMap = False #show the depth Map
showPreview = True #show the preview image

# MobilenetSSD label texts
labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]
objectOfIntrest = 15 # person

'''
Spatial detection network demo.
    Performs inference on RGB camera and retrieves spatial location coordinates: x,y,z relative to the center of depth map.
'''

def getPipeline():
  # Get argument first
  nnBlobPath = str((Path(__file__).parent / Path('./models/mobilenet-ssd_openvino_2021.4_6shave.blob')).resolve().absolute())

  # Create pipeline
  pipeline = dai.Pipeline()

  # Define sources and outputs
  camRgb = pipeline.create(dai.node.ColorCamera)
  spatialDetectionNetwork = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
  monoLeft = pipeline.create(dai.node.MonoCamera)
  monoRight = pipeline.create(dai.node.MonoCamera)
  stereo = pipeline.create(dai.node.StereoDepth)

  xoutRgb = pipeline.create(dai.node.XLinkOut)
  xoutNN = pipeline.create(dai.node.XLinkOut)
  xoutBoundingBoxDepthMapping = pipeline.create(dai.node.XLinkOut)
  xoutDepth = pipeline.create(dai.node.XLinkOut)

  xoutRgb.setStreamName("rgb")
  xoutNN.setStreamName("detections")
  xoutBoundingBoxDepthMapping.setStreamName("boundingBoxDepthMapping")
  xoutDepth.setStreamName("depth")

  # Properties
  camRgb.setIspScale(1,5) #THE_12_MP 4056x3040 -> 812x608 #THE_1080_P 1920x1080 -> 384, 216
  camRgb.setPreviewSize(384, 216) 
  camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
  camRgb.setInterleaved(False)
  camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

  monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
  monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
  monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
  monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

  #########################
  # Use ImageManip to resize to 300x300 by steching
  manip = pipeline.createImageManip() # manip = pipeline.create(dai.node.ImageManip)

  manip.setMaxOutputFrameSize(270000) # 300x300x3

  if stretch:
      manip.initialConfig.setResize(300,300)
      manip.initialConfig.setKeepAspectRatio(False) # Stretching the image
  else:
      manip.initialConfig.setResizeThumbnail(300, 300)

  camRgb.preview.link(manip.inputImage)
  ##########################

  # Setting node configs
  stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
  # Align depth map to the perspective of RGB camera, on which inference is done
  stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
  stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())

  spatialDetectionNetwork.setBlobPath(nnBlobPath)
  spatialDetectionNetwork.setConfidenceThreshold(0.5)
  spatialDetectionNetwork.input.setBlocking(False)
  spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
  spatialDetectionNetwork.setDepthLowerThreshold(100)
  spatialDetectionNetwork.setDepthUpperThreshold(5000)

  # Linking
  monoLeft.out.link(stereo.left)
  monoRight.out.link(stereo.right)

  #######################
  manip.out.link(spatialDetectionNetwork.input)
  #######################


  spatialDetectionNetwork.passthrough.link(xoutRgb.input)

  spatialDetectionNetwork.out.link(xoutNN.input)
  spatialDetectionNetwork.boundingBoxMapping.link(xoutBoundingBoxDepthMapping.input)

  stereo.depth.link(spatialDetectionNetwork.inputDepth)
  spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)
  return pipeline


# https://docs.python.org/3/library/contextlib.html#contextlib.ExitStack
with contextlib.ExitStack() as stack:
    device_infos = dai.Device.getAllAvailableDevices()
    number_of_devices = len(device_infos)
    if number_of_devices == 0:
        raise RuntimeError("No devices found!")
    else:
        print("Found", number_of_devices, "devices")
    devices = {}

    for device_info in device_infos:
        # Note: the pipeline isn't set here, as we don't know yet what device it is.
        # The extra arguments passed are required by the existing overload variants
        openvino_version = dai.OpenVINO.Version.VERSION_2021_4
        usb2_mode = False
        device = stack.enter_context(dai.Device(openvino_version, device_info, usb2_mode))

        # Note: currently on POE, DeviceInfo.getMxId() and Device.getMxId() are different!
        print("=== Connected to " + device_info.getMxId())
        mxid = device.getMxId()
        cameras = device.getConnectedCameras()
        usb_speed = device.getUsbSpeed()

        # Get a customized pipeline based on identified device type
        pipeline = getPipeline()
        device.startPipeline(pipeline)

        # Output queue will be used to get the rgb frames from the output defined above
        devices[mxid] = {
            'previewQueue': device.getOutputQueue(name="rgb", maxSize=4, blocking=False),
            'detectionNNQueue': device.getOutputQueue(name="detections", maxSize=4, blocking=False),
            'xoutBoundingBoxDepthMapping': device.getOutputQueue(name="boundingBoxDepthMapping", maxSize=4, blocking=False),
            'depthQueue': device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        }

    startTime = time.monotonic()
    counter = 0
    fps = 0
    
    while True:
      counter+=1
      current_time = time.monotonic()
      print(counter)
      if (current_time - startTime) > 1 :
          print("current_time = " + str(current_time))
          print("startTime    = " + str(startTime))
          fps = counter / (current_time - startTime)
          counter = 0
          startTime = current_time
          
      
      for mxid, q in devices.items():
          if q['detectionNNQueue'].has():
              
              inDetections = q['detectionNNQueue'].get().detections

              if showPreview:
                inPreview = q['previewQueue'].get().getCvFrame()
              
              if showDepthMap:
                depthFrame = q['depthQueue'].get().getFrame()
                depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
                depthFrameColor = cv2.equalizeHist(depthFrameColor)
                depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

                if len(inDetections) != 0 and showDepthMap:
                  boundingBoxMapping = q['xoutBoundingBoxDepthMapping'].get()
                  roiDatas = boundingBoxMapping.getConfigData() # roi == region of interest

              if showPreview:
                height = inPreview.shape[0]
                width  = inPreview.shape[1]
                
              #need index to extract only the required depth data
              index = 0
              for detection in inDetections:
                if detection.label == objectOfIntrest:
                  #Draw Frames on Depth Map for detected objects of interest i.e. people
                  if showDepthMap:
                    roiData = roiDatas[index]
                    roi = roiData.roi
                    roi = roi.denormalize(depthFrameColor.shape[1], depthFrameColor.shape[0])
                    topLeft = roi.topLeft()
                    bottomRight = roi.bottomRight()
                    xmin = int(topLeft.x)
                    ymin = int(topLeft.y)
                    xmax = int(bottomRight.x)
                    ymax = int(bottomRight.y)
                    cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), 255, cv2.FONT_HERSHEY_SCRIPT_SIMPLEX)
                      
                  if showPreview:
                    # Denormalize bounding box
                    x1 = int(detection.xmin * width)
                    x2 = int(detection.xmax * width)
                    y1 = int(detection.ymin * height)
                    y2 = int(detection.ymax * height)
                    try:
                        label = labelMap[detection.label]
                    except:
                        label = detection.label
                    cv2.putText(inPreview, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                    cv2.putText(inPreview, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                    cv2.putText(inPreview, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                    cv2.putText(inPreview, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                    cv2.putText(inPreview, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

                    cv2.rectangle(inPreview, (x1, y1), (x2, y2), (255, 0, 0), cv2.FONT_HERSHEY_SIMPLEX)
                    
                  # CAPTURE POSITION DATA HERE
                  # print(f"{int(detection.spatialCoordinates.x)} mm, {int(detection.spatialCoordinates.y)} mm, {int(detection.spatialCoordinates.z)} mm")
                index += index

              # Show the frame
              if showDepthMap:
                cv2.imshow(f"depth- {mxid} ", depthFrameColor)
              if showPreview:
                cv2.putText(inPreview, "NN fps: {:.2f}".format(fps), (2, inPreview.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255,255,255))
                cv2.imshow(f"preview - {mxid}", inPreview)
      
      # PROCESS DATA HERE
      # SEND TO SERVER

      if cv2.waitKey(1) == ord('q'):
          break