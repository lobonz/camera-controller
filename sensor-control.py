#!/usr/bin/env python3

#https://docs.luxonis.com/projects/api/en/latest/samples/SpatialDetection/spatial_mobilenet/#rgb-mobilenetssd-with-spatial-data

from pathlib import Path
# import sys
import cv2
import depthai as dai
import contextlib
import numpy as np
import time
import math
import os

from config import STRETCH, SHOWDEPTHMAP, SHOWPREVIEW, LABELMAP, OBJECTOFINTEREST, MODEL_PATH, MYDEVICES, PREVIEWSCALE, WALL, XFACTOR, PLANVIEW, SHOWPLANVIEW, SPATIALMERGETOLERANCE, WRITELOG

from Zonemanager import Zonemanager
zone = Zonemanager()
targetinfoPrevTime = time.perf_counter()
targetinfoHangTime = 3

relativetowall = True

if WRITELOG:
  log = open(os.path.join(os.path.dirname(__file__),'sensor.log'), 'w')

'''
Spatial detection network.
    Performs inference on RGB camera and retrieves spatial location coordinates: x,y,z relative to the center of depth map.
'''
def filled_circle(img, center, radius, color):
    thickness = -1
    line_type = 8
    cv2.circle(img,
               center,
               radius,
               color,
               thickness,
               line_type)

def circle(img, center, radius, color):
    thickness = 2
    line_type = 8
    cv2.circle(img,
               center,
               radius,
               color,
               thickness,
               line_type)

def phatText(img, text, location, color):
  #cv2.putText(img, text, location, cv2.FONT_HERSHEY_DUPLEX, 1, (0,255,255),4)
  cv2.putText(img, text, location, cv2.FONT_HERSHEY_DUPLEX, 1, color,1)

def plan_view(detections):
  #PLANVIEW
  #Create Black Empty Image
  size = PLANVIEW['h'], PLANVIEW['w'], 3 # last is the array 3rd drim so will likely be RGB I guess
  #print(size)
  plan_image = np.zeros(size, dtype=np.uint8)
  
  #Draw FOVs - {'cam': 1, 'offset': {'x': 1250, 'y': 600, 'z': 0}, 'type': 'S2', 'fov': 108}
  for cam in MYDEVICES:
    #print (MYDEVICES[cam])
    angle = math.radians(MYDEVICES[cam]['fov'] / 2)
    center = int(plan_image.shape[1] - (MYDEVICES[cam]['offset']['x']*PLANVIEW['scale'])) # 
    adjacent = plan_image.shape[0]
    oppositeSide = int(adjacent*math.tan(angle))

    fov_cnt = np.array([
        (center+oppositeSide,plan_image.shape[0]),
        (center, 0),
        (center-oppositeSide,plan_image.shape[0])
    ])
    cv2.fillPoly(plan_image, [fov_cnt], color=(70, 70, 70))

  #Draw Key
  keyx = 10
  keyy = 20
  keysize = 20
  keyspace = 5

  keyy = keyy + keysize + keyspace
  cv2.rectangle(plan_image, (keyx, keyy), (keyx+keysize, keyy+keysize), (255, 0, 0), -1)
  phatText(plan_image, 'Target Live', (keyx + keysize + keyspace , keyy + keysize), (255, 0, 0))

  keyy = keyy + keysize + keyspace
  cv2.rectangle(plan_image, (keyx, keyy), (keyx+keysize, keyy+keysize), (255, 0, 255), -1)
  phatText(plan_image, 'Target Found', (keyx + keysize + keyspace , keyy + keysize), (255, 0, 255))

  keyy = keyy + keysize + keyspace
  cv2.rectangle(plan_image, (keyx, keyy), (keyx+keysize, keyy+keysize), (0, 127, 255), -1)
  phatText(plan_image, 'Velocity', (keyx + keysize + keyspace , keyy + keysize), (0, 127, 255))

  keyy = keyy + keysize + keyspace
  cv2.rectangle(plan_image, (keyx, keyy), (keyx+keysize, keyy+keysize), (0, 255, 0), -1)
  phatText(plan_image, 'Detections', (keyx + keysize + keyspace , keyy + keysize), (0, 255, 0))

  keyy = keyy + keysize + keyspace
  cv2.rectangle(plan_image, (keyx, keyy), (keyx+keysize, keyy+keysize), (0, 0, 255), -1)
  phatText(plan_image, 'New', (keyx + keysize + keyspace , keyy + keysize), (0, 0, 255))
  
  keyy = keyy + keysize + keyspace
  cv2.rectangle(plan_image, (keyx, keyy), (keyx+keysize, keyy+keysize), (0, 255, 255), -1)
  phatText(plan_image, 'Kept', (keyx + keysize + keyspace , keyy + keysize), (0, 255, 255))

  keyy = keyy + keysize + keyspace
  cv2.rectangle(plan_image, (keyx, keyy), (keyx+keysize, keyy+keysize), (255, 0, 127), -1)
  phatText(plan_image, 'Merged', (keyx + keysize + keyspace , keyy + keysize), (255, 0, 127))
  
  keyy = keyy + keysize + keyspace
  cv2.rectangle(plan_image, (keyx, keyy), (keyx+keysize, keyy+keysize), (128, 128, 128), -1)
  phatText(plan_image, 'Deleted', (keyx + keysize + keyspace , keyy + keysize), (128, 128, 128))
  
  
  #Draw Detections
  for i, position in enumerate(detections):
    filled_circle( plan_image, (int(position['x']*PLANVIEW['scale']), int(position['z']*PLANVIEW['scale'])),int(PLANVIEW['h'] / 40),(0, 255, 0))
  
  #Draw Position Info
  for i, group in enumerate(zone.targetinfo):
    #print(zone.targetinfo[group])
    if len(zone.targetinfo[group]) > 0:
      #targetinfo = { 'kept': [], 'merged': [], 'deleted': [], 'new': []}
      for position in zone.targetinfo[group]:
        #print(position)
        if group == "new":
          circle( plan_image, (int(position['x']*PLANVIEW['scale']), int(position['z']*PLANVIEW['scale'])),int(PLANVIEW['h'] / 20),(0, 0, 255))
        if group == "kept":  
          filled_circle( plan_image, (int(position['x']*PLANVIEW['scale']), int(position['z']*PLANVIEW['scale'])),int(PLANVIEW['h'] / 120),(0, 255, 255))
          
        if group == "merged":  
          filled_circle( plan_image, (int(position['x']*PLANVIEW['scale']), int(position['z']*PLANVIEW['scale'])),int(PLANVIEW['h'] / 60),(255, 0, 127))
        if group == "deleted":
          circle( plan_image, (int(position['x']*PLANVIEW['scale']), int(position['z']*PLANVIEW['scale'])),int(PLANVIEW['h'] / 20),(128, 128, 128))      
  
  #Draw Targets
  for target in zone.targets:
    # print(target)
    tx = int(target.x*PLANVIEW['scale'])
    tz = int(target.z*PLANVIEW['scale'])
    radius = int(PLANVIEW['h'] / 60)
    targetColor = (255, 0, 255)
    if target.exists:
      targetColor = (255, 0, 0)    
    filled_circle( plan_image, (tx, tz),radius,targetColor )
    circle( plan_image, (tx, tz),int(SPATIALMERGETOLERANCE * PLANVIEW['scale']),targetColor)
    
    #velocity vector
    cv2.line(plan_image, (tx, tz), (int(tx + target.velocity['x']* PLANVIEW['scale']), int(tz + target.velocity['z']* PLANVIEW['scale'])), (0, 127, 255),2)
    
    if target.exists:
      cv2.putText(plan_image, target.id[:4], (tx+radius, tz), cv2.FONT_HERSHEY_DUPLEX, 1, (255,0,0),4)
      cv2.putText(plan_image, target.id[:4], (tx+radius, tz), cv2.FONT_HERSHEY_DUPLEX, 1, (0,255,255),1)
      
      #Draw Velocity
      # cv2.putText(plan_image, str(target.velocity['x'])[:4], (tx+radius, tz+4*radius), cv2.FONT_HERSHEY_DUPLEX, 1, (255,0,0),4)
      # cv2.putText(plan_image, str(target.velocity['x'])[:4], (tx+radius, tz+4*radius), cv2.FONT_HERSHEY_DUPLEX, 1, (0,255,255),1)
      #Note really interested in Y :)
      # cv2.putText(plan_image, str(target.velocity['z'])[:4], (tx+radius, tz+8*radius), cv2.FONT_HERSHEY_DUPLEX, 1, (255,0,0),4)
      # cv2.putText(plan_image, str(target.velocity['z'])[:4], (tx+radius, tz+8*radius), cv2.FONT_HERSHEY_DUPLEX, 1, (0,255,255),1)

  global targetinfoPrevTime
  global targetinfoHangTime
  
  # Reset our array of points after HangTime seconds so we dont get trails for ever but have time to see points visualised
  if time.perf_counter() - targetinfoPrevTime > targetinfoHangTime:
    zone.targetinfo = { 'kept': [], 'merged': [], 'deleted': [], 'new': []}
    targetinfoPrevTime = time.perf_counter()
  return plan_image

def getPipeline():
  # Get argument first
  nnBlobPath = str((Path(__file__).parent / Path(MODEL_PATH)).resolve().absolute())

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

  if STRETCH:
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

  # # Better handling for occlusions:
  # stereo.setLeftRightCheck(True)
  # # Closer-in minimum depth, disparity range is doubled:
  # stereo.setExtendedDisparity(False)
  # # Better accuracy for longer distance, fractional disparity 32-levels:
  # stereo.setSubpixel(False)

  #UNCOMMENT FOR WIDE CAMERAS
  # stereo.useHomographyRectification(True) # gives dimension x 2
  # stereo.useHomographyRectification(False) # gives dimension x 1.5
  #Told to use enableDistortionCorrection as its the newer API
  stereo.enableDistortionCorrection(True)

  spatialDetectionNetwork.setBlobPath(nnBlobPath)
  spatialDetectionNetwork.setConfidenceThreshold(0.75)
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
            'depthQueue': device.getOutputQueue(name="depth", maxSize=4, blocking=False),
            'fps': 0,
            'counter': 0,
            'startTime': time.perf_counter(),
            'dtStartTime': 0,
            'dt': 0
        }
    
    while True:
      detectedObjects = []
      for mxid, q in devices.items():
        #nned try accept here for when camera fails or is unplugged
        if q['detectionNNQueue'].has():
            
          #Calculate fps
          current_time = time.perf_counter()
          q['counter']+=1
          if (current_time - q['startTime']) > 1 :
              q['fps'] = q['counter'] / (current_time - q['startTime'])
              q['counter'] = 0
              q['startTime'] = current_time

          #Calculate delta time to update Targets
          q['dt'] = current_time - q['dtStartTime']
          q['dtStartTime'] = current_time
          

          inDetections = q['detectionNNQueue'].get().detections

          if SHOWPREVIEW:
            inPreview = q['previewQueue'].get().getCvFrame()
          
          if SHOWDEPTHMAP:
            depthFrame = q['depthQueue'].get().getFrame()
            depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
            depthFrameColor = cv2.equalizeHist(depthFrameColor)
            depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)
            depthFrameColor = cv2.flip(depthFrameColor, 1) # 0 flip vertical, 1 flip horizontal, -1 flip both
            depthheight = depthFrameColor.shape[0]
            depthwidth  = depthFrameColor.shape[1]
            

            if len(inDetections) != 0 and SHOWDEPTHMAP:
              boundingBoxMapping = q['xoutBoundingBoxDepthMapping'].get()
              roiDatas = boundingBoxMapping.getConfigData() # roi == region of interest

          if SHOWPREVIEW:
            height = inPreview.shape[0]
            width  = inPreview.shape[1]
            #need to flip and scale detections too
            inPreview = cv2.flip(inPreview, 1) # 0 flip vertical, 1 flip horizontal, -1 flip both
                
          #need index to extract only the required depth data
          index = 0

                          
          for detection in inDetections:
            if detection.label == OBJECTOFINTEREST:

              if WRITELOG:
                log.write(f"{detection.xmin} {detection.xmax} {detection.ymin} {detection.ymax} {detection.spatialCoordinates.x} {detection.spatialCoordinates.y} {detection.spatialCoordinates.z} {detection.confidence}\n")
                
              detected_z = int(detection.spatialCoordinates.z + MYDEVICES[mxid]['offset']['z'])
              detected_x = int(detection.spatialCoordinates.x*XFACTOR + MYDEVICES[mxid]['offset']['x'])
              detected_y = int(detection.spatialCoordinates.y + MYDEVICES[mxid]['offset']['y'])

              #Draw Frames on Depth Map for detected objects of interest i.e. people
              if SHOWDEPTHMAP:
                roiData = roiDatas[index]
                roi = roiData.roi
                roi = roi.denormalize(depthFrameColor.shape[1], depthFrameColor.shape[0])
                topLeft = roi.topLeft()
                bottomRight = roi.bottomRight()
                
                #invert X since we flipped the camera
                xmax = depthwidth - int(topLeft.x)
                ymin = int(topLeft.y)
                xmin = depthwidth - int(bottomRight.x)
                ymax = int(bottomRight.y)
                cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), 255, cv2.FONT_HERSHEY_SCRIPT_SIMPLEX)
                cv2.putText(depthFrameColor, f"Z: {int(detection.spatialCoordinates.z )} mm", (xmin + 10, ymin + 60), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255,0,0),3)
                cv2.putText(depthFrameColor, f"Z: {int(detection.spatialCoordinates.z )} mm", (xmin + 10, ymin + 60), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0,255,255),1)

              if SHOWPREVIEW:
                # Denormalize bounding box PLUS invert X since we flipped the camera
                x2 = width - int(detection.xmin * width)
                x1 = width - int(detection.xmax * width)
                y1 = int(detection.ymin * height)
                y2 = int(detection.ymax * height)
                try:
                    label = LABELMAP[detection.label]
                except:
                    label = detection.label

                cv2.putText(inPreview, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255,0,0),3)
                cv2.putText(inPreview, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0,255,255),1)

                cv2.putText(inPreview, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255,0,0),3)
                cv2.putText(inPreview, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0,255,255),1)

                if relativetowall:
                  cv2.putText(inPreview, f"X: {WALL['width'] - detected_x} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255,0,0),3)
                  cv2.putText(inPreview, f"X: {WALL['width'] - detected_x} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0,255,255),1)
                else:
                  cv2.putText(inPreview, f"X: {detected_x} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255,0,0),3)
                  cv2.putText(inPreview, f"X: {detected_x} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0,255,255),1)

                cv2.putText(inPreview, f"Y: {detected_y} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255,0,0),3)
                cv2.putText(inPreview, f"Y: {detected_y} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0,255,255),1)

                cv2.putText(inPreview, f"Z: {detected_z} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255,0,0),3)
                cv2.putText(inPreview, f"Z: {detected_z} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0,255,255),1)

                cv2.rectangle(inPreview, (x1, y1), (x2, y2), (255, 0, 0), cv2.FONT_HERSHEY_SIMPLEX)
                
                
              # CAPTURE POSITION DATA HERE
              # 'label'
              # 'confidence'
              # 'x_min' bounding box left
              # 'x_max' bounding box right
              # 'y_min' bounding box top
              # 'y_max' bounding box bottom

              
              #Limit appending based on wall limits
              if detected_z < WALL['zmax'] and detected_z > WALL['zmin']:
                #Limit appending based on camera limits.
                if detected_x < MYDEVICES[mxid]['xmax'] and detected_x > MYDEVICES[mxid]['xmin'] :
                  detectedObjects.append({
                      "cam": MYDEVICES[mxid]['cam'],
                      "x": WALL['width'] - detected_x, #reverse direction
                      "y": detected_y,
                      "z": detected_z,
                      "w": detection.xmax - detection.xmin,
                      "h": detection.ymax - detection.ymin,
                      "dt": q['dt']
                  })

              #VERSION BEFORE CAMERA X LIMITS
              # if myz < WALL['zmax'] and myz > WALL['zmin']:
              #   detectedObjects.append({
              #       "cam": MYDEVICES[mxid]['cam'],
              #       "x": WALL['width'] - int(detection.spatialCoordinates.x*XFACTOR + MYDEVICES[mxid]['offset']['x']),
              #       "y": int(detection.spatialCoordinates.y + MYDEVICES[mxid]['offset']['y']),
              #       "z": myz,
              #       "w": detection.xmax - detection.xmin,
              #       "h": detection.ymax - detection.ymin,
              #       "dt": q['dt']
              #   })

              # print(f"{int(detection.spatialCoordinates.x)} mm, {int(detection.spatialCoordinates.y)} mm, {int(detection.spatialCoordinates.z)} mm")
            index += index
          
          # Show the frame
          if SHOWDEPTHMAP:
            cv2.imshow(f"depth - CAM:{MYDEVICES[mxid]['cam']} ", depthFrameColor)
          if SHOWPREVIEW:
            
            cv2.putText(inPreview, f"CAM: {MYDEVICES[mxid]['cam']}", (2, 0 + 30), cv2.FONT_HERSHEY_DUPLEX, 0.75, (255,0,0),3)
            cv2.putText(inPreview, f"CAM: {MYDEVICES[mxid]['cam']}", (2, 0 + 30), cv2.FONT_HERSHEY_DUPLEX, 0.75, (0,255,255),1)

            cv2.putText(inPreview, f"XFACTOR: {round(XFACTOR,2)}", (2, 0 + 60), cv2.FONT_HERSHEY_DUPLEX, 0.75, (255,0,0),3)
            cv2.putText(inPreview, f"XFACTOR: {round(XFACTOR,2)}", (2, 0 + 60), cv2.FONT_HERSHEY_DUPLEX, 0.75, (0,255,255),1)
            
            cv2.putText(inPreview, f"RWALL: {relativetowall}", (int(width*0.4) , 0 + 30), cv2.FONT_HERSHEY_DUPLEX, 0.75, (255,0,0),3)
            cv2.putText(inPreview, f"RWALL: {relativetowall}", (int(width*0.4) , 0 + 30), cv2.FONT_HERSHEY_DUPLEX, 0.75, (0,255,255),1)


            cv2.putText(inPreview, "Targets: {:}".format(len(zone.targets)), (2, inPreview.shape[0] - 40), cv2.FONT_HERSHEY_DUPLEX, 0.75, (255,0,0),3)
            cv2.putText(inPreview, "Targets: {:}".format(len(zone.targets)), (2, inPreview.shape[0] - 40), cv2.FONT_HERSHEY_DUPLEX, 0.75, (0,255,255),1)

            # cv2.putText(inPreview, "DT: {:.6f}".format(q['dt']), (2, inPreview.shape[0] - 40), cv2.FONT_HERSHEY_DUPLEX, 0.75, (255,0,0),3)
            # cv2.putText(inPreview, "DT: {:.6f}".format(q['dt']), (2, inPreview.shape[0] - 40), cv2.FONT_HERSHEY_DUPLEX, 0.75, (0,255,255),1)

            cv2.putText(inPreview, "NN fps: {:.2f}".format(q['fps']), (2, inPreview.shape[0] - 10), cv2.FONT_HERSHEY_DUPLEX, 0.75, (255,0,0),3)
            cv2.putText(inPreview, "NN fps: {:.2f}".format(q['fps']), (2, inPreview.shape[0] - 10), cv2.FONT_HERSHEY_DUPLEX, 0.75, (0,255,255),1)

            #Add centre registrations
            cv2.line(inPreview, (int(inPreview.shape[0]/2), 0), (int(inPreview.shape[0]/2), inPreview.shape[1]), (0,255,255), 1)
            cv2.line(inPreview, (0, int(inPreview.shape[1]/2)), (inPreview.shape[0], int(inPreview.shape[1]/2)), (0,255,255), 1) 

            inPreview = cv2.resize(inPreview, (inPreview.shape[1]*PREVIEWSCALE, inPreview.shape[0]*PREVIEWSCALE))
            
            
            cv2.imshow(f"preview - CAM:{MYDEVICES[mxid]['cam']}", inPreview)
            

          
      
      # PROCESS DATA HERE & SEND TO SERVER
      zone.update(detectedObjects)

      #check for any Targets which may have timed out and send update to Hub.
      zone.sendUpdate()
      
      if SHOWPLANVIEW:
        cv2.imshow("Plan View", plan_view(detectedObjects))

      if cv2.waitKey(1) == ord('q'):
        if WRITELOG:
          log.close()
        break
      if cv2.waitKey(1) == ord('.'):
        XFACTOR = XFACTOR + .05
      if cv2.waitKey(1) == ord(','):
        XFACTOR = XFACTOR - .05
      if cv2.waitKey(1) == ord('w'):
        #toggle wall length calculation to flip position
        relativetowall = not relativetowall

