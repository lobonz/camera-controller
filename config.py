
# Ribbon directory board of 1500x1250mm on the far left to be run as an independent screen with resolution 810x675p.
# The remaining interactive screen of 9500x1250mm has a resolution of 5130x675p.
# Physically its 500mm flat next to directory board, 1000mm curve and then 8000mm flat length.

BNZ = { 'width': 11000, 'depth': 5000, 'zmin': 1500, 'zmax': 5000}

RD = { 'width': 2500, 'depth': 2500, 'zmin': 1500, 'zmax': 2500}
# RDENTRANCEWALLWIDTH = 2500
# RDENTRANCEDEPTH = 2500

WALL = BNZ
# WALLWIDTH = BNZWALLWIDTH
# WALLHEIGHT = BNZSPACEDEPTH

PLANVIEWSCALE = .1
PLANVIEW = { "scale": PLANVIEWSCALE, "w": int(WALL['width']*PLANVIEWSCALE), "h": int(WALL['depth']*PLANVIEWSCALE) }
#Object describing devices and offsets to apply to positions
OLDDEVICE = {
  "18443010B15AF40800": { "cam": 1, "offset": {"x":1250 ,"y":600, "z":0}, "type": "S2", "fov": 68.7938}
  }

STANDARDDEVICES = {
  "194430103182F51200": { "cam": 1, "offset": {"x":2000 ,"y":600, "z":0}, "type": "S2", "fov": 68.7938}, 
  "1944301031E0F41200": { "cam": 2, "offset": {"x":2000+1500 ,"y":600, "z":0}, "type": "S2", "fov": 68.7938}
  }

WIDEDEVICES = {
  "19443010F120F51200": { "cam": 1, "offset": {"x":1500 ,"y":1000, "z":0}, 'xmax':1500+(3350/2), 'xmin':0, "type": "S2Wide", "fov": 108 },
  "1944301041D3F51200": { "cam": 2, "offset": {"x":1500+3350 ,"y":1000, "z":0}, 'xmax':1500+3350+(3200/2), 'xmin':1500+3350-(3350/2), "type": "S2Wide", "fov": 108 },
  "1944301091F3F51200": { "cam": 3, "offset": {"x":1500+3350+3200 ,"y":1000, "z":0}, 'xmax':BNZ['width'], 'xmin':1500+3350+3200-(3200/2), "type": "S2Wide", "fov": 108 }
  }
WIDEDEVICESZERO = {
"19443010F120F51200": { "cam": 1, "offset": {"x":0 ,"y":1000, "z":0}, 'xmax':1500+(3350/2), 'xmin':0, "type": "S2Wide", "fov": 108 },
"1944301041D3F51200": { "cam": 2, "offset": {"x":0 ,"y":1000, "z":0}, 'xmax':1500+3350+(3200/2), 'xmin':1500+3350-(3350/2), "type": "S2Wide", "fov": 108 },
"1944301091F3F51200": { "cam": 3, "offset": {"x":0 ,"y":1000, "z":0}, 'xmax':BNZ['width'], 'xmin':1500+3350+3200-(3200/2), "type": "S2Wide", "fov": 108 }
}

XFACTOR_WIDE = .63
XFACTOR_STD = 1

MYDEVICES = WIDEDEVICES
XFACTOR = XFACTOR_WIDE # measurement scaling factor to correct X Measurements

PREVIEWSCALE = 1
STRETCH = False # set to false for Letterbox

SHOWDEPTHMAP = False #show the depth Map
SHOWPREVIEW = True #show the preview image
SHOWPLANVIEW = True

# MobilenetSSD label texts
LABELMAP = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]
OBJECTOFINTEREST = 15 # person

MODEL_NAME = "mobilenet-ssd"
MODEL_PATH = "./models/mobilenet-ssd_openvino_2021.4_6shave.blob" # 300x300
LABEL_MAP = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]
##########
# If you change the model above you need to edit depthai_utils.py and main.py too match the required frame size for the NN

TIMEDOUTTIME = 4 #how long without an update before we delete a target
NOTTRACKEDTIME = 3 #how long till we consider it not tracked anymore - used to send nottracked to Unreal
EXISTSTRACKEDTIME = 3 #how long to exist before we treat as live

SPATIALMERGETOLERANCE = 800

DEBUG = True