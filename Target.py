import uuid
import time
from config import TIMEDOUTTIME, NOTTRACKEDTIME, EXISTSTRACKEDTIME

class Target:
  def __init__(self, position):
    #{'mxid': '194430103182F51200', 'x': 141, 'y': -156, 'z': 947, 'w': 0.625, 'h': 0.597900390625, 'dt': 0.03091109999999997}
    print("__INIT__")
    print(position)
    self.id = str(uuid.uuid4())
    self.x = position['x']
    self.y = position['y']
    self.z = position['z']
    self.w = position['w']
    self.h = position['h']

    self.velocity = {}
    self.velocity['x'] = 0
    self.velocity['y'] = 0
    self.velocity['z'] = 0
    
    self.history = {}
    self.history['x'] = [position['x']]
    self.history['y'] = [position['y']]
    self.history['z'] = [position['z']]
    self.history['samples'] = 5 # moving average samples

    self.status = "tracked"
    self.lastupdate = time.perf_counter()
    self.inittime = time.perf_counter()
    self.exists = False
  def live(self):
    if self.exists: return self.exists
    #we are only live if we have been around long enough
    if self.inittime < time.perf_counter() - EXISTSTRACKEDTIME and self.status == "tracked":
      self.exists = True
      return True
    else:
      return False

  def timedOut(self):
    if self.lastupdate < time.perf_counter() - TIMEDOUTTIME:
      self.status = "dead"
      return True
    else:
      if self.lastupdate < time.perf_counter() - NOTTRACKEDTIME:
        self.status = "nottracked"
      return False

  def update(self, position):
    #set status every positional update
    self.status = "tracked"
    self.lastupdate = time.perf_counter()
    
    #need to update velocity but we should never apply except for the predicted Position
    self.history['x'].append(position['x'])
    self.history['y'].append(position['y'])
    self.history['z'].append(position['z'])

    #remove the last entry if our lists are longer than required smaples
    if len(self.history['x']) > self.history['samples']: #all are the same length so we only need to check x
      self.history['x'].pop(0)
      self.history['y'].pop(0)
      self.history['z'].pop(0)

    lastx = self.x
    lasty = self.y
    lastz = self.z

    #update positions
    self.x = self.ma(self.history['x'])
    self.y = self.ma(self.history['y'])
    self.z = self.ma(self.history['z'])

    self.w = position['w']
    self.h = position['h']

    self.velocity['x'] = (self.x - lastx)/position['dt']
    self.velocity['y'] = (self.y - lasty)/position['dt']
    self.velocity['z'] = (self.z - lastz)/position['dt']


  def predictedPosition(self):
    #add some magic to predict next position
    # self.x += self.velocity_x # * dt
    # self.y += self.velocity_y # * dt
    # self.z += self.velocity_y # * dt
    return (self.x, self.y, self.z)

  def ma(self, a): #Moving Average
    sum=0
    for i in a:
        sum += i
    return sum/len(a)
