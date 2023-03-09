import uuid
import time
from config import TIMEDOUTTIME, NOTTRACKEDTIME, EXISTSTRACKEDTIME, MAXVELOCITY, DECELERATION

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
    self.history['vx'] = [0]
    self.history['vy'] = [0]
    self.history['vz'] = [0]
    self.history['samples'] = 5 # moving average samples
    self.history['vsamples'] = 15 # moving average samples

    self.status = "tracked"
    self.lastupdatetime = time.perf_counter()
    self.lastupdatetimedata = time.perf_counter() # last time updated from data not estimated
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
    if self.lastupdatetimedata < time.perf_counter() - TIMEDOUTTIME:
      self.status = "dead"
      return True
    else:
      if self.lastupdatetimedata < time.perf_counter() - NOTTRACKEDTIME:
        self.status = "nottracked"
      return False

  def update(self, position):
    #set status every positional update
    self.status = "tracked"
    thistime = time.perf_counter()
    
    #position
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

    #update positions MAs
    self.x = self.ma(self.history['x'])
    self.y = self.ma(self.history['y'])
    self.z = self.ma(self.history['z'])

    #Throw the width and height in we dont do anything clever with these yet.
    self.w = position['w']
    self.h = position['h']

    #velocity
    #NOTE this dt is not the same as the dt in the position information
    #     this dt is our change in time, the other dt is the % of detection certainty
    dt = thistime - self.lastupdatetime
    
    self.history['vx'].append(self.clamp((self.x - lastx)/dt, -MAXVELOCITY, MAXVELOCITY))
    self.history['vy'].append(self.clamp((self.y - lasty)/dt, -MAXVELOCITY, MAXVELOCITY))
    self.history['vz'].append(self.clamp((self.z - lastz)/dt, -MAXVELOCITY, MAXVELOCITY))

    #remove the last entry if our lists are longer than required smaples
    if len(self.history['vx']) > self.history['vsamples']: #all are the same length so we only need to check x
      self.history['vx'].pop(0)
      self.history['vy'].pop(0)
      self.history['vz'].pop(0)

    #update velocity MAs
    self.velocity['x'] = self.ma(self.history['vx'])
    self.velocity['y'] = self.ma(self.history['vy'])
    self.velocity['z'] = self.ma(self.history['vz'])

    self.lastupdatetime = thistime
    self.lastupdatetimedata = thistime

  def updatePosition(self):
    #update our position based on prediction - when we have no data coming in.
    self.x, self.y, self.z = self.predictedPosition()
    dt = time.perf_counter() - self.lastupdatetime
    self.lastupdatetime = time.perf_counter()

    #Apply decelleration
    self.velocity['x'] = self.clamp(self.velocity['x'] - DECELERATION*dt, -MAXVELOCITY, MAXVELOCITY)
    self.velocity['y'] = self.clamp(self.velocity['y'] - DECELERATION*dt, -MAXVELOCITY, MAXVELOCITY)
    self.velocity['z'] = self.clamp(self.velocity['z'] - DECELERATION*dt, -MAXVELOCITY, MAXVELOCITY)
    #reset history
    self.history['vx'] = [self.velocity['x']]
    self.history['vy'] = [self.velocity['y']]
    self.history['vz'] = [self.velocity['z']]
    

  def predictedPosition(self):
    #predict next position based on velocity
    dt = time.perf_counter() - self.lastupdatetime
    return (self.x + self.velocity['x'] * dt, self.y + self.velocity['y'] * dt, self.z + self.velocity['z'] * dt)

  def ma(self, a): #Moving Average
    sum=0
    for i in a:
        sum += i
    return sum/len(a)

  def clamp(self, num, min_value, max_value):
   return max(min(num, max_value), min_value)