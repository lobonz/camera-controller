from config import MYDEVICES, SPATIALMERGETOLERANCE, SHOWPLANVIEW

class Zonemanager:
  # Set up Server Client
  def messageReceived(self, msg):
      print(msg)

  def __init__(self):
    from Serverclient import Serverclient
    self.hub = Serverclient('127.0.0.1', 1337)
    self.hub.subscribe('interactive-screen-channel', self.messageReceived)

    #Track the people using a list of updatable targets that will persist through missing data representing the position of people.
    self.targets = []
    self.targetinfo = { 'kept': [], 'merged': [], 'deleted': [], 'new': []}
    
  def distance3d(self,x1,y1,z1, x2,y2,z2): return ((x2-x1)**2+(y2-y1)**2+(z2-z1)**2)**0.5
  def averagePositions(self,pos1,pos2):
    return {'x': (pos1['x']+pos2['x'])/2, 'y': (pos1['y']+pos2['y'])/2, 'z': (pos1['z']+pos2['z'])/2, 'w': (pos1['w']+pos2['w'])/2, 'h': (pos1['h']+pos2['h'])/2, 'dt': (pos1['dt']+pos2['dt'])/2}
  #
  # # # # #   U P D A T E   # # # # #
  #
  def update(self, positions):
    mergedpositions = []

    #IF WE HAVE POSITIONS PROCESS THEM
    if len(positions) > 0:
      from Target import Target
      #TODO: NEED TO ONLY MERGE FROM DIFFERENT CAMERAS

      # Merge points with our SPATIALMERGETOLERANCE and average them
      # loop through positions
      if len(positions) > 1:
        positionIsMerged = [False] * len(positions) #Initialise mergedpositions array with values of False for each position

        #Loop Through Positions Merging Points that are within the tolerance
        for i in range(len(positions)): # 123456
          #Check each position against the others one by one
          #each time around the loop we only check those we
          #havent checked before by reducing the range we check against.
          #Any points within the SPATIALMERGETOLERANCE we average to get a final position
          thisPosition = positions[i]
          for j in range ( i+1, len(positions)):
            # print("distance3d")
            # print(positions[i]['x'], positions[i]['y'], positions[i]['z'] , positions[j]['x'], positions[j]['y'], positions[j]['z'])
            disttopoint = self.distance3d(positions[i]['x'], positions[i]['y'], positions[i]['z'] , positions[j]['x'], positions[j]['y'], positions[j]['z'])
            # print(disttopoint)
            if disttopoint < SPATIALMERGETOLERANCE:
              averagePosition = self.averagePositions(thisPosition, positions[j])
              positionIsMerged[i] = True
              positionIsMerged[j] = True
              mergedpositions.append(averagePosition)

              #Add the values we keep to the plan view
              if SHOWPLANVIEW and positionIsMerged[i] != True:
                self.targetinfo['merged'].append(positions[i])
              if SHOWPLANVIEW:
                self.targetinfo['merged'].append(positions[j])

          #If not merged then add the original position to the merged positions
          if not positionIsMerged[i]:
            mergedpositions.append(positions[i])

          #Add the values we keep to the plan view
          if SHOWPLANVIEW:
              self.targetinfo['kept'].append(positions[i])
      
      if len(positions) == 1: #only 1 position
        mergedpositions = positions
        if SHOWPLANVIEW:
              self.targetinfo['kept'].append(positions[0])
      
      #update targets
      positionswithoutatarget = {}
      targetswithaposition = [False] * len(self.targets)
      # find closest target and update with this position.
      # loop through mergedpositions
      for i, position in enumerate(mergedpositions):
          closesttarget = -1
          #TODO use this value to set a max distance we would consider ok to match to.
          closestdistance = 3000 #stupid large value bigger than our screen
          positionswithoutatarget[i] = False
          #loop through targets to find the closest one
          for j, target in enumerate(self.targets):
              if targetswithaposition[j] != True:
                  targetX, targetY, targetZ = target.predictedPosition()
                  disttotarget = self.distance3d(position['x'], position['y'], position['z'] , targetX, targetY, targetZ)
                  if disttotarget < closestdistance:
                      closesttarget = j
                      closestdistance = disttotarget
          #update closest target with position
          if closesttarget != -1: # we have a target for this position
              #Update target position and size
              self.targets[closesttarget].update(position)
              targetswithaposition[closesttarget] = True
          else:
              positionswithoutatarget[i] = True

      # if no targets make a new one.
      # for key, value in dictionary.items():
      for i, withoutatarget in positionswithoutatarget.items():
        #print ("positionswithoutatarget " + str(i) + " = " + str(withoutatarget) )
        if withoutatarget:
          #print ("no target")
          newtarget = Target(mergedpositions[i])
          self.targets.append(newtarget)
          if SHOWPLANVIEW:
              self.targetinfo['new'].append(mergedpositions[i])
      #For Targets we have no position update them with known velocity
      for i, targethasposition in enumerate(targetswithaposition):
        if not targethasposition:
          target.updatePosition()
    #WE have no positions so update them with known velocity
    else:
      #update positions based on velocity
      for j, target in enumerate(self.targets):
        target.updatePosition()

  #Send data to the server
  def sendUpdate(self):
    positions = [] # positions to send to the Hub

    
    i = len(self.targets) - 1
    for target in self.targets[::-1]:
      # if too many targets remove when the position is not updated within a few seconds.
      if target.timedOut():
        if SHOWPLANVIEW:
            self.targetinfo['deleted'].append({"x": self.targets[i].x, "y":self.targets[i].y, "z":self.targets[i].z, "w":self.targets[i].w, "h":self.targets[i].h})
        self.targets.pop(i)
        i = i - 1
        #send dead to hub when a target dies
        position = {
            "uuid": target.id,
            "status": "dead"
        }
        positions.append(position)
      
      elif target.live():
        position = {
            "uuid": target.id,
            "x": target.x,
            "y": target.y,
            "z": target.z,
            "w": target.w,
            "h": target.h,
            "status": target.status
        }
        positions.append(position)
    if len(positions) > 0:
      self.hub.publish(positions)

    # FORMAT
    # id, x, y, z, w, h, status
    # [
    #     {
    #     "id":"1",
    #     "status":"dead"
    #     },
    #     {
    #     "id":"2",
    #     "z":2909,
    #     "h":1780,
    #     "w":1435,
    #     "x":4531,
    #     "y":340,
    #     "status": "tracked"
    #     },
    #     {
    #     "id":"3",
    #     "z":2909,
    #     "h":1780,
    #     "w":1435,
    #     "x":4531,
    #     "y":340,
    #     "status": "nottracked"
    #     }
    # ]
    # position = object['depth_x'], object['depth_y'], object['depth_z']