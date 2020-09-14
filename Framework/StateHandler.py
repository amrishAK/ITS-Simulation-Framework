from misc import get_speed
from Container import Container

class RSUStateHandler(object):
    CommunicationRange = 50 #in meters

    #message data
    CurrentMsg = {}
    MessageDataStore = {} 

    def Setup(self):
        for key in Container().GetRxHandlersList():
            self.CurrentMsg[key] = {}
            self.MessageDataStore[key] = {}

class VehicleStateHandler(object):  
    # Vehicle information
    vehicleId = None
    speed = 0
    minSpeed = 5
    speedLimit = 0
    startWaypoint = None
    endWaypoint = None
    CurrentLocation = None
    
    #communication
    CommunicationRange = 50 #in meters
    
    #traffic light state
    isAtTrafficLight = 0
    lightState = "Green"
    light_id_to_ignore = -1

    #message data
    CurrentMsg = {}
    MessageDataStore = {} 

    def Setup(self):
        for key in Container().GetRxHandlersList():
            self.CurrentMsg[key] = {}
            self.MessageDataStore[key] = {}
        

    def SetVehicleId(self,id):
        self.vehicleId = id

    def Update(self):
        vehicle = Container().GetActor()
        self.speed = get_speed(vehicle)
        self.speedLimit = vehicle.get_speed_limit()
        self.Currentlocation = vehicle.get_location()
