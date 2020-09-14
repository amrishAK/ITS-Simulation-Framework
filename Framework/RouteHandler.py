
from global_route_planner import GlobalRoutePlanner
from global_route_planner_dao import GlobalRoutePlannerDAO
from RoadOption import RoadOption

from misc import distance_vehicle
from collections import deque
from Container import Container
from CarlaVizUtil import CarlaPainter
from FrameworkException import StopException

class RouteHandler(object):
    painter = CarlaPainter('localhost',8089)

    def __init__(self):
        
        self.map = None
        self.globalRoutePlanner : GlobalRoutePlanner = None

        #route info
        self.startWaypoint = None
        self.endWaypoint = None
        self.currentWaypoint = None
        self.targetWaypoint = None
        self.targetRoadOption = None
        self.minimumDistance = 10 #in meters
        self.direction = None
        self.incomingDirection = None
        self.incomingWaypoint = None
        
        self.nextWaypoint = None
        self.nextRoadOption = None

        #route queue and buffer
        self.waypointQueue = deque(maxlen=20000)
        self.bufferSize = 5
        self.waypointBuffer = deque(maxlen=self.bufferSize)

    def SetupRouteManager(self,currentLocation,endLocation,resolution = 4.5):
        self.map = Container().GetMap()
        self.samplingResolution = resolution
        
        #setup the planner
        dao = GlobalRoutePlannerDAO(self.map,resolution)
        self.globalRoutePlanner  = GlobalRoutePlanner(dao= dao)
        self.globalRoutePlanner.setup()

        #set current wayPoint
        self.currentWaypoint =  self.map.get_waypoint(currentLocation)

        #create route
        self.CreateRoute(currentLocation,endLocation)

    def CreateRoute(self,startLocation,endLocation=None): 
        self.startWaypoint = self.map.get_waypoint(startLocation)
        
        if endLocation is not None:
            self.endWaypoint = self.map.get_waypoint(endLocation) 
        
        #generate route plan
        route = self.globalRoutePlanner.trace_route(
            self.startWaypoint.transform.location,
            self.endWaypoint.transform.location)

        #load route to queue
        self.LoadWaypointQueue(route)

    def LoadWaypointQueue(self,route):        
        #clean the queue
        self.waypointQueue.clear()
        lines = []

        #load route to the queue
        for element in route:
            self.waypointQueue.append(element)
            lines.append([element[0].transform.location.x,element[0].transform.location.y,element[0].transform.location.z])

        self.painter.draw_polylines(lines)
        #load the buffer
        self.waypointBuffer.clear()

    def LoadBuffer(self):
        #check if the buffer is empty and load
        if not self.waypointBuffer:
            for _ in range(self.bufferSize):
                if self.waypointQueue:
                    self.waypointBuffer.append(self.waypointQueue.popleft())
                else:
                    break

    def get_incoming_waypoint_and_direction(self, steps=3):
        """
        Returns direction and waypoint at a distance ahead defined by the user.

            :param steps: number of steps to get the incoming waypoint.
        """
        waypoint = None
        roadOption = RoadOption.VOID

        if len(self.waypointQueue) > steps:
            waypoint, roadOption = self.waypointQueue[steps]
        else:
            try:
                waypoint, waypoint = self.waypointQueue[-1]
            except IndexError:
                pass
        
        return waypoint, roadOption

    def Update(self):
        speedLimit = Container().GetActor().get_speed_limit()
        self.direction = self.targetRoadOption
        if self.direction is None:
            self.direction = RoadOption.LANEFOLLOW

        if len(self.waypointBuffer) > 0:
            self.nextWaypoint,self.nextRoadOption = self.waypointBuffer[0]
        else:
            self.nextWaypoint,self.nextRoadOption = self.waypointQueue[0]

        self.look_ahead_steps = int((speedLimit) / 10)

        self.incomingWaypoint, self.incomingDirection = self.get_incoming_waypoint_and_direction(
            steps=self.look_ahead_steps)
        if self.incomingDirection is None:
            self.incomingDirection = RoadOption.LANEFOLLOW


    def SetupRun(self):

        if len(self.waypointQueue) == 0:
            raise StopException

        #check and load the buffer
        self.LoadBuffer()

        vehicle = Container().GetActor()

        #get current and target waypoint
        self.currentWaypoint = self.map.get_waypoint(vehicle.get_location())
        self.targetWaypoint, self.targetRoadOption = self.waypointBuffer[0]

        #update waypoint queue
        max_index = -1
        vehicleTransform = vehicle.get_transform()

        for i, (waypoint, _) in enumerate(self.waypointBuffer):
            if distance_vehicle(waypoint, vehicleTransform) < self.minimumDistance:
                max_index = i
        if max_index >= 0:
            for i in range(max_index + 1):
                self.waypointBuffer.popleft()

        return self.targetWaypoint
