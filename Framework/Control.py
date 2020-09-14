import carla
import random
import time
import pygame

from abc import ABC,abstractmethod, abstractproperty

from RouteHandler import RouteHandler
from PIDHandler import PIDHandler
from BehaviourAgent import BaseVehicleAgent, BasicVehicleAgent
from StateHandler import VehicleStateHandler, RSUStateHandler
from Container import Container
from CarlaVizUtil import CarlaPainter
from misc import get_speed


class BaseControl(ABC):
    _Agent = None
    _LoopControl = True
    _Configuration = None
    def __init__(self):
        self._Configuration = Container().GetConfig()
        self.MainSetup()

    @abstractmethod
    def Loop(self):
        raise NotImplementedError

    def Run(self):
        world = Container().GetWorld()
        try:
            while self._LoopControl:
                if self._Configuration["AsyncMode"] is False:
                    if not world.wait_for_tick(10.0):
                        continue
                else:
                    world.tick()

                self.Loop()
        except Exception as ex:
            print(ex)
        finally:
            self.Close()
            return

    @abstractmethod
    def MainSetup(self):
        raise NotImplementedError

    @abstractmethod
    def Close(self):
        raise NotImplementedError

    def GetSpawnPoints(self,forSpawning = True):

        try:
            spawnPoints = Container().GetSpawnPoints()
    
            spawnConfig = self._Configuration["ActorSpawning" if forSpawning else "ActorDestination"]
            if spawnConfig["Random"]:
                return random.choice(spawnPoints)
            
            else:

                givenLocation = spawnConfig["SpawnLocation"]

                if len(givenLocation) == 0:
                        print("Location not given assigning a random location")
                        return random.choice(spawnPoints)

                def getLocation ():
                    
                    pickedLocation = None

                    if len(givenLocation) == 1:
                        pickedLocation = givenLocation[0]
                    else:
                        pickedLocation = random.choice(givenLocation)
                
                    return carla.Location(x=pickedLocation[0], y=pickedLocation[1], z=pickedLocation[2])
                
                def isInRange(target,actual) :  return target.distance(actual.location) <= 30
                
                InRangePoints = []
                
                for point in spawnPoints:
                    if isInRange(getLocation(),point):
                        InRangePoints.append(point)

                return random.choice(InRangePoints)
        except Exception as ex:
            print(f"In GSP{ex}")

class VehicleControl (BaseControl):    
    __Controller = PIDHandler()
    __RouteManager = RouteHandler()

    def SetupVehicle(self):
        world = Container().GetWorld()
        vehicle_bp = random.choice(world.get_blueprint_library().filter('vehicle.audi.*'))
        port = self._Configuration["Port"]
        role_name = f"ego1:localhost:{port}"
        vehicle_bp.set_attribute('role_name',role_name)

        while True:
            vehicleTransform  = self.GetSpawnPoints()
            vehicle = world.spawn_actor(vehicle_bp,vehicleTransform)

            if vehicle is not None:
                if self._Configuration["AsyncMode"]:
                    Container().GetWorld().tick()
                else:
                    time.sleep(10)

                Container().GetState().SetVehicleId(vehicle.id)
                print(f"Vehicle spawned at {vehicle.get_transform()}")
                Container().SetActor(vehicle)
                break

    def SetupRoute(self):
        print("In route")
        spawn_points = Container().GetSpawnPoints()
        random.shuffle(spawn_points)
        print(Container().GetActor().get_location())
        currentLocation = Container().GetActor().get_location()
        destination = self.GetSpawnPoints(False).location
        self.__RouteManager.SetupRouteManager(currentLocation,destination)
        print("In route")

        #add to the container
        Container().SetRouteManager(self.__RouteManager)

    def SetupStateHandler(self):
        state = VehicleStateHandler()
        state.Setup()
        Container().SetState(state)      

    def SetupAgent(self):
        self._Agent = Container().GetAgent()
        if self._Agent is None:
            self._Agent = BasicVehicleAgent(Container())
        self._Agent.SetupAgent()

    def MainSetup(self):

        self.SetupStateHandler()
        self.SetupVehicle()
        self.SetupRoute()
        self.SetupAgent()
        print("In loop")

        self.__Controller.SetupController()
        self.Run()
       

    def Close(self):
        if self._LoopControl:
            self._LoopControl = False
            Container().GetActor().destroy()

    def Loop(self):
        #update state
        control = None
        try:
            #update state
            Container().GetState().Update()

            #update route
            self.__RouteManager.Update()

            #motion plan
            targetSpeed = self._Agent.RunStep()

            targetWaypoint = self.__RouteManager.SetupRun()
            
            targetSpeed = Container().GetActor().get_speed_limit() if targetSpeed is None else targetSpeed        
            #control
            control = self.__Controller.RunStep(Container().GetActor(),targetWaypoint, targetSpeed)

        except Exception as ex:
            print(ex)
            control = carla.VehicleControl()
            control.steer = 0.0
            control.throttle = 0.0
            control.brake = 1.0
            control.hand_brake = False
            
        Container().GetActor().apply_control(control)
    

class RSUControl(BaseControl):
    

    def SetupStateHandler(self):
        state = VehicleStateHandler()
        state.Setup()
        Container().SetState(state)

    def SetupRSU(self):
        try:
            world = Container().GetWorld()
            vehicle_bp = random.choice(world.get_blueprint_library().filter('static.prop.streetsign'))
            port = self._Configuration["Port"]
            role_name = f"RSU:localhost:{port}"
            vehicle_bp.set_attribute('role_name',role_name)
            
            transform = None

            if self._Configuration["ActorSpawning"]["Random"] is False:
                pickedLocation = self._Configuration["ActorSpawning"]["SpawnLocation"][0]
                location = carla.Location(x=pickedLocation[0], y=pickedLocation[1], z=pickedLocation[2])
                rotation = carla.Rotation(0,0,0)
                transform  = carla.Transform(location,rotation)
            else:
                transform = random.choice(Container().GetSpawnPoints())              
            
            while True:
                vehicle = world.spawn_actor(vehicle_bp,transform)
                
                if vehicle is not None:
                    time.sleep(10)
                    print(f"RSU {vehicle} spawned at {vehicle.get_transform()}")
                    Container().SetActor(vehicle)
                    break
        except Exception as ex:
            print(f"RSU not set {ex}")
        
    def Loop(self):
         self._Agent.RunStep()

    def MainSetup(self):

        self.SetupRSU()
        self.SetupStateHandler()
        self._Agent = Container().GetAgent()
        if self._Agent is None:
            print("Agent behaviour not set! Exiting")
            exit()
        self._Agent.Setup()
        self.Run()

    def Close(self):
        if self._LoopControl:
            self._LoopControl = False

