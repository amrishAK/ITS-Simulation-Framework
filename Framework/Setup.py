import sys
sys.path.append('Framework')
sys.path.append('Framework/carla-0.9.9-py3.7-linux-x86_64.egg')
import carla

from Container import Container
from RxHandler import Receiver
from Control import VehicleControl,RSUControl

import asyncio
from threading import Thread
import json

class Setup(object):
    __Configuration = None
    __Control = None
    __RxHandler = None 
    
    def __init__(self,configFile:str):
        self.__LoadConfigFile(configFile)
        self.SetupEnvoirment()
        receiverThread = Thread(target=self.SetupCommunication)
        receiverThread.daemon = True
        receiverThread.start()
        self.SetupControl()

    def __LoadConfigFile(self,pathStr):
        with open(pathStr, 'r') as file:
            conf =  json.loads(file.read())
            print(conf)
            self.__Configuration = conf
            Container().SetConfig(conf)

    def SetupEnvoirment(self):
        try:
            host = self.__Configuration["CarlaHost"]
            port = self.__Configuration["CarlaPort"]
            carlaClient = carla.Client(host,port, worker_threads=1)
            print("created client")
            carlaClient.set_timeout(20.0)

            world = carlaClient.get_world()
            print("World is loaded")
            
            # update settings
            if self.__Configuration["AsyncMode"] is True:
                settings = world.get_settings()
                settings.synchronous_mode = True
                world.apply_settings(settings)
                print("Settings updated")

            wMap = world.get_map()
            print("Map is loaded")

            points = wMap.get_spawn_points()
            print("SP is loaded")

            Container().SetWorld(world)
            Container().SetMap(wMap)
            Container().SetSpawnPoints(points)
            print("Envoirment created")

        except Exception as ex:
            print(ex)
            exit()

    def SetupCommunication(self):
        self.__RxHandler = Receiver()
        asyncio.run(self.__RxHandler.Setup())

    def SetupControl(self):
        print("In setup control")
        try:
            print(self.__Configuration["Type"])
            if self.__Configuration["Type"] == "Vehicle":
                self.__Control = VehicleControl()
            elif self.__Configuration["Type"] == "RSU":
                self.__Control = RSUControl()
        except Exception as ex:
            print(ex)
'''
Container is the repository class that holds all the required object throught the lifecycle
    1. StateHandler - one instance is shared throught the project (like singleton)
    2. RouteManager - one instance is shared throught the project (like singleton)
    3. WorldObject - one instance is shared throught the project (like singleton)
    4. VehicleObject - one instance is shared throught the project (like singleton)
    5. RxHandlers - instace is created if needed (like scoped) ## @addRxHandler("messageType")
    6. BroadCastHandler - instace is created if needed (like scoped)    
    7. Can also add custom helper classes as scoped or singleton

Setup 
    1. Adaptor (carla client)
    2. Communication unit (Rx and Tx)
    3. Control unit(Driving Peice/ RSU control)

Driving peice (way point navigation)
    1.Route management
    2.Driving Agent (motion plan) - customisable peice from base class Agent - @addAgent
    3.Contorl (PID controller - lateral and longidudinal)

Communication unit
    1. Broadcasting handler (v2v, v2r, v2i, i2v, r2v)
    2. Reception handler - background process (socket server) | receives request and process in async manner
    3. Message Rx handler - customisable peice from base class BaseRxHandler - @addRxHandler("messageType")
    4. Message Tx handler - customisable peice from base class - @addTxHandler("messageType","isBackgroundProcess")
'''