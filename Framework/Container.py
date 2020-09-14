from Singleton import Singleton


@Singleton
class Container(object):
    
    __Configration = None
    __RxHandlers = {}

    #simulation objects
    __World = None
    __RouteManager = None
    __Map = None
    __MapSpawnPoints = None
    
    __State = None
    __Actor = None

    __Agent = None
    def SetAgent(self,cls):
        self.__Agent = cls

    def GetAgent(self):
        try:
            print(f"actor is  {self.__Actor}")
            return self.__Agent(self)
        except:
            return None

    def SetConfig(self,config):
        self.__Configration = config

    def GetConfig(self):
        return self.__Configration

    def AddRxHandler(self,messageType,cls):
        self.__RxHandlers[messageType] = cls
        print(f"Adding Handlers..... {self.__RxHandlers}")

    def GetRxHandler(self,messageType):
        print(f"getting Handlers..... {self.__RxHandlers}")
        if messageType in self.__RxHandlers.keys():
            return self.__RxHandlers[messageType](self)
        else:
            raise Exception
        
    def GetRxHandlersList(self):
        print(f"getting handlers list {self.__RxHandlers.keys()}")
        return self.__RxHandlers.keys()

    def SetWorld(self,world):
        self.__World = world

    def GetWorld(self):
        return self.__World

    def SetActor(self,vehicle):
        self.__Actor = vehicle
        
    def GetActor(self):
        return self.__Actor

    def SetRouteManager(self,vehicle):
        self.__RouterManager = vehicle

    def GetRouteManager(self):
        return self.__RouterManager

    def SetState(self,vehicle):
        self.__State = vehicle

    def GetState(self):
        return self.__State 

    def SetMap(self,map):
        self.__Map = map

    def GetMap(self):
        return self.__Map

    def SetSpawnPoints(self,points):
        self.__MapSpawnPoints = points

    def GetSpawnPoints(self):
        return self.__MapSpawnPoints
