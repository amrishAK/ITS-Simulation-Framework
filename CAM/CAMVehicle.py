from Framework.RoadOption import RoadOption
from Framework.Setup import Setup
from Framework.Container import Container
from Framework.BehaviourAgent import BaseVehicleAgent
from Framework.FrameworkException import StopException
from Framework.RxHandler import BaseRxHandler
from Framework.Decrators import drivingAgent, addRxHandler
from Framework.CarlaVizUtil import CarlaPainter
from Framework.misc import get_speed
from Framework.BroadcastHandler import BroadCaster

import random
import json


## Behaviour agent ----------------

@drivingAgent
class CAMAgent(BaseVehicleAgent):

    CAMSpeedLimit = None
    painter = CarlaPainter('localhost',8089)

    def __init__(self,container:Container):
        self.__Container = container


    def GenerateMessage(self):
        data = {}
        data['msgType'] = 'CAM'
        CAM = {}
        CAM['CAMId'] = random.random()
        CAM['msg'] = "RSU1"
        data['msg'] = CAM
        return json.dumps(data)

    def RunStep(self):

        egoVehicle = self.__Container.GetActor()
        ego_vehicle_loc = egoVehicle.get_location()

        message : dict = self.__Container.GetState().CurrentMsg['CAM']
        text = ""

        for i, value in message.items():
            text += f"{i}|{value['CAMId']}|{value['msg']}"
        
        self.__Container.GetState().CurrentMsg['CAM'] = {}
        
        self.painter.draw_texts(text,[[ego_vehicle_loc.x,ego_vehicle_loc.y,10]])

        broadCast = BroadCaster()
        broadCast.R2V(self.GenerateMessage())
       
        targetSpeed, _ = self.BasicMobiltyModel()

        return targetSpeed


### CAM Message handler ------------

@addRxHandler('CAM')
class CAMHandler(BaseRxHandler):

    def __init__(self,container:Container):
        self.__Container = container

    def Main(self,data):
        
        message = data['msg']
        CAMId = message['CAMId']
        # ["CAM"]
        messageDict : dict = self.__Container.GetState().CurrentMsg["CAM"]

        #Checking if the CAM message exists
        if CAMId in messageDict.keys():
            #if yes skip
            return 

        #if no add to the dict
        message['Status'] = "Pending"
        messageDict[CAMId] = message

        # update the state
        self.__Container.GetState().CurrentMsg['CAM'] = messageDict        



import argparse

if __name__ == "__main__":    
    argument = argparser = argparse.ArgumentParser()
    argument.add_argument('--file',metavar='F',default='')
    args = argument.parse_args()
    cf = args.file 
    try:
        Setup(cf)
    except KeyboardInterrupt:
        print("close!!")