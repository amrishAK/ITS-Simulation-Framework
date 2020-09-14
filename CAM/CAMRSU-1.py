from Framework.Setup import Setup
from Framework.BehaviourAgent import BaseRsuAgent
from Framework.BroadcastHandler import BroadCaster
from Framework.CarlaVizUtil import CarlaPainter
from Framework.RxHandler import BaseRxHandler
from Framework.Decrators import rsuAgent,addRxHandler
from Framework.Container import Container
import json
import time
import random

@rsuAgent
class CAMRSU(BaseRsuAgent):

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

    def Setup(self):
        try:
            self.location = self.__Container.GetActor().get_location()
            location = self.location
            self.painter.draw_polylines([[location.x+5,location.y+5,0],[location.x-5,location.y-5,0],[location.x-5,location.y+5,0],[location.x+5,location.y-5,0]])
            self.__Message = self.GenerateMessage()
        except Exception as ex:
            print(ex)

    def RunStep(self):

        message : dict = self.__Container.GetState().CurrentMsg['CAM']
        text = ""

        for i, value in message.items():
            text += f"{i}|{value['CAMId']}|{value['msg']}"
        
        self.__Container.GetState().CurrentMsg['CAM'] = {}
        
        self.painter.draw_texts(text,[[self.location.x,self.location.y,10]])

        broadCast = BroadCaster()
        broadCast.R2V(self.__Message)
        time.sleep(5)

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



if __name__ == "__main__":    
    try:
        print("first")
        Setup("RSU-1Config.json")
    except KeyboardInterrupt:
        print("close!!")
