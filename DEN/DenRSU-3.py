from Framework.Setup import Setup
from Framework.BehaviourAgent import BaseRsuAgent
from Framework.BroadcastHandler import BroadCaster
from Framework.CarlaVizUtil import CarlaPainter
from Framework.Decrators import rsuAgent
from Framework.Container import Container
import json
import time
import random

@rsuAgent
class DenRSU(BaseRsuAgent):

    painter = CarlaPainter('localhost',8089)

    def __init__(self,container:Container):
        self.__Container = container

    def GenerateMessage(self):
        data = {}
        data['msgType'] = 'DEN'
        den = {}
        den['DenId'] = random.random()
        den['Type'] = "LaneStatus"
        den['LaneID'] = "-3"
        den["LaneSatus"] = "Close"
        data['msg'] = den
        return json.dumps(data)

    def Setup(self):
        try:
            location = self.__Container.GetActor().get_location()
            # texts = ["Lane '-3' is closed"]
            # self.painter.draw_texts(texts,[[location.x,location.y,10]])
            self.painter.draw_polylines([[location.x+5,location.y+5,0],[location.x-5,location.y-5,0],[location.x-5,location.y+5,0],[location.x+5,location.y-5,0]])
            self.__Message = self.GenerateMessage()
        except Exception as ex:
            print(ex)

    def RunStep(self):
        broadCast = BroadCaster()
        broadCast.R2V(self.__Message)
        time.sleep(5)


if __name__ == "__main__":    
    try:
        Setup("RSU-3Config.json")
    except KeyboardInterrupt:
        print("close!!")
