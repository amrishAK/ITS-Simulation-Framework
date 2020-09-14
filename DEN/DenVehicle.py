from Framework.RoadOption import RoadOption
from Framework.Setup import Setup
from Framework.Container import Container
from Framework.BehaviourAgent import BaseVehicleAgent
from Framework.FrameworkException import StopException
from Framework.RxHandler import BaseRxHandler
from Framework.Decrators import drivingAgent, addRxHandler
from Framework.CarlaVizUtil import CarlaPainter
from Framework.misc import get_speed
import json


## Behaviour agent ----------------

@drivingAgent
class DENAgent(BaseVehicleAgent):

    DenSpeedLimit = None
    painter = CarlaPainter('localhost',8089)

    def __init__(self,container:Container):
        self.__Container = container

    def CheckForLaneChangeDen(self,location):
        message : dict = self.__Container.GetState().CurrentMsg['DEN']
        
        for _, value in message.items():
            #lane change message
            if value['Type'] == "LaneStatus":
                cwp = self.RouteManager.currentWaypoint
                claneId = cwp.lane_id
                
                if value['Status'] is not 'Sucess' and str(claneId) == value['LaneID']:

                    vehicle_list = self.__Container.GetWorld().get_actors().filter("*vehicle*")
                    def distace(v): return v.get_location().distance(location)
                    vehicle_list = [v for v in vehicle_list if distace(v) < 45 and v.id != self.vechileState.vehicleId]
                    status = self.AgentLaneChange(location,cwp,vehicle_list)
                    value['Status'] = 'Sucess' if status else 'Failed'

                    return True

            elif value['Type'] == "SpeedLimit":
                print(value)

                if value['Status'] is not 'Sucess':
                    if int(value['Speed']) == 0:
                        self.DenSpeedLimit = None
                    else:
                        self.DenSpeedLimit = int(value['Speed'])
            else:
                pass
        return False

    def RunStep(self):

        egoVehicle = self.__Container.GetActor()
        ego_vehicle_loc = egoVehicle.get_location()

        #display lane id and speed
        texts = [f"Lane: {self.RouteManager.currentWaypoint.lane_id}\n Current Speed:{int(get_speed(egoVehicle))} "]
        self.painter.draw_texts(texts,[[ego_vehicle_loc.x,ego_vehicle_loc.y,10]])

        targetSpeed, behaviour = self.BasicMobiltyModel()

        if behaviour is "normal" and self.CheckForLaneChangeDen(ego_vehicle_loc):
                targetSpeed = min(self.Behavior.max_speed, self.vechileState.speedLimit - 5)

        if self.DenSpeedLimit is not None:
            targetSpeed = self.DenSpeedLimit

        return targetSpeed


### DEN Message handler ------------

@addRxHandler('DEN')
class DenHandler(BaseRxHandler):

    def __init__(self,container:Container):
        self.__Container = container

    def Main(self,data):
        
        message = data['msg']
        denId = message['DenId']
        # ["DEN"]
        messageDict : dict = self.__Container.GetState().CurrentMsg["DEN"]

        #Checking if the Den message exists
        if denId in messageDict.keys():
            #if yes skip
            return 

        #if no add to the dict
        message['Status'] = "Pending"
        messageDict[denId] = message

        # update the state
        self.__Container.GetState().CurrentMsg['DEN'] = messageDict        



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