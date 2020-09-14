from Container import Container,addStateHandler,addRxHandler
from StateHandler import BaseStateHandler
from RxHandler import BaseRxHandler
from Agent import Agent

# @addStateHandler
class SBStateHandler(BaseStateHandler):

    SlotBasedDriving  = False
    
    #slot infromation
    SlotSpeed = None
    SlotHeadway = None

    TSPoint = None
    TEPoint = None

#messgae Handler
# @addRxHandler('SBM')
class SlotMessageHandler(BaseRxHandler):

    def Main(self,data):
        
        if data['MessageType'] is 'TransistionState':
            
            message = data['Message']

            slotHandler : SBStateHandler = Container().GetState()

            slotHandler.SlotSpeed = message['SlotSpeed']
            slotHandler.SlotHeadway = message['SlotHeadway']

            slotHandler.TEPoint = message['TEPoint']
            slotHandler.TSPoint = message['TSPoint']

# agent
class SBDrivingAgent(Agent):
    
    def RunStep(self):
        targetSpeed = None
        return targetSpeed