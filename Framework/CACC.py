from Container import Container


#messageType
#message
    #vehicleId
    #data
from RxHandler import BaseRxHandler

class CAMRxHandler(BaseRxHandler):

    def Main(self,data):
        camMsg = Container().GetState().CurrentMsg['CAM']
        id = data['vehicleId']       
        camMsg[id] = data['data']
        Container().GetState().CurrentMsg['CAM'] = camMsg

class CACCRxHandler(BaseRxHandler):

    def Main(self,data):