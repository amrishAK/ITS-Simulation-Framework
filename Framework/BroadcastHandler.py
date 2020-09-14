import asyncio
from Container import Container


class BroadCaster(object):    
    
    __loop = None

    def __init__(self):
        self.__loop = asyncio.get_event_loop()

    async def Broadcast(self,message,ip,port):
        try:
            _, writer = await asyncio.open_connection(ip, port, loop=self.__loop)
            writer.write(message.encode())
            writer.close()
        except:
            print(f"exception occure when{ip},{port}")
            pass

    def V2V(self,data):
        communicationRange = Container().GetState().CommunicationRange
        orgin = Container().GetActor()
        world = Container().GetWorld()
        vehicle_list = world.get_actors().filter("*vehicle*")
        
        orginLocation = orgin.get_location()
        orginId = orgin.id

        def isInRange(v) :  orginLocation.distance(v.get_location()) <= communicationRange
        
        for vehicle in vehicle_list:
            if vehicle.id is not orginId or isInRange(vehicle):
                array = vehicle.attributes.get('role_name').split(':')
                ip = array[1]
                port = array[2]
                self.__loop.run_until_complete(self.Broadcast(data,ip,port))

    def V2x(self,data):
        # passtatic.prop.streetsign

        communicationRange = Container().GetState().CommunicationRange
        orgin = Container().GetActor()
        world = Container().GetWorld()
        _list = world.get_actors().filter("*vehicle*")
        _list.extend (world.get_actors.filter('passtatic.prop.streetsign'))

        orginLocation = orgin.get_location()
        orginId = orgin.id

        def isInRange(v) :  orginLocation.distance(v.get_location()) <= communicationRange
        
        for vehicle in _list:
            if vehicle.id is not orginId or isInRange(vehicle):
                array = vehicle.attributes.get('role_name').split(':')
                ip = array[1]
                port = array[2]
                self.__loop.run_until_complete(self.Broadcast(data,ip,port))


    def R2V(self,data):
        world = Container().GetWorld()
        vehicle_list = world.get_actors().filter("*vehicle*")
        print(Container().GetActor())
        def isInRange(v) : return Container().GetActor().get_location().distance(v.get_location()) <= 50
        
        for vehicle in vehicle_list:
            if isInRange(vehicle):
                print(f"Role is {vehicle.attributes.get('role_name')}")
                array = vehicle.attributes.get('role_name').split(':')
                ip = array[1]
                port = array[2]
                self.__loop.run_until_complete(self.Broadcast(data,ip,port))

class SampleTxHandler(object):
    pass