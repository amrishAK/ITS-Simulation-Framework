import socket
import asyncio
from threading import Thread
from abc import ABC, abstractmethod
import json
from Container import Container


class Receiver(object):

    __Socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def __init__(self):
        pass

    def Close(self):
        self.server.close()

    async def Setup(self):
        config = Container().GetConfig()
        host = 'localhost'
        port = config["Port"]
        self.server = await asyncio.start_server(self.handle_echo, host,port)
        async with self.server:
            await self.server.serve_forever()

    async def handle_echo(self,reader, writer):
        data = await reader.read(1024)
        message = data.decode()

        if message is None:
            writer.close

        try:
            msgJson = json.loads(message)
            msgType = msgJson['msgType']
            handler = Container().GetRxHandler(msgType)
            handler.Main(msgJson)
        except:
            pass
        #close the connection
        writer.close


class BaseRxHandler (ABC):
    '''
        Abstract method: Main(self,data)
    '''
    @abstractmethod
    def Main(self,data):
        raise NotImplementedError


