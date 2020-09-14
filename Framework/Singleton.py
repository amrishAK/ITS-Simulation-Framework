import threading

class SingletonWraper:

    __instance = None
    __decrator = None

    def __call__(self):
        if self.__instance is None:
            with threading.Lock():
                if self.__instance is None:
                    self.__instance = self.__decrator()
        # print(f"in Wrapper {self.__instance.GetActor()}")
        return self.__instance 


    def __init__(self,decrator):
        self.__decrator = decrator

def Singleton(cls):
    return SingletonWraper(cls)