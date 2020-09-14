from Container import Container

#decrators ----------->>
def addRxHandler(id_):
    print(f"adding RxHandler...... {id_}")
    def innterFunction(cls):
        Container().AddRxHandler(id_,cls)
    return innterFunction

def addStateHandler(cls):
    Container().SetState(cls())

#decorator
def drivingAgent(cls):
    print(f"Adding Vehicle agent{cls}")
    Container().SetAgent(cls)

def rsuAgent(cls):
    print(f"Adding RSU agent {cls}")
    Container().SetAgent(cls)