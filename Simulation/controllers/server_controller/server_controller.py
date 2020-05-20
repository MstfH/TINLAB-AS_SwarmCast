from controller import Supervisor
from math import sqrt

server = Supervisor()
TIME_STEP = int(server.getBasicTimeStep())

# do this once only
# robot_node = supervisor.getFromDef("OMNI_WHEELS")
# trans_field = robot_node.getField("translation")

botNode = []
botNames = [
    "bot1", "bot2"]

# #populate botNode

for i in range(len(botNames)):
    botNode.append(server.getFromDef(botNames[i]))
    
    
#print ID + translation
while server.step(TIME_STEP) != -1:
    for i in range(len(botNode)):
        transval = botNode[i].getField("translation").getSFVec3f()
        print(i, ": ", transval)
