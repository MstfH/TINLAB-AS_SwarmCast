import state_machine
import jobs
import robot

'''
1000.0-50.0
[North, North-East, East, South East, South, South-West, West, North-West]
'''
def dSPoints(dSValue, currentState):
    previousJob = jobs.avoid()
    
    for i in range(8):
        sensorValue = dSValue[i]
        print(str(i) + ': ' + str(sensorValue))
        if sensorValue <= 150.0:
            if jobs.job != jobs.avoid():
                previousJob = jobs.job
            
            #if the comment in the line below is implemented the code crashes
            if str(currentState) == 'State.TRAVELING_NORTH':# and i == 0 or i == 1 or i == 7:
                if dSValue[5] >= 100.0 and dSValue[6] >= 100.0 and dSValue[7] >= 100.0:
                    jobs.movementDirection = 'west'
                    jobs.job = jobs.avoid()
                elif dSValue[1] >= 100.0 and dSValue[2] >= 100.0 and dSValue[3] >= 100.0:
                    jobs.movementDirection = 'east'
                    jobs.job = jobs.avoid()
                else:
                    jobs.movementDirection = 'south'
                    jobs.job = jobs.avoid()

            #if the line down below is outcommented just like line 22 code crashes
            elif str(currentState) == 'State.TRAVELING_EAST' and i == 2 or i == 1 or i == 3:
                if dSValue[3] >= 100.0 and dSValue[5] >= 100.0 and dSValue[4] >= 100.0:
                    jobs.movementDirection = 'south'
                    jobs.job = jobs.avoid()
                elif dSValue[1] >= 100.0 and dSValue[0] >= 100.0 and dSValue[7] >= 100.0:
                    jobs.movementDirection = 'north'
                    jobs.job = jobs.avoid()
                else:
                    jobs.movementDirection = 'west'
                    jobs.job = jobs.avoid()

            elif str(currentState) == 'State.TRAVELING_SOUTH' and i == 3 or i == 4 or i == 5:
                if dSValue[1] >= 100.0 and dSValue[2] >= 100.0 and dSValue[3] >= 100.0:
                    jobs.movementDirection = 'east'
                    jobs.job = jobs.avoid()
                elif dSValue[5] >= 100.0 and dSValue[6] >= 100.0 and dSValue[7] >= 100.0:
                    jobs.movementDirection = 'west'
                    jobs.job = jobs.avoid()
                else:
                    jobs.movementDirection = 'north'
                    jobs.job = jobs.avoid()

            elif str(currentState) == 'State.TRAVELING_WEST' and i == 5 or i == 6 or i == 7:
                if dSValue[3] >= 100.0 and dSValue[5] >= 100.0 and dSValue[4] >= 100.0:
                    jobs.movementDirection = 'south'
                    jobs.job = jobs.avoid()
                elif dSValue[1] >= 100.0 and dSValue[0] >= 100.0 and dSValue[7] >= 100.0:
                    jobs.movementDirection = 'north'
                    jobs.job = jobs.avoid()
                else:
                    jobs.movementDirection = 'east'
                    jobs.job = jobs.avoid()

        elif jobs.job == jobs.avoid():
            jobs.movementDirection = 'no'
            jobs.job = previousJob

    print(currentState)
