# Webots imports

# Python imports
import time
import json

# Custom imports
import robot

# ---SwarmCast decentralized aggregation algorithm---

# Robot aggregation attributes
robotInfo = {
    "R-ID": 2,
    "ID-List": [2],
    "G-ID": 2,
    "G-Size": 1
}

TWaiting = 3            # Waiting time in sec.
TAvoiding = 5           # Avoiding time in sec.
goalNumber = 2          # Total aggregation goal number

# During the aggregation the robot has 2 states (behaviours). The search and the wait state.
# The search state is when the robot is searching for another robot, so that it can start or join an aggregation with another robot.
# The robot goes into the wait state when it detects an object, so that it can communicate with the object.


def search():
    print("Searching state")

    # TODO: move forward in a random direction

    detected = readSensors()

    if detected:

        # TODO: stop moving forward

        HELLO = {
            "Name": "HELLO",
            "R-ID": robotInfo.get("R-ID")
        }

        sendMessage(HELLO)
        response = {}
        response = getMessage()

        if response.get("Name") == "ACK" or response.get("Name") == "HELLO":
            searchToWait(response)

    else:
        print("Respond not received, moving on")
        tEnd = time.time() + TAvoiding
        while time.time() < tEnd:
            robot.step(robot.timestep)
            print("Avoiding obstacles")
            # TODO: rotate and look for a free direction,
            #       move forward in a free random direction and avoid obstacles


def wait():
    print("Waiting state")
    k = 10
    # This timer is the lifespan of the robots in the aggregation. The timer increases with the size of the group.
    WTimer = k * robotInfo.get("G-Size")
    tEnd = time.time() + WTimer

    message = {}

    # for testing if the WTimer resets after receiving propagation message
    print("Waiting time:", tEnd)

    while time.time() < tEnd:
        robot.step(robot.timestep)

        detected = readSensors()

        # If an object is detected, listen for new messages. If not, break out of loop.
        if detected:
            if robot.receiver.getQueueLength() > 0:
                rawData = robot.receiver.getData().decode()
                receivedMessage = json.loads(rawData)
                message = receivedMessage
                robot.receiver.nextPacket()
        elif not detected:
            print("None object detected, breaking out of WTimer loop")
            break

        # If the message received is HELLO, send ACK message as a response.
        if message.get("Name") == "HELLO":
            print("Sending ACK to HELLO")
            # TODO: send ACK message to sender of HELLO with UNIQUE(P2P) channel. Right now its broadcasting...
            ACK = {
                "Name": "ACK",
                "R-ID": robotInfo.get("R-ID"),
                "G-ID": robotInfo.get("G-ID"),
                "G-Size": robotInfo.get("G-Size"),
                "ID-List": robotInfo.get("ID-List"),
                "State": "wait"
            }

            sendMessage(ACK)

        elif message.get("Name") == "PROPAGATE":
            print("Receiving PROPAGATE after ACK")
            if robotInfo.get("G-ID") == message.get("G-ID"):
                # If the message received is PROPAGATE and the tag is joining, this robot will add the new robot to the group.
                if message.get("R-ID") not in robotInfo.get("ID-List") and message.get("Tag") == "joining":
                    print("Joining group")
                    robotInfo.get(
                        "ID-List").append(message.get("R-ID"))
                    newGSize = robotInfo.get("G-Size") + 1
                    robotInfo.update({"G-Size": newGSize})

                    WTimer = k * robotInfo.get("G-Size")
                    tEnd = time.time() + WTimer

                    print("New Group size:", robotInfo.get("G-Size"))
                    print("New timer:", tEnd)

                    # TODO: Check if the message is being broadcasted properly to bots in aggregation
                    sendMessage(message)

                # If the message received is PROPAGATE and the tag is leaving, this robot will remove the robot to the group.
                else:
                    if message.get("R-ID") in robotInfo.get("ID-List") and message.get("Tag") == "leaving":
                        print("Leaving group")
                        robotInfo.get(
                            "ID-List").remove(message.get("R-ID"))
                        newGSize = robotInfo.get("G-Size") - 1
                        robotInfo.update({"G-Size": newGSize})

                        # TODO: Check if the message is being broadcasted properly to bots in aggregation
                        sendMessage(message)

    waitToSearch()

# This function regulates the steps taken during the search to wait transition.


def searchToWait(message):
    # If an ACK message is received, add ACK attributes to robot attributes. Then send joining PROPAGATE message and enter wait state.
    if message.get("Name") == "ACK":
        print("ACK - Transitioning from search to wait")

        robotInfo["G-ID"] = message.get("G-ID")
        robotInfo["G-Size"] = message.get("G-Size") + 1
        for x in message.get("ID-List"):
            if x not in robotInfo.get("ID-List"):
                robotInfo.get("ID-List").append(x)

        # Setup and broadcasting a PROPAGATE message with "joining" tag.
        PROPAGATE = {
            "Name": "PROPAGATE",
            "R-ID": robotInfo.get("R-ID"),
            "G-ID": robotInfo.get("G-ID"),
            "Tag": "joining"
        }

        sendMessage(PROPAGATE)

        wait()

    # If a HELLO message is received, send an ACK message in return.
    elif message.get("Name") == "HELLO" and message.get("R-ID") < robotInfo.get("R-ID"):
        print("HELLO - transitioning from search to wait")
        ACK = {
            "Name": "ACK",
            "R-ID": robotInfo.get("R-ID"),
            "G-ID": robotInfo.get("G-ID"),
            "G-Size": robotInfo.get("G-Size"),
            "ID-List": robotInfo.get("ID-List"),
            "State": "wait"
        }

        sendMessage(ACK)

        wait()

# This function regulates the steps taken during the wait to search transition.


def waitToSearch():
    print("Transitioning from wait to search")

    # Setup and broadcasting a PROPAGATE message with "leaving" tag.
    PROPAGATE = {
        "Name": "PROPAGATE",
        "R-ID": robotInfo.get("R-ID"),
        "G-ID": robotInfo.get("G-ID"),
        "Tag": "leaving"
    }

    sendMessage(PROPAGATE)

    # Reset robot's attributes after leaving the group.
    robotInfo["ID-List"].clear()
    robotInfo["ID-List"].append(
        robotInfo.get("R-ID"))
    robotInfo["G-ID"] = robotInfo.get("R-ID")
    robotInfo["G-Size"] = 1

    tEnd = time.time() + TAvoiding
    while time.time() < tEnd:
        robot.step(robot.timestep)
        print("Avoiding obstacles")
        # TODO: rotate and look for a free direction,
        #       move forward in a free random direction and avoid obstacles

    search()

# This function reads distance sensors and returns true when an object detected.


def readSensors():
    dsValues = []
    detected = False
    for i in range(8):
        dsValues.append(robot.ds[i].getValue())

    for values in dsValues:
        if values <= 100:
            detected = True
    return detected

# This function sends messages via an emmiter


def sendMessage(MessageToSend):
    jsonMessage = json.dumps(MessageToSend)
    robot.emitter.setRange(101)
    robot.emitter.send(jsonMessage.encode())
    print("Sending:", MessageToSend)

# This function receives message via a receiver


def getMessage():
    receivedMessage = {}
    tEnd = time.time() + TWaiting
    while time.time() < tEnd:
        robot.step(robot.timestep)
        print("Waiting for response")

        if robot.receiver.getQueueLength() > 0:
            rawData = robot.receiver.getData().decode()
            receivedMessage = json.loads(rawData)
            robot.receiver.nextPacket()
            break
    return receivedMessage


def run():
    # Loop to keep the aggregation algorithm running until the goal is reached (an aggregation of 25 robots)
    while robotInfo.get("G-Size") < goalNumber:
        robot.step(robot.timestep)

        if robotInfo.get("G-Size") == goalNumber:
            break
        else:
            search()
