# Webots imports

# Python imports
import time
import json

# Custom imports
import robot

# ---SwarmCast decentralized aggregation algorithm---

# Robot aggregation attributes.
robotInfo = {
    "R-ID": 2,
    "ID-List": [2],
    "G-ID": 2,
    "G-Size": 1
}

TWaiting = 3            # Waiting time in sec.
TAvoiding = 5           # Avoiding time in sec.
goalNumber = 2          # Total aggregation goal number.

# During the aggregation the robot has 2 states (behaviours). The search and the wait state.
# The search state is when the robot is searching for another robot, so that it can start or join an aggregation with another robot.
# The robot goes into the wait state when it detects a robot, so that it can communicate with the robot.


def search():
    print("Searching state")

    # Move forward in a random direction
    robot.move("N")

    detected = readSensors()

    if detected:

        # Stop moving forward
        robot.move("I")

        HELLO = {
            "Name": "HELLO",
            "R-ID": robotInfo.get("R-ID")
        }

        sendBroadcastMessage(HELLO)
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

            ACK = {
                "Name": "ACK",
                "R-ID": robotInfo.get("R-ID"),
                "G-ID": robotInfo.get("G-ID"),
                "G-Size": robotInfo.get("G-Size"),
                "ID-List": robotInfo.get("ID-List"),
                "State": "wait"
            }

            sendP2PMessage(ACK, message.get("R-ID"))

        elif message.get("Name") == "PROPAGATE":
            if robotInfo.get("G-ID") == message.get("G-ID"):
                print("Waiting in aggregation")
                # If the message received is PROPAGATE and the tag is joining, this robot will add the new robot to the group.
                if message.get("R-ID") not in robotInfo.get("ID-List") and message.get("Tag") == "joining":
                    print("Receiving PROPAGATE after ACK")
                    print("New robot is joining the group")
                    robotInfo.get(
                        "ID-List").append(message.get("R-ID"))
                    newGSize = robotInfo.get("G-Size") + 1
                    robotInfo.update({"G-Size": newGSize})

                    WTimer = k * robotInfo.get("G-Size")
                    tEnd = time.time() + WTimer

                    # TODO: Check if the message is being broadcasted properly to bots in aggregation
                    sendBroadcastMessage(message)

                # If the message received is PROPAGATE and the tag is leaving, this robot will remove the robot to the group.
                else:
                    if message.get("R-ID") in robotInfo.get("ID-List") and message.get("Tag") == "leaving":
                        print("A robot is leaving the group")
                        robotInfo.get(
                            "ID-List").remove(message.get("R-ID"))
                        newGSize = robotInfo.get("G-Size") - 1
                        robotInfo.update({"G-Size": newGSize})

                        # TODO: Check if the message is being broadcasted properly to bots in aggregation
                        sendBroadcastMessage(message)

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

        sendBroadcastMessage(PROPAGATE)

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

        sendP2PMessage(ACK, message.get("R-ID"))

        wait()

# This function regulates the steps taken during the wait to search transition.


def waitToSearch():
    print("Transitioning from wait to search")

    # Stop the loop if aggregation goal number is reached
    if robotInfo.get("G-Size") == goalNumber:
        return

    # Setup and broadcasting a PROPAGATE message with "leaving" tag.
    PROPAGATE = {
        "Name": "PROPAGATE",
        "R-ID": robotInfo.get("R-ID"),
        "G-ID": robotInfo.get("G-ID"),
        "Tag": "leaving"
    }

    sendBroadcastMessage(PROPAGATE)

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
        if values <= 50:
            detected = True
    return detected

# This function sends broadcast HELLO and PROPAGATE messages via an emmiter


def sendBroadcastMessage(MessageToSend):
    robot.emitter.setChannel(robot.emitter.CHANNEL_BROADCAST)
    jsonMessage = json.dumps(MessageToSend)
    robot.emitter.setRange(51)
    robot.emitter.send(jsonMessage.encode())
    robot.emitter.setChannel(robotInfo.get("R-ID"))
    print("Sending broadcast:", MessageToSend)

# This function sends P2P ACK messages via an emmiter


def sendP2PMessage(MessageToSend, channel):
    robot.emitter.setChannel(channel)
    jsonMessage = json.dumps(MessageToSend)
    robot.emitter.setRange(51)
    robot.emitter.send(jsonMessage.encode())
    robot.emitter.setChannel(robotInfo.get("R-ID"))
    print("Sending P2P:", MessageToSend)

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
    robot.emitter.setChannel(robotInfo.get("R-ID"))
    robot.receiver.setChannel(robotInfo.get("R-ID"))
    # Loop to keep the aggregation algorithm running until the goal is reached (an aggregation of 25 robots)
    while robotInfo.get("G-Size") < goalNumber:
        robot.step(robot.timestep)

        if robotInfo.get("G-Size") == goalNumber:
            break
        else:
            search()
