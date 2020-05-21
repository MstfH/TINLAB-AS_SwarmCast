# Webots imports

# Python imports
import time
import json
import random

# Custom imports
import robot

# ---SwarmCast decentralized aggregation algorithm---

# The robots attributes. The ID needs to be unique for each robot (random atm for testing)
robotInfo = {
    "R-ID": 2,
    "ID-List": [2],
    "G-ID": 2,
    "G-Size": 1
}

distanceInBetween = 4   # Distance between the robot and the object.
sensorRange = 5         # The range of the sensor.
TWaiting = 5            # Waiting time in sec.
TAvoiding = 2          # Avoiding time in sec.


# During the aggregation the robot has 2 states (behaviours). The search and the wait state.
# The search state is when the robot is searching for another robot, so that it can start or join an aggregation with another robot.
# The wait state is when the robot detects an object, so that it can communicate with the object.
def search():
    print("Searching state")

    # TODO: setup TAvoiding,
    # TODO: moveForward() and
    # TODO: avoidObstacle()

    tEnd = time.time() + TAvoiding
    while time.time() < tEnd:
        robot.step(robot.timestep)

    distanceInBetween = 6   # for testing
    sensorRange = 5         # for testing

    # If an object is detected, send an HELLO message to the object and wait for response.
    if distanceInBetween < sensorRange:
        # message["R-ID"] = robotInfo.get("R-ID")
        # TODO: Stop moving, turn on radio, send HELLO message to detected object/robot.

        HELLO = {
            "Name": "HELLO",
            "R-ID": robotInfo.get("R-ID")
        }

        ACK = {}

        sendMessage(HELLO)
        ACK = getMessage()

        # If the object respond with an ACK message, return the message.
        if ACK.get("Name") == "ACK":
            print("ACK received")
            searchToWait(ACK)

        # If the object does not respond, the robot will identify the object as an obstacle and apply obstacle avoidance.
        else:
            print("ACK not received, moving on")
            tEnd = time.time() + TAvoiding
            while time.time() < tEnd:
                robot.step(robot.timestep)
                print("Message system disabled to avoid obstacle")
                # TODO: moveForward()
                # TODO: avoidObstacle()
                # TODO: disable message system(radio) for the time being


def wait():
    print("Waiting state")
    # This timer is the lifespan of the robots in the aggregation. The timer encreases with the size of the size of the group.
    WTimer = robotInfo.get("G-Size")
    message = {}

    # The time the robot have the stay in the group.
    tEnd = time.time() + WTimer
    while time.time() < tEnd:
        robot.step(robot.timestep)

    if distanceInBetween < sensorRange:
        if robot.receiver.getQueueLength() > 0:
            print("Number of packets:", str(robot.receiver.getQueueLength()))
            rawData = robot.receiver.getData().decode()
            receivedMessage = json.loads(rawData)
            message = receivedMessage
            robot.receiver.nextPacket()

    # If the message received is HELLO, send ACK message as a respond.
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
                print("joining")
                robotInfo.get("ID-List").append(message.get("R-ID"))
                print(robotInfo.get("ID-List"))
                newGSize = robotInfo.get("G-Size") + 1
                robotInfo.update({"G-Size": newGSize})

                # TODO: Check if the message is being broadcasted properly to bots in aggregation
                sendMessage(message)

            # If the message received is PROPAGATE and the tag is leaving, this robot will remove the robot to the group.
            else:
                if message.get("R-ID") in robotInfo.get("ID-List") and message.get("Tag") == "leaving":
                    print("leaving")
                    robotInfo.get("ID-List").remove(message.get("R-ID"))
                    print(robotInfo.get("ID-List"))
                    newGSize = robotInfo.get("G-Size") - 1
                    robotInfo.update({"G-Size": newGSize})
                    # TODO: Check if the message is being broadcasted properly to bots in aggregation
                    sendMessage(message)


# This function regulates the steps taken during the search to wait transition.
def searchToWait(message):
    # TODO: stop riding

    # If ACK message is received, set and add ACK attributes to robot attributes. Then send joining PROPAGATE message and enter wait state.
    if message.get("Name") == "ACK":
        print("ACK transitioning from search to wait")
        # TODO: Write a way to use only the first ACK message received

        robotInfo["G-ID"] = message.get("G-ID")
        robotInfo["G-Size"] = message.get("G-Size") + 1
        for x in message.get("ID-List"):
            if x not in robotInfo.get("ID-List"):
                robotInfo.get("ID-List").append(x)

        # Setup and broadcasting a PROPAGATE message with "joining" tag
        PROPAGATE = {
            "Name": "PROPAGATE",
            "R-ID": robotInfo.get("R-ID"),
            "G-ID": robotInfo.get("G-ID"),
            "Tag": "joining"
        }
        sendMessage(PROPAGATE)

        wait()

        # Setup and broadcasting a PROPAGATE message with "leaving" tag
        PROPAGATE = {
            "Name": "PROPAGATE",
            "R-ID": robotInfo.get("R-ID"),
            "G-ID": robotInfo.get("G-ID"),
            "Tag": "leaving"
        }
        sendMessage(PROPAGATE)
        search()

    elif message.get("Name") == "HELLO" and message.get("R-ID") < robotInfo.get("R-ID"):
        print("HELLO transitioning from search to wait")
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

        # Setup and broadcasting a PROPAGATE message with "leaving" tag
        PROPAGATE = {
            "Name": "PROPAGATE",
            "R-ID": robotInfo.get("R-ID"),
            "G-ID": robotInfo.get("G-ID"),
            "Tag": "leaving"
        }
        sendMessage(PROPAGATE)
        search()


# This function regulates the steps taken during the search to wait transition.
def waitToSearch():
    # Reset robot's attributes after leaving the group
    robotInfo["ID-List"].clear()
    robotInfo["ID-List"].append(robotInfo.get("R-ID"))
    robotInfo["G-ID"] = robotInfo.get("R-ID")
    robotInfo["G-Size"] = 1
    search()


# This function regulates the states transition (of search and wait) of the robot.
# The robot always start in the searching state.
# If an object is detected, the searchToWait function is called.
# If no object is detected, the waitToSearch function is called.
def transition(message):
    if distanceInBetween < sensorRange:
        print("to wait")
        searchToWait(message)
    elif distanceInBetween >= sensorRange and message.get("Tag") == "leaving":
        print("to search with tag")
        waitToSearch()
    else:
        print("to search")
        search()


def sendMessage(MessageToSend):
    jsonMessage = json.dumps(MessageToSend)
    robot.emitter.send(jsonMessage.encode())
    print("Sending...", MessageToSend)


def getMessage():
    receivedMessage = {}
    tEnd = time.time() + TWaiting
    while time.time() < tEnd:
        robot.step(robot.timestep)
        print("waiting for ACK respond")

        if robot.receiver.getQueueLength() > 0:
            rawData = robot.receiver.getData().decode()
            receivedMessage = json.loads(rawData)
            robot.receiver.nextPacket()
            break
    return receivedMessage


# def radio():
#     receivedMessage = ""

#     # Messages
#     HELLO_test = {
#         "Name": "HELLO",
#         "R-ID": 0
#     }

#     ACK_test = {
#         "Name": "ACK",
#         "R-ID": 1,
#         "G-ID": 1,
#         "G-Size": 1,
#         "ID-List": [1],
#         "State": "joining"
#     }

#     PROPAGATE_test = {
#         "Name": "PROPAGATE",
#         "R-ID": 0,
#         "G-ID": 0,
#         "Tag": ""
#     }

#     tempDict = {}

#     jsonMessage = json.dumps(HELLO_test)

#     robot.emitter.send(jsonMessage.encode())

#     if robot.receiver.getQueueLength() > 0:
#         # print("Rx:", robot.receiver.getData(), " strength:", robot.receiver.getSignalStrength(
#         # ), " direction:", robot.receiver.getEmitterDirection())

#         rawData = robot.receiver.getData().decode()
#         receivedMessage = json.loads(rawData)
#         tempDict = receivedMessage
#         print("RECEIVED message:", tempDict)

#     return tempDict


def run():
    receivedMessage = {}

    # Loop to keep the aggregation algorithm running unitl the goal is reached (an aggregation of 25 robots)
    goalNumber = 2
    while robotInfo.get("G-Size") < goalNumber:

        if robot.receiver.getQueueLength() > 0:
            rawData = robot.receiver.getData().decode()
            receivedMessage = json.loads(rawData)
            print("Packet at the beginning", str(receivedMessage))
            robot.receiver.nextPacket()

        robot.step(robot.timestep)
        transition(receivedMessage)
        print("Group size:", str(robotInfo.get("G-Size")))
