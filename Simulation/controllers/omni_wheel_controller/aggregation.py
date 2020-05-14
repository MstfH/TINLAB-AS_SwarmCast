# Webots imports
import time

# SwarmCast aggregation algorithm


class Aggregation:

    distanceInBetween = 4   # Distance between the robot and the object.
    sensorRange = 5         # The range of the sensor.
    TWaiting = 5            # Waiting time in sec.
    TAvoiding = 10          # Avoiding time in sec.
    ACKReceived = False
    message = "ACK"         # Name of the current message.

    # During the aggregation the robot has 2 states (behaviours). The search and the wait state.
    # The search state is when the robot is searching for another robot, so that it can start or join an aggregation with another robot.
    # The wait state is when the robot detects an object, so that it can communicate with the object.
    def search(self, robot, HELLO, ACK, PROPAGATE):
        print("Searching state")

        # TODO: moveForward() and
        # TODO: avoidObstacle()

        # If an object is detected, send an HELLO message to the object and wait for response.
        if self.distanceInBetween < self.sensorRange:
            print("Hello there...")
            HELLO["R-ID"] = robot.get("R-ID")
            # TODO: Stop moving, turn on radio, send HELLO message to detected object/robot.

            tEnd = time.time() + TWaiting
            while time.time() < tEnd:
                print("waiting for ACK respond")
                # TODO: check if ACK message is received and break out of loop, set ACKReceived to true.

            # If the object respond with an ACK message, return the message.
            if ACKReceived == True:
                print("ACK received")
                # TODO: return ACK message

            # If the object does not respond, the robot will identify the object as an obstacle and apply obstacle avoidance.
            if ACKReceived == False:
                print("ACK not received, moving on")
                tEnd = time.time() + TAvoiding
                while time.time() < tEnd:
                    print("Message system disabled to avoid obstacle")
                    # TODO: moveForward()
                    # TODO: avoidObstacle()
                    # TODO: disable message system(radio) for the time being

    def wait(self, robot, HELLO, ACK, PROPAGATE):
        print("Waiting state")
        # This timer is the lifespan of the robots in the aggregation. The timer encreases with the size of the size of the group.
        WTimer = 1 * robot.get("G-Size")
        # Name of the message received.
        message = "PROPAGATE"
        # The time the robot have the stay in the group.
        time.sleep(WTimer - 1)

        # TODO: Turn on sensor to detect neighbor(s) robots
        # TODO: Turn on radio when neighbor(s) is detected

        # If the message received is HELLO, send ACK message as a respond.
        if message == "HELLO":
            print("General kenobi...h")
            # TODO: send ACK message to sender of HELLO

        elif message == "PROPAGATE":
            print("General kenobi...p")
            if robot.get("G-ID") == PROPAGATE.get("G-ID"):
                # If the message received is PROPAGATE and the tag is joining, this robot will add the new robot to the group.
                if PROPAGATE.get("R-ID") not in robot.get("ID-List") and PROPAGATE.get("Tag") == "joining":
                    print("joining")
                    robot.get("ID-List").append(PROPAGATE.get("R-ID"))
                    print(robot.get("ID-List"))
                    newGSize = robot.get("G-Size") + 1
                    robot.update({"G-Size": newGSize})
                    # TODO: Broadcast PROPAGATE message to bots in aggregation

                # If the message received is PROPAGATE and the tag is leaving, this robot will remove the robot to the group.
                else:
                    if PROPAGATE.get("R-ID") in robot.get("ID-List") and PROPAGATE.get("Tag") == "leaving":
                        print("leaving")
                        robot.get("ID-List").remove(PROPAGATE.get("R-ID"))
                        print(robot.get("ID-List"))
                        newGSize = robot.get("G-Size") - 1
                        robot.update({"G-Size": newGSize})
                        # TODO: Broadcast PROPAGATE message to bots in aggregation

    # This function regulates the states transition (of search and wait) of the robot.
    # The robot always start in the searching state.
    # If an object is detected, the searchToWait function is called.
    # If no object is detected, the waitToSearch function is called.
    def transition(self, robot, HELLO, ACK, PROPAGATE):
        if self.distanceInBetween < self.sensorRange:
            print("to wait")
            self.searchToWait(robot, HELLO, ACK, PROPAGATE)
        elif self.distanceInBetween >= self.sensorRange and PROPAGATE.get("Tag") == "leaving":
            print("to search with tag")
            self.waitToSearch(robot, HELLO, ACK, PROPAGATE)
        else:
            print("to search")
            self.search(robot, HELLO, ACK, PROPAGATE)

    # This function regulates the steps taken during the search to wait transition.
    def searchToWait(self, robot, HELLO, ACK, PROPAGATE):
        # TODO: stop riding
        # TODO: Turn radio on for messages
        # TODO: assign new received message to message variable

        # If ACK message is received, set and add ACK attributes to robot attributes. Then send joining PROPAGATE message and enter wait state.
        if self.message == "ACK":
            print("transitioning")
            # TODO: Write a way to use only the first ACK message received

            robot["G-ID"] = ACK.get("G-ID")
            robot["G-Size"] = ACK.get("G-Size") + 1
            for x in ACK.get("ID-List"):
                if x not in robot.get("ID-List"):
                    robot.get("ID-List").append(x)

            # TODO: setup and broadcast a PROPAGATE message with "joining" tag

            self.wait(robot, HELLO, ACK, PROPAGATE)

            # TODO: set PROPAGATE Tag to "leaving"

        elif self.message == "HELLO" and HELLO.get("R-ID") < robot.get("R-ID"):
            print("transitioning")
            # TODO: setup and send ACK message to only HELLO sender
            self.wait(robot, HELLO, ACK, PROPAGATE)

            # TODO: set PROPAGATE Tag to "leaving"

    # This function regulates the steps taken during the search to wait transition.
    def waitToSearch(self, robot, HELLO, ACK, PROPAGATE):
        # TODO: broadcast a PROPAGATE message with "leaving" tag
        # TODO: set T-Avoiding and apply obstacle avoidance

        # Reset robot's attributes after leaving the group
        robot["ID-List"].clear()
        robot["ID-List"].append(robot.get("R-ID"))
        robot["G-ID"] = robot.get("R-ID")
        robot["G-Size"] = 1
        self.search(robot, HELLO, ACK, PROPAGATE)


class Robot:

    # The robots attributes (random atm for testing)
    robot = {
        "R-ID": 1,
        "ID-List": [1, 2, 3, 4, 5],
        "G-ID": 15,
        "G-Size": 1
    }

    # Messages
    HELLO = {
        "NAME": "HELLO",
        "R-ID": 0
    }

    ACK = {
        "NAME": "ACK",
        "R-ID": 0,
        "G-ID": 0,
        "G-Size": 20,
        "ID-List": [],
        "State": ""
    }

    PROPAGATE = {
        "NAME": "PROPAGATE",
        "R-ID": 0,
        "G-ID": 0,
        "Tag": ""
    }

    # TODO: Open radio to update messages with incoming messages
    aggr = Aggregation()

    # Loop to keep the aggregation algorithm running unitl the goal is reached (an aggregation of 25 robots)
    totaalAmountOfRobots = 25
    while robot.get("G-Size") < totaalAmountOfRobots:
        aggr.transition(robot, HELLO, ACK, PROPAGATE)
        ACK["G-Size"] = ACK.get("G-Size") + 1
        print(str(robot.get("G-Size")))


bot = Robot()
