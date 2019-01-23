import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

        self.current_waypoint = 0
        self.set_relative_waypoint(0, 0, 3.0)

    def set_relative_waypoint(self, dx, dy, dz):
        self.all_waypoints.append((dx, dy, dz))

    def get_current_waypoint(self):
        index = self.current_waypoint
        wpt = self.all_waypoints[index]
        return (wpt[0] + self.global_home[0],
                wpt[1] + self.global_home[1],
                wpt[2] + self.global_home[2])

    def next_waypoint(self):
        if self.current_waypoint < len(self.all_waypoints):
            self.current_waypoint += 1

    def local_position_callback(self):
        """
        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """

        # get current waypoint
        wpt = self.get_current_waypoint()

        # debug print
        print(self.local_position, self.target_position, wpt)

        # process position based on flight state
        if self.flight_state == States.TAKEOFF:
            # coordinate conversion
            altitude = -1.0 * self.local_position[2]

            # check if altitude is within 95% of target
            if altitude > 0.95 * self.target_position[2]:
                # go to landing
                self.landing_transition()

        elif self.flight_state == States.WAYPOINT:
            pass
        elif self.flight_state == States.LANDING:
            # why was this in the velocity callback in the up-down exaple?
            if ((self.global_position[2] - self.global_home[2] < 0.1) and
                    abs(self.local_position[2]) < 0.01):
                self.disarming_transition()

    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        print(self.local_velocity)
        # if self.flight_state == States.LANDING:
        #     if ((self.global_position[2] - self.global_home[2] < 0.1) and
        #             abs(self.local_position[2]) < 0.01):
        #         self.disarming_transition()
        pass

    def state_callback(self):
        """
        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        print(self.flight_state, self.armed, self.guided)

        if self.flight_state == States.MANUAL:
            # wait for manual state
            if not self.guided:
                # initiate arming
                self.arming_transition()

        elif self.flight_state == States.ARMING:
            # wait until armed
            if self.armed:
                # initiate takeoff
                self.takeoff_transition()

        elif self.flight_state == States.TAKEOFF:
            pass

        elif self.flight_state == States.WAYPOINT:
            self.disarming_transition()

        elif self.flight_state == States.LANDING:
            pass

        elif self.flight_state == States.DISARMING:
            if not self.armed:
                self.manual_transition()

        else:
            print("INVALID_STATE")
            exit(1)

    def calculate_box(self):
        """TODO: Fill out this method
        
        1. Return waypoints to fly a box
        """
        pass

    def arming_transition(self):
        """T
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        self.take_control()
        self.arm()

        # set the current location to be the home position
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])

        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """TODO: Fill out this method
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        target_altitude = 3.0
        wpt = self.get_current_waypoint()
        self.target_position[2] = wpt[2]
        self.takeoff(wpt[2])
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")

    def landing_transition(self):
        """T
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    # conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    print(drone.global_home)
    drone.start()
