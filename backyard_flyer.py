import argparse
import time
import math
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class Events(Enum):
    LOCAL_POSITION = 0
    VELOCITY = 1
    STATE = 2


# NED COORDINATE SYSTEM
# right handed (down is positive)
# X is positive north (up in simulator)
# Y is positive east  (right in simulator)
# Z is psitive down

class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.in_mission = True
        self.check_state = {}

        # set waypoint parameters
        self.waypoints = []  # list of waypoints in flight order
        self.current_waypoint = 0  # index of current waypoint
        self.wpt_dx = 1.0  # waypoint tolerance NORTH
        self.wpt_dy = 1.0  # waypoint tolerance EAST
        self.wpt_dz = 0.5  # waypoint tolerance DOWN
        self.loiter_state = 0
        self.loiter_time = time.clock()

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def set_waypoint(self, dx, dy, dz, hdg, loiter):
        """add a waypoint relative to the home position
        coordinates are NED (up is negative)
        :param dx: north
        :param dy: east
        :param dz: down
        :param hdg : heading to fly
        :param loiter : time to loiter at waypoint
        :return: none
        """
        wpt = (dx + self.global_home[0],
               dy + self.global_home[1],
               dz + self.global_home[2],
               hdg,
               loiter)

        self.waypoints.append(wpt)

    def get_current_waypoint(self):
        """get the current waypoint
            :return the waypoint at the current index
        """
        index = self.current_waypoint
        if index >= len(self.waypoints):
            raise Exception("out of waypoint bounds")
        return self.waypoints[index]

    def next_waypoint(self):
        """advance to next waypoint
            :return true if next waypoint is valid
            :return false if no more waypoints
        """
        if self.current_waypoint < len(self.waypoints):
            # advance the index
            self.current_waypoint += 1
        # return if the current waypoint is valid or not
        return self.current_waypoint < len(self.waypoints)

    def at_waypoint(self, wpt, pos):
        """check if at the waypoint
        :return true if within waypoint bounds
        :return false otherwise
        """
        dx = math.fabs(pos[0] - wpt[0])
        dy = math.fabs(pos[1] - wpt[1])
        dz = math.fabs(pos[2] - wpt[2])
        p = (dx < self.wpt_dx) and \
            (dy < self.wpt_dy) and \
            (dz < self.wpt_dz)
        if p:
            pass
        return p

    def local_position_callback(self):
        """
        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        # process the event
        self.state_handler(Events.LOCAL_POSITION)

    def velocity_callback(self):
        """
        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
        # process the event
        self.state_handler(Events.LOCAL_POSITION)

    def state_callback(self):
        """
        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        # process the event
        self.state_handler(Events.STATE)

    def calculate_box(self, dx, dy, dz, t):
        """
        compute a rectangle with 4 waypoints
        home position is lower right corner

        :param dx: north distance for one leg
        :param dy: east  distance for one leg
        :param dz: altitude (up is negative)
        :param t : time to loiter at waypoint
        """
        west = 0.5 * math.pi
        north = 0.0
        east = 0.5 * math.pi
        south = math.pi
        self.set_waypoint(0.0, -dy, dz, west, t)
        self.set_waypoint(dx, -dy, dz, north, t)
        self.set_waypoint(dx, 0.0, dz, east, t)
        self.set_waypoint(0.0, 0.0, dz, south, t)

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
        """
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        self.loiter_state = 0  # reset loiter state
        wpt = self.get_current_waypoint()  # get current position
        self.target_position[0] = wpt[0]
        self.target_position[1] = wpt[1]
        self.target_position[2] = wpt[2]
        self.wpt_loiter = wpt[4]
        # command next waypoint : correct Z to positive up (waypoint is ned)
        self.cmd_position(wpt[0], wpt[1], -wpt[2], wpt[3])
        self.flight_state = States.WAYPOINT
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

    def manual_state(self, event):
        if event == Events.LOCAL_POSITION:
            pass
        elif event == Events.VELOCITY:
            pass
        elif event == Events.STATE:
            # starting up, initiate arming
            self.arming_transition()
        else:
            print("INVALID EVENT", self.flight_state, event)

    def arming_state(self, event):
        if event == Events.LOCAL_POSITION:
            pass
        elif event == Events.VELOCITY:
            pass
        elif event == Events.STATE:
            # wait until armed then initiate takeoff
            if self.armed:
                self.takeoff_transition()
        else:
            print("INVALID EVENT", self.flight_state, event)

    def takeoff_state(self, event):
        if event == Events.LOCAL_POSITION:
            # check position and go to first waypoint if the position is in bounds
            # coordinate conversion
            altitude = -1.0 * self.local_position[2]

            # check if altitude is within 95% of target
            if altitude > 0.95 * self.target_position[2]:
                # go to landing
                self.waypoint_transition()
        elif event == Events.VELOCITY:
            pass
        elif event == Events.STATE:
            pass
        else:
            print("INVALID EVENT", self.flight_state, event)

    def waypoint_state(self, event):
        if event == Events.LOCAL_POSITION:
            # check waypoint bounds and go to next
            # loiter a few seconds at each waypoint
            wpt = self.get_current_waypoint()
            if self.loiter_state == 0:
                if self.at_waypoint(wpt, self.local_position):
                    # start a timer
                    self.loiter_time = time.clock()
                    self.loiter_state = 1
            elif self.loiter_state == 1:
                t = time.clock()
                # wait for waypoint loiter time to expire
                if (t - self.loiter_time) >= wpt[4]:
                    # if there is a next waypoint
                    has_next = self.next_waypoint()
                    if has_next:
                        # update waypoint and fly to it
                        self.waypoint_transition()
                    else:
                        # land
                        self.landing_transition()

        elif event == Events.VELOCITY:
            pass
        elif event == Events.STATE:
            pass
        else:
            print("INVALID EVENT", self.flight_state, event)

    def landing_state(self, event):
        if event == Events.LOCAL_POSITION:
            # check position and go to disarming if in touchdown bounds
            # why was this in the velocity callback in the up-down exaple?
            if ((self.global_position[2] - self.global_home[2] < 0.1) and
                    abs(self.local_position[2]) < 0.01):
                self.disarming_transition()
        elif event == Events.VELOCITY:
            pass
        elif event == Events.STATE:
            pass
        else:
            print("INVALID EVENT", self.flight_state, event)

    def disarming_state(self, event):
        if event == Events.LOCAL_POSITION:
            pass
        elif event == Events.VELOCITY:
            pass
        elif event == Events.STATE:
            # shutdown
            self.manual_transition()
        else:
            print("INVALID EVENT", self.flight_state, event)

    def state_handler(self, event):
        # get current state
        state = self.flight_state

        # process the event for the current state
        if state == States.MANUAL:
            self.manual_state(event)
        elif state == States.ARMING:
            self.arming_state(event)
        elif state == States.TAKEOFF:
            self.takeoff_state(event)
        elif state == States.WAYPOINT:
            self.waypoint_state(event)
        elif state == States.LANDING:
            self.landing_state(event)
        elif state == States.DISARMING:
            self.disarming_state(event)
        else:
            print("INVALID STATE")
            exit(1)

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

    # set the flightplan
    # parameters for a box are:
    # 1: x distance for legs
    # 2: y distance for legs
    # 3: down position for legs
    # 4: time to loiter at each waypoint
    # the box starts and ends at global_home
    drone.calculate_box(10.0, 10.0, -3.0, 5.0)

    time.sleep(2)
    print(drone.global_home)
    drone.start()
