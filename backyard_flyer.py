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
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)
        self.vertices=[False,
            False,
            False,
            False]
    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            altitude = -1.0 * self.local_position[2]
            if altitude > 0.95 * self.target_position[2]:
                self.calculate_box()
    def velocity_callback(self):
        if self.flight_state==States.LANDING:
            if ((self.global_position[2]-self.global_home[2])<0.1) & abs(self.local_position[2]<0.01):
                self.arming_transition()
    def state_callback(self):
        if not self.in_mission:
            return
        if self.flight_state==States.MANUAL:
            self.arming_transition()
        elif self.flight_state==States.ARMING:
            self.takeoff_transition()
        elif self.flight_state==States.DISARMING:
            self.manual_transition()
    def calculate_box(self):
        print("Local Position =",self.local_position,"\n    Target Position =",self.target_position)
        if self.vertices[0]==False:
            self.target_position[1]=10
            #self.cmd_position(north=self.target_position[0],east= self.target_position[1],altitude= self.target_position[2],heading= 0)
            self.cmd_position(*self.target_position,heading= 0)
            print("East = 10")
            if self.local_position[1]>=0.95*self.target_position[1]:
                self.vertices[0]=True
                print(self.vertices[0])
        elif self.vertices[1]==False:
            self.target_position[0]=10
            self.cmd_position(*self.target_position,heading= 0)
            print("north = 10")
            if self.local_position[0]>=0.95*self.target_position[0]:
                self.vertices[1]=True
        elif  self.vertices[2]==False:
            self.target_position[1]=0
            self.cmd_position(*self.target_position,heading= 0)
            print("east = -10")
            if self.local_position[1]<=0:
                self.vertices[2]=True
        elif self.vertices[3]==False:
            self.target_position[0]=0
            self.cmd_position(*self.target_position,heading= 0)
            print("north = -10")
            if self.local_position[0]<=0:
                self.vertices[3]=True
                self.landing_transition()
                self.reset_square()


        #self.cmd_position(north=10,east= 0,altitude= 2* self.target_position[2],heading= 0)
        #self.cmd_position(north=0,east= 0,altitude= 2* self.target_position[2],heading= 0)
    def reset_square(self):
        for e,i in enumerate(self.vertices):
            self.vertices[e]=False
            print(self.vertices)
    def arming_transition(self):
        self.take_control()
        self.arm()
        self.set_home_position(self.global_position[0],
                               self.global_position[1],
                               self.global_position[2])
        self.flight_state=States.ARMING
        print("*arming transition done")
    def takeoff_transition(self):

        target_alti =3.0
        self.target_position[2]=target_alti
        self.takeoff(target_alti)
        print("**takeoff transition")
        self.flight_state=States.TAKEOFF
    def waypoint_transition(self):
        self.calculate_box()
        self.flight_state=States.WAYPOINT
        print("***waypoint transition")
    def landing_transition(self):
        self.land()
        self.flight_state=States.LANDING
        print("****landing transition")
    def disarming_transition(self):
        self.disarm()
        self.flight_state=States.DISARMING
        print("*****disarm transition")
    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("******manual transition")

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
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
