import argparse
import time
import msgpack
import sys

from enum import Enum, auto
import numpy as np

from planning_utils import plan_takeoff_and_landing, bresify_path, a_star, heuristic, create_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, goal):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.goal = goal

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    # modified to respect vertical waypoints (though only for going up)
    def local_position_callback(self):
        DEADPAN = 3.0
        is_near_altitude = -1.0 * self.local_position[2] > 0.95 * self.target_position[2]
        if self.flight_state == States.TAKEOFF:
            if is_near_altitude:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            is_near_destination = np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < DEADPAN
            if is_near_destination and is_near_altitude:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    # added support for disarming at altitude if close to target
    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.target_position[2] < 0.1:
                if abs(self.local_position[2]) < 0.01 + self.target_position[2]:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1],
                          self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        # print("waypoints", self.waypoints)
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        # instead of starting at flying altitude, start above wherever we are...
        # self.target_position[2] = TARGET_ALTITUDE
        self.target_position[2] = self._altitude + TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        with open('colliders.csv') as f:
            _, lat0, _, lon0 = f.readline().replace(",", " ").replace("\n", "").split()

        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(float(lon0), float(lat0), 0) 

        # TODO: retrieve current global position
        # TODO: convert to current local position using global_to_local()
        local_pos = global_to_local(self.global_position, self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # Define starting point on the grid (this is just grid center)
        # TODO: convert start position to current position rather than map center
        grid_start = (int(-north_offset + local_pos[0]), int(-east_offset + local_pos[1]))

        # Set goal as some arbitrary position on the grid
        # TODO: adapt to set goal as latitude / longitude position and convert
        # goal_lon, goal_lat, goal_alt = goal
        goal_pos = global_to_local(self.goal, self.global_home)
        grid_goal = (int(-north_offset + goal_pos[0]), int(-east_offset + goal_pos[1]))
        # grid_goal = (grid_start[0] + 10, grid_start[1] + 10)
        print('Local Start and Goal: ', grid_start, grid_goal)

        # adding support for take-off and landing at altitude
        start_point = (grid_start[0], grid_start[1], self._altitude)
        end_point = (grid_goal[0], grid_goal[1], self.goal[2])
        # print("3d points", start_point, end_point)
        starting_points, ending_points, close_to_start, close_to_end = plan_takeoff_and_landing(
            start_point, end_point, TARGET_ALTITUDE, grid)
        # print("safe_points", close_to_start, close_to_end)
        # print("starting_ending_points", starting_points, ending_points)

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)

        start = time.time()
        # print(grid_start, close_to_start, type(grid_start), type(close_to_start))
        # print(grid_goal, close_to_end, type(grid_goal), type(close_to_end))
        path, cost = a_star(grid, heuristic, close_to_start, close_to_end)
        orig_path_len = len(path)
        end = time.time()
        if (orig_path_len == 0):
            sys.exit()
        print("a_star took {0}s to find {1} points at a cost of {2}".format(end - start, len(path), cost))

        # TODO: prune path to minimize number of waypoints
        # attempt 1. collinearity
        # nevermind this is fairly pointless

        # attempt 2. bresenham
        # starting with wherever we are now
        start = time.time()
        path = bresify_path(path, grid)
        end = time.time()
        print("bresify_path took {0}s to convert {1} elements to {2}".format(end - start, orig_path_len, len(path)))

        # TODO (if you're feeling ambitious): Try a different approach altogether!
        # graphs, other stuff?

        # Convert path to waypoints
        starting_points = [[int(p[0] + north_offset), int(p[1] + east_offset), int(p[2]), 0] for p in starting_points]
        ending_points = [[int(p[0] + north_offset), int(p[1] + east_offset), int(p[2]), 0] for p in ending_points]
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]

        # Set self.waypoints
        wayps = starting_points + waypoints + ending_points
        self.waypoints = wayps

        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--goal', type=str, default='-122.396846, 37.797240, 0', help="lon/lat/alt")
    args = parser.parse_args()
    print("args", args)

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    goal = np.array(args.goal.replace(" ", "").split(",")).astype(float)
    print("goal", goal)
    # proces
    # goal = (-122.396846, 37.797240, 0)  # TODO: retrive from args?
    drone = MotionPlanning(conn, goal)
    time.sleep(1)

    drone.start()
