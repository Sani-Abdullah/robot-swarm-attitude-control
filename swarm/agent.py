import numpy as np
import config as cf
# from terrain import Terrain
from util.exceptions import VelocityDirectionError
from datetime import datetime, timedelta

class Agent:
    '''
    A member agent of a swarm
    '''
    def __init__(self, id: int, terrain, x, y):
        self.id = id
        self.terrain = terrain
        self.titter = cf.NOMINAL_TITTER # degrees
        self.velocity = cf.NOMINAL_VELOCITY
        # self.position = 17, 5
        self.position = x, y
        # np.random.randint(2, terrain.width - 2), np.random.randint(2, 5)
        self.safety_position = 0

        self.states = [cf.FORWARD_TRANSLATION]

    def sense(self):
        '''
        Scan the terrain for obstacles and wait for the swarm to decide on a directive
        '''
        obstacles = self.get_obstacles()
        if obstacles and cf.DODGING_OBSTACLE not in self.states:
            holes = self.get_holes(obstacles)
            if holes:
                best_hole = self.get_best_hole(holes)
                safety_position = self.calculate_safety_position(best_hole)
                velocity_x = np.random.random() * cf.MAXIMUM_VELOCITY
                velocity_y = self.obstacle_dodging_y_velocity(safety_position, best_hole[0])
                self.velocity = np.sqrt(np.square(velocity_x) + np.square(velocity_y))
                dydx = (self.position[1] - safety_position[1]) / (self.position[0] - safety_position[0])
                titter = np.arctan(dydx) * 180 / np.pi
                self.titter = np.abs(titter) if dydx > 0 else 180 + titter
            else:
                self.states.append(cf.HALTING)
                self.velocity = 0
                pass
        else:
            self.no_obstacle_hole_appraoch()
    def translate(self):
        '''
        Agent translates at current velocity facing titter
        '''
        self.reached_safety_do()
        velocity_x_component = self.velocity * np.cos(self.titter / (180 / np.pi))
        velocity_y_component = self.velocity * np.sin(self.titter / (180 / np.pi))

        # recall s = vt
        self.position = self.position[0] + velocity_x_component * cf.TRANSLATION_INTERVAL, self.position[1] + velocity_y_component * cf.TRANSLATION_INTERVAL

    def directive_complier(self, directive: dict):
        '''
        Act in accordance to a given directive
        '''
        self.titter = directive['titter']
        self.velocity = directive['vr']

    def reached_safety_do(self):
        '''
        If true, the agent's velocity vector is reset to nominal
        '''
        if self.safety_position != 0 and self.position[1] >= self.safety_position[1] + 0.001:
            self.states.append(cf.FORWARD_TRANSLATION)
            self.states.remove(cf.DODGING_OBSTACLE)
            self.velocity = cf.NOMINAL_VELOCITY
            self.titter = cf.NOMINAL_TITTER
            self.safety_position = 0
        try:
            if self.position[1] >= self.best_hole[1] + 0.001:
                self.states.remove(cf.FORWARD_TRANSLATION_AVOIDING)
                self.velocity = cf.NOMINAL_VELOCITY
                del self.best_hole
        except AttributeError:
            pass
        except ValueError:
            pass

    def transmit_distress(self, distress_data: dict) -> dict:
        '''
        Broadcast a distress signal to all the agents in the terrain
        '''
        # for agent in self.terrian.agents:
        #     if agent == self:
        #         continue
        #     agent.receive_distress(self.id, distress_data)
        return self.terrain.receive_distress(self.id, distress_data)

    def get_obstacles(self) -> list:
        '''
        Check for obstacles within sensing range
        '''
        obstacles = []
        for obstacle in self.terrain.obstacles:
            agent_safety_x_l = self.position[0] + cf.AGENT_RADIUS + cf.OBSTACLE_ALLOWANCE # count as obstacle if safety radius is in the obstacle - left side
            agent_safety_x_r = self.position[0] - cf.AGENT_RADIUS - cf.OBSTACLE_ALLOWANCE # count as obstacle if safety radius is in the obstacle - right side
            if obstacle[1] - self.position[1] <= cf.OBSTACLE_PANIC_ACT_DISTANCE and agent_safety_x_l > obstacle[0] and agent_safety_x_r < obstacle[0] + obstacle[2]: # if and only if obstacle is the agent's vicinity
                obstacles.append(obstacle)
        sorted_obstacles = sorted(obstacles, key=lambda  x: x[0])
        return sorted_obstacles

    def get_holes(self, sorted_obstacles: list) -> list:
        '''
        Returns the details of the openings between obstacles
        
        Hole and obstacle archtecture:
        hole_0 [obatacle_0] hole_1 [obtacle_1] hole_2 [obstacle_2] .... [obstacle_n] hole_n+1
        
        Note: hole_0 and hole_n+1 could be empty
        '''
        holes = []
        if sorted_obstacles[0][0] != 0: # add leftmost hole if it exists
            hole_start_position = 0, sorted_obstacles[0][1]
            hole_width = sorted_obstacles[0][0] - 0
            hole_distance_from_self = self.position[0] - (hole_start_position[0] + hole_width / 2)
            holes.append((hole_start_position, hole_width, hole_distance_from_self))
        for obstacle_index in range(len(sorted_obstacles) - 1): # holes in between the obstacles
            hole_start_position = sorted_obstacles[obstacle_index][0] + sorted_obstacles[obstacle_index][2], sorted_obstacles[obstacle_index][1]
            hole_width = sorted_obstacles[obstacle_index + 1][0] - hole_start_position[0]
            hole_distance_from_self = self.position[0] - (hole_start_position[0] + hole_width / 2)
            holes.append((hole_start_position, hole_width, hole_distance_from_self))
        if sorted_obstacles[-1][0] + sorted_obstacles[-1][2] < self.terrain.width: # add rightmost hole if it exists
            hole_start_position = sorted_obstacles[-1][0] + sorted_obstacles[-1][2], sorted_obstacles[-1][1]
            hole_width = self.terrain.width - hole_start_position[0]
            hole_distance_from_self = self.position[0] - (hole_start_position[0] + hole_width / 2)
            holes.append((hole_start_position, hole_width, hole_distance_from_self))
        if holes:
            return holes
        return []

    def get_best_hole(self, holes) -> tuple:
        '''
        Return the hole that is closest to the agent and wide enough
        '''
        wide_enough_holes = [hole for hole in holes if hole[1] > 2 * (cf.AGENT_RADIUS + cf.OBSTACLE_ALLOWANCE)] # first select holes that the agent can pass freely
        sorted_holes = sorted(wide_enough_holes, key=lambda x: np.abs(x[2])) # sort the holes starting from the closest to the agent
        self.best_hole = sorted_holes[0]
        return self.best_hole

    def no_obstacle_hole_appraoch(self):
        if cf.FORWARD_TRANSLATION_AVOIDING not in self.states:
            obstacles = []
            for obstacle in self.terrain.obstacles:
                if obstacle[1] - self.position[1] <= cf.OBSTACLE_PANIC_ACT_DISTANCE:
                    obstacles.append(obstacle)
            sorted_obstacles = sorted(obstacles, key=lambda  x: x[0])
            if sorted_obstacles:
                holes = self.get_holes(sorted_obstacles=sorted_obstacles)
                best_hole = self.get_best_hole(holes)

                sy = best_hole[0][1] - self.position[1]
                sa = 3 * (cf.AGENT_RADIUS + cf.OBSTACLE_ALLOWANCE) # agent safety distance


                # time_to_arrive = (sy + sa) / cf.NOMINAL_VELOCITY
                # time_to_arrive_date = datetime.now() + timedelta(seconds=time_to_arrive)
                # self.terrain.holes_time_to_arrive[best_hole[0]] = time_to_arrive_date

                if best_hole[0] not in self.terrain.holes_time_to_arrive:
                    time_to_arrive = (sy - 8 * sa) / cf.NOMINAL_VELOCITY
                    time_to_arrive_date = datetime.now() + timedelta(seconds=time_to_arrive)
                    self.terrain.holes_time_to_arrive[best_hole[0]] = time_to_arrive_date.timestamp()
                    y_velocity = sy / time_to_arrive # the same as nominal velocity
                else:
                    if datetime.now() - datetime.fromtimestamp(self.terrain.holes_time_to_arrive[best_hole[0]]) > timedelta(seconds = sa / cf.NOMINAL_VELOCITY): # if the previous agent has arrived
                        time_to_arrive = (sy - 8 * sa) / cf.NOMINAL_VELOCITY
                        time_to_arrive_date = datetime.now() + timedelta(seconds=time_to_arrive)
                        self.terrain.holes_time_to_arrive[best_hole[0]] = time_to_arrive_date.timestamp()
                        y_velocity = sy / time_to_arrive # the same as nominal velocity
                    else:
                        time_to_arrive = sy / cf.NOMINAL_VELOCITY
                        time_to_arrive_date = datetime.now() + timedelta(seconds=time_to_arrive)
                        self.terrain.holes_time_to_arrive[best_hole[0]] = time_to_arrive_date.timestamp()
                        y_velocity =  sy / time_to_arrive
                self.states.append(cf.FORWARD_TRANSLATION_AVOIDING)
                self.velocity = y_velocity


        

    def calculate_safety_position(self, best_hole: tuple) -> tuple:
        '''
        Returns the safest point to rotate to
        '''
        self.states.append(cf.DODGING_OBSTACLE)
        self.states.remove(cf.FORWARD_TRANSLATION)
        safety_x_spread_margin = 5 * (cf.OBSTACLE_ALLOWANCE + cf.AGENT_RADIUS) if best_hole[1] > self.terrain.width / 2 else 2 * (cf.OBSTACLE_ALLOWANCE + cf.AGENT_RADIUS)
        try:
            safety_x = np.random.randint(best_hole[0][0] + safety_x_spread_margin, best_hole[0][0] + best_hole[1] - safety_x_spread_margin) + np.random.random()
        except ValueError:
            safety_x = best_hole[0][0] + best_hole[1] / 2
        safety_y = best_hole[0][1]
        self.safety_position = safety_x, safety_y
        return self.safety_position
    
    def in_path_check(self, distressed_agent_safety_position: tuple) -> bool:
        '''
        Check if this agent is on a collision course to the distressed safety point
        '''
        pass


    def get_in_safety_point_path_agents(self) -> list:
        '''
        Returns a list of all the agents that are on a collision course to the distress safety point. Starting from the closest
        '''
        pass
            
    def position_request(self) -> dict:
        '''
        Get the positions of all agents in the terrain -> {node_id: position}
        '''
        pass

    # def time_to_arrive_safety_point(self, destination: tuple, hole_queue_length: int = 0) -> float:
    #     '''
    #     How long before the forward-tip of safety radius reaches destination
    #     '''
    #     sy = destination[1] - self.position[1]
    #     sa = 2 * (cf.AGENT_RADIUS + cf.OBSTACLE_ALLOWANCE) # agent safety distance
    #     time_to_arrive = (sy + sa * hole_queue_length) / cf.NOMINAL_VELOCITY
    #     return time_to_arrive
    
    def obstacle_dodging_y_velocity(self, destination: tuple, hole_start_position: tuple) -> float:
        sy = destination[1] - self.position[1]
        sa = 3 * (cf.AGENT_RADIUS + cf.OBSTACLE_ALLOWANCE) # agent safety distance
        if hole_start_position not in self.terrain.holes_time_to_arrive:
            time_to_arrive = sy / cf.NOMINAL_VELOCITY
            time_to_arrive_date = datetime.now() + timedelta(seconds=time_to_arrive)
            self.terrain.holes_time_to_arrive[hole_start_position] = time_to_arrive_date.timestamp()
            y_velocity = sy / time_to_arrive # the same as nominal velocity
        else:
            if datetime.now() - datetime.fromtimestamp(self.terrain.holes_time_to_arrive[hole_start_position]) > timedelta(seconds = sa / cf.NOMINAL_VELOCITY): # if the previous agent has arrived
                time_to_arrive = sy / cf.NOMINAL_VELOCITY
                time_to_arrive_date = datetime.now() + timedelta(seconds=time_to_arrive)
                self.terrain.holes_time_to_arrive[hole_start_position] = time_to_arrive_date.timestamp()
                y_velocity = sy / time_to_arrive # the same as nominal velocity
            else:
                time_to_arrive = (sy + 8* sa) / cf.NOMINAL_VELOCITY
                time_to_arrive_date = datetime.now() + timedelta(seconds=time_to_arrive)
                self.terrain.holes_time_to_arrive[hole_start_position] = time_to_arrive_date.timestamp()
                y_velocity =  sy / time_to_arrive


        return y_velocity



    def time_to_clear(self, destination: tuple, direction: str = 'free') -> float:
        '''
        How long before the rear-tip of safety radius exits destination
        '''
        pass

    def can_translate_on_x_axis_check(self, safe_radius_units: int = 1) -> tuple:
        '''
        Can this agent translate along the x axis to give way to the distressed agent?
        '''
        pass

    def get_best_end_point(self, sorted_holes: list) -> tuple:
        '''
        Return closest navigable end point to the agent or () if not found

        Hole and obstacle archtecture:
        hole_0 [obatacle_0] hole_1 [obtacle_1] hole_2 [obstacle_2] .... [obstacle_n] hole_n+1
        
        Note: hole_0 and hole_n+1 could be empty
        '''
        pass
