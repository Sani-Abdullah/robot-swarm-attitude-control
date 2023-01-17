import numpy as np
import config as cf
# from terrain import Terrain
from util.exceptions import VelocityDirectionError

class Agent:
    '''
    A member agent of a swarm
    '''
    def __init__(self, id: int, terrain):
        self.id = id
        self.terrain = terrain
        self.titter = cf.NOMINAL_TITTER # degrees
        self.velocity = cf.NOMINAL_VELOCITY
        self.position = 17, 5
        self.safety_position = 0

    def sense(self):
        '''
        Scan the terrain for obstacles and wait for the swarm to decide on a directive
        '''
        pass


    def translate(self):
        '''
        Agent translates at current velocity facing titter
        '''
        # if self.safety_position != 0:
        #     self.reached_safety()
        # if not self.halted:
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

    def reached_safety(self):
        '''
        If true, the agent's velocity vector is reset to nominal
        '''
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
            if obstacles[1] - self.position[1] <= cf.OBSTACLE_PANIC_ACT_DISTANCE:
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
        widths = [hole[1] for hole in holes if hole[1] > 2 * (cf.AGENT_RADIUS + cf.OBSTACLE_ALLOWANCE)]
        best_index = np.argmin(np.abs(widths))
        best_hole = holes[best_index]
        return best_hole

    def calculate_safety_position(self, best_hole: tuple) -> tuple:
        '''
        Returns the safest point to rotate to
        '''
        safety_x = np.random.randint(best_hole[0][0] + cf.OBSTACLE_ALLOWANCE + cf.AGENT_RADIUS, best_hole[0][0] + best_hole[1] - cf.AGENT_RADIUS) + np.random.random()
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

    def time_to_arrive(self, destination: tuple, direction: str = 'free') -> float:
        '''
        How long before the forward-tip of safety radius reaches destination
        '''
        pass

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
