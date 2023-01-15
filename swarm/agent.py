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
        self.position = np.random.randint(2, terrain.width - 2), np.random.randint(2, 5)
        self.velocity = cf.NOMINAL_VELOCITY
        self.can_sense = True
        self.halted = False
        self.state = cf.FORWARD_TRANSLATION
        self.safety_position = 0

    def sense(self):
        '''
        Scan the terrain for obstacles and wait for the swarm to decide on a directive
        '''
        # remember to sense before translating
        # remember that agent velocity is the resultant

        if self.can_sense:
            blocking_obstacles = []
            for obstacle in self.terrain.obstacles:
                if np.abs(self.position[1] - obstacle[1]) <= cf.OBSTACLE_PANIC_ACT_DISTANCE and self.position[0] > obstacle[0] - cf.AGENT_RADIUS  and self.position[0] < obstacle[0] + obstacle[2] + cf.AGENT_RADIUS:
                    blocking_obstacles.append(obstacle)

            directive = {} # what to do after sensing
            if not blocking_obstacles: # if no obstacle
                velocity_resultant = self.velocity
                rotation_angle = self.titter
                directive = {
                    'type': cf.DISTRESS_NONE,
                    'vr': velocity_resultant,
                    'titter': rotation_angle,
                }
            else:
                holes = self.get_holes(blocking_obstacles)
                best_end_point = self.get_best_end_point(sorted_holes=holes)
                safety_point = self.calculate_safety_position(best_end_point)
                if best_end_point: # if obstacle(s) is/are traversible
                    distress_call_response = self.transmit_distress({
                        'type': cf.DISTRESS_OBSTACLE_FOUND_SAFETY,
                        'agent_id': self.id,
                        'safety_point': safety_point,
                        'agents_in_sp_path': self.get_in_safety_point_path_agents()
                    })
                    distress_call_response.update({'type': cf.DISTRESS_OBSTACLE_FOUND_SAFETY})
                    directive = distress_call_response
                else: # if obstacle is too long or its a complete barricade
                    # move to r or l
                    distress_call_response = self.transmit_distress({
                        'type': cf.DISTRESS_OBSTACLE_NOT_FOUND_SAFETY,
                        'agent_id': self.id,
                        'agents_in_sp_path': self.get_in_safety_point_path_agents()
                    })
                    distress_call_response.update({'type': cf.DISTRESS_OBSTACLE_NOT_FOUND_SAFETY})
                    directive = distress_call_response

            self.directive_complier(directive) # rotate to new titter and adjust velocity to new vr

    def translate(self):
        '''
        Agent translates at current velocity facing titter
        '''
        if self.safety_position != 0:
            self.reached_safety()
        if not self.halted:
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
        '''If true, the agent's velocity vector is reset to nominal'''
        if self.position[0] >= self.safety_position[0] + 0.01 and self.position[1] >= self.safety_position[1] + 0.01:
            self.velocity = cf.NOMINAL_VELOCITY
            self.titter = cf.NOMINAL_TITTER
            self.safety_position = 0

    def transmit_distress(self, distress_data: dict) -> dict:
        '''
        Broadcast a distress signal to all the agents in the terrain
        '''
        # for agent in self.terrian.agents:
        #     if agent == self:
        #         continue
        #     agent.receive_distress(self.id, distress_data)
        return self.terrain.receive_distress(self.id, distress_data)

    def calculate_safety_position(self, obstacle_end_position: tuple) -> tuple:
        '''
        Returns the safest point to rotate to
        '''
        obstacle_end_x, obstacle_end_y, direction = obstacle_end_position
        safety_position_x = obstacle_end_x + direction * (cf.AGENT_RADIUS + cf.OBSTACLE_ALLOWANCE)
        safety_position_y = obstacle_end_y - cf.AGENT_RADIUS - cf.OBSTACLE_ALLOWANCE
        self.safety_position = safety_position_x, safety_position_y
        return safety_position_x, safety_position_y
    
    def in_path_check(self, distressed_agent_safety_position: tuple) -> bool:
        '''
        Check if this agent is on a collision course to the distressed safety point
        '''
        self_x, self_y = self.position
        distressed_agent_safety_x, distressed_agent_safety_y = distressed_agent_safety_position
        distressed_agent_sefty_left_boundary, distressed_agent_safety_right_boundary = distressed_agent_safety_x - cf.SAFETY_RADIUS, distressed_agent_safety_x + cf.SAFETY_RADIUS
        if self_y - distressed_agent_safety_y <= cf.SAFETY_RADIUS: # this agent is ahead of distressed agent but not clear of its safety point
            self.position = self_x, self_y + cf.SAFETY_RADIUS - (self_y - distressed_agent_safety_y) # i.e safety_radius + distressed_y #if simplified
        if self_x < distressed_agent_sefty_left_boundary or self_x > distressed_agent_safety_right_boundary: # and self_y > (distressed_agent_safety_y + cf.SAFETY_RADIUS + cf.OBSTACLE_ALLOWANCE):
            return False
        else:
            return True


    def get_in_safety_point_path_agents(self) -> list:
        '''
        Returns a list of all the agents that are on a collision course to the distress safety point. Starting from the closest
        '''
        in_path_agents = []
        for agent in self.terrain.agents:
            if agent == self:
                continue
            if agent.in_path_check(self.position):
                in_path_agents.append(agent.id)
        if in_path_agents:
            sorted_in_path_agents = list(sorted(in_path_agents, key=lambda x: self.terrain.agents[x].position[1], reverse=True))
            return sorted_in_path_agents
        return in_path_agents
            
    def position_request(self) -> dict:
        '''
        Get the positions of all agents in the terrain -> {node_id: position}
        '''
        positions = {}
        for agent in self.terrain.agents:
            positions[agent.id] = agent.position
        return positions

    def time_to_arrive(self, destination: tuple, direction: str = 'free') -> float:
        '''
        How long before the forward-tip of safety radius reaches destination
        '''
        self_x, self_y = self.position
        destination_x, destination_y = destination
        distance_to_destination = np.sqrt(np.square(self_x - destination_x) - np.square(self_y - destination_y)) - cf.SAFETY_RADIUS
        if direction not in ['free', 'horizontal', 'vertical']: # free: in the direction of the velocity vector
            raise VelocityDirectionError('Velocity direction is incorrect')
        if direction == 'free':
            velocity = self.velocity
        elif direction == 'horizontal':
            velocity = self.velocity * np.cos(self.titter / (180 / np.pi))
        else:
            velocity = self.velocity * np.sin(self.titter / (180 / np.pi))
        time_to_arrive = distance_to_destination / velocity
        return time_to_arrive

    def time_to_clear(self, destination: tuple, direction: str = 'free') -> float:
        '''
        How long before the rear-tip of safety radius exits destination
        '''
        self_x, self_y = self.position
        destination_x, destination_y = destination
        distance_to_destination = np.sqrt(np.square(self_x - destination_x) - np.square(self_y - destination_y)) + cf.SAFETY_RADIUS
        if direction not in ['free', 'horizontal', 'vertical']: # free: in the direction of the velocity vector
            raise VelocityDirectionError('Velocity direction is incorrect')
        if direction == 'free':
            velocity = self.velocity
        elif direction == 'horizontal':
            velocity = self.velocity * np.cos(self.titter / (180 / np.pi))
        else:
            velocity = self.velocity * np.sin(self.titter / (180 / np.pi))
        time_to_clear = distance_to_destination / velocity
        return time_to_clear

    def can_translate_on_x_axis_check(self, safe_radius_units: int = 1) -> tuple:
        '''
        Can this agent translate along the x axis to give way to the distressed agent?
        '''
        agents_positions = self.position_request()
        agents_within_vicinity = {'left': [], 'right': []}
        for agent_id, position in agents_positions.items():
            x_distance = self.position[0] - position[0] # horizontal distance between self agent and this agent
            y_distance = self.position[1] - position[1] # vertical distance between self agent and this agent

            # check if there are any agents within the vicinity of this agent before translation decision is made
            if np.abs(x_distance) < ((2 * safe_radius_units + 1) * cf.SAFETY_RADIUS + cf.AGENT_RADIUS) and np.abs(y_distance) > (2 * cf.SAFETY_RADIUS + cf.AGENT_RADIUS):
                if x_distance > 0:
                    agents_within_vicinity['left'].append(agent_id)
                else:
                    agents_within_vicinity['right'].append(agent_id)
        
        if not agents_within_vicinity['left']:
            return True, 'left'
        elif not agents_within_vicinity['right']:
            return True, 'right'
        else:
            return False, 'proceed'

    def get_holes(self, obstacles: list) -> list:
        '''
        Returns the details of the openings between obstacles
        
        Hole and obstacle archtecture:
        hole_0 [obatacle_0] hole_1 [obtacle_1] hole_2 [obstacle_2] .... [obstacle_n] hole_n+1
        
        Note: hole_0 and hole_n+1 could be empty
        '''
        obstacle_xs_y = [(obstacle[0], obstacle[0] + obstacle[2], obstacle[1]) for obstacle in obstacles]
        # The two x coordinates and y coordinate of the obstacles
        print('obstacle_xs_y: ', obstacle_xs_y)
        self.sorted_obstacle_xs_y = list(sorted(obstacle_xs_y, key = lambda x: x[0]))
        print('sorted_obstacle_xs_y: ', self.sorted_obstacle_xs_y)
        holes = []
        if obstacle_xs_y:
            if self.sorted_obstacle_xs_y[0][0] > 0:
                hole_gap = self.sorted_obstacle_xs_y[0][0] - 0
                hole_centre_dx_with_agent = self.position[0] - (0 + hole_gap / 2)
                holes.append((hole_gap, hole_centre_dx_with_agent, 0))
            for hole_index in range(1, len(self.sorted_obstacle_xs_y)):
                hole_gap = self.sorted_obstacle_xs_y[hole_index][0] - self.sorted_obstacle_xs_y[hole_index - 1][1]
                # if dx is +ve then hole is on the left of the agent
                hole_centre_dx_with_agent = self.position[0] - (self.sorted_obstacle_xs_y[hole_index -1][1] + hole_gap / 2)
                holes.append((hole_gap, hole_centre_dx_with_agent, hole_index))
            if self.sorted_obstacle_xs_y[-1][1] < self.terrain.width:
                hole_gap = self.terrain.width - self.sorted_obstacle_xs_y[-1][1]
                hole_centre_dx_with_agent = self.position[0] - (self.terrain.width - self.sorted_obstacle_xs_y[-1][1])
                holes.append((hole_gap, hole_centre_dx_with_agent, len(holes))) # last index is previous length before appending
            sorted_holes = list(sorted(holes, key = lambda x: np.abs(x[1])))
            return sorted_holes
        return []

    def get_best_end_point(self, sorted_holes: list) -> tuple:
        '''
        Return closest navigable end point to the agent or (-1, -1) if not found

        Hole and obstacle archtecture:
        hole_0 [obatacle_0] hole_1 [obtacle_1] hole_2 [obstacle_2] .... [obstacle_n] hole_n+1
        
        Note: hole_0 and hole_n+1 could be empty
        '''
        for hole in sorted_holes:
            if hole[0] > 2 * (cf.AGENT_RADIUS + cf.OBSTACLE_ALLOWANCE): # if hole_gap is enough for agent to pass
                if hole[1] > 0: # if hole is on the left of the agent. use the left x (0) of the obstacle
                    best_end_point = self.sorted_obstacle_xs_y[hole[2]][0], self.sorted_obstacle_xs_y[hole[2]][2], +1
                else: # if hole is on the right of the agent. use the right x (1) of the obstacle
                    best_end_point = self.sorted_obstacle_xs_y[hole[2]][1], self.sorted_obstacle_xs_y[hole[2]][2], -1
                return best_end_point
            return ()
