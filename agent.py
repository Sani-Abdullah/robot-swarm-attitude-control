import numpy as np
import config as cf
# from terrain import Terrain
from exceptions import VelocityDirectionError

class Agent:
    def __init__(self, id: int, terrain):
        self.id = id
        self.terrain = terrain
        self.titter = 0 # degrees
        self.position = np.random.randint(0, terrain.width + 1), np.random.randint(0, terrain.height + 1)
        self.velocity = cf.COMMON_VELOCITY

    def sense():
        '''<TBD>'''
        # obstacle_end_position: end_x, end_y, direction (+1 ] or -1 [)
        # distress data:
        #   - obstacle found: {type: int, }
        #   - obstacle not found: {type: int, }
        pass

    def transmit_distress(self, distress_data: dict):
        '''
        Broadcast a distress signal to all the agents in the terrain
        '''
        for agent in self.terrian.agents:
            agent.receive_distress(self.id, distress_data)

    def receive_distress(self, sender_id: int, distress_data: dict):
        '''<TBD>'''
        if sender_id != self.id:
            if distress_data['type'] == cf.DISTRESS_OBSTACLE_FOUND_SAFETY:
                pass

    def calculate_safety_position(obstacle_end_position: tuple) -> tuple:
        '''
        Node node calculate safe destination to rotate to
        '''
        obstacle_end_x, obstacle_end_y, direction = obstacle_end_position
        safety_position_x = obstacle_end_x + direction * (cf.AGENT_RADIUS + cf.OBSTACLE_ALLOWANCE)
        safety_position_y = obstacle_end_y - cf.AGENT_RADIUS - cf.OBSTACLE_ALLOWANCE
        return safety_position_x, safety_position_y
    
    def in_path_check(self, distressed_agent_safety_position: tuple) -> bool:
        '''
        Check if this agent can vertically collide with the distressed node
        '''
        self_x, _ = self.position
        distressed_agent_safety_x, _= distressed_agent_safety_position
        distressed_agent_sefty_left_boundary, distressed_agent_safety_right_boundary = distressed_agent_safety_x - cf.SAFETY_RADIUS, distressed_agent_safety_x + cf.SAFETY_RADIUS
        if self_x < distressed_agent_sefty_left_boundary or self_x > distressed_agent_safety_right_boundary:
            return False
        else:
            return True

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

    def can_translate_on_x_axis_check(self, safe_radius_units=1) -> tuple:
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

