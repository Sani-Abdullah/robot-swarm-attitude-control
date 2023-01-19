import numpy as np
import config as cf
# from terrain import Terrain
from util.exceptions import VelocityDirectionError
from util.decorators import halting_disabled
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
        self.avoiding_position = 0
        self.approach_target_position = 0

        self.states = [cf.FORWARD_TRANSLATION]

    def sense(self):
        '''
        Scan the terrain for obstacles and wait for the swarm to decide on a directive
        '''
        self.approach_target()
        obstacles = self.get_obstacles()
        if obstacles and cf.DODGING_OBSTACLE not in self.states and cf.APPROACHING_TARGET not in self.states and cf.APPROACHED_TARGET not in self.states:
            holes = self.get_holes(obstacles)
            self.transmit_distress({})
            if holes:
                best_hole = self.get_best_hole(holes)
                safety_position = self.calculate_safety_position(best_hole)
                # self.slow_down_in_safety_point_path()
                velocity_x = np.random.random() * cf.MAXIMUM_VELOCITY
                velocity_x, velocity_y = self.obstacle_dodging_velocity(safety_position, best_hole[0])
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
            pass

    def translate(self, translation_interval = cf.TRANSLATION_INTERVAL):
        '''
        Agent translates at current velocity facing titter
        '''
        if cf.HALTING in self.states:
            self.velocity = 0
            return
        self.reached_target_do()
        self.reached_safety_do()
        self.reached_avoiding_point_do()
        velocity_x_component = self.velocity * np.cos(self.titter / (180 / np.pi))
        velocity_y_component = self.velocity * np.sin(self.titter / (180 / np.pi))
        # recall s = vt
        self.position = self.position[0] + velocity_x_component * translation_interval, self.position[1] + velocity_y_component * translation_interval

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
        # try:
        #     if self.position[1] >= self.best_hole[0][1] + 0.001:
        #         self.states.remove(cf.FORWARD_TRANSLATION_AVOIDING)
        #         self.velocity = cf.NOMINAL_VELOCITY
        #         del self.best_hole
        # except AttributeError:
        #     pass
        # except ValueError:
        #     pass

    def reached_avoiding_point_do(self):
        if self.avoiding_position != 0 and self.position[1] >= self.avoiding_position[1] + 0.001:
            self.states.remove(cf.FORWARD_TRANSLATION_AVOIDING)
            self.states.append(cf.FORWARD_TRANSLATION)
            self.velocity = cf.NOMINAL_VELOCITY
            self.titter = cf.NOMINAL_TITTER
            self.avoiding_position = 0

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
            if sorted_obstacles[obstacle_index][0] - sorted_obstacles[obstacle_index + 1][0]:
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
        if cf.FORWARD_TRANSLATION_AVOIDING not in self.states and cf.DODGING_OBSTACLE not in self.states:
            obstacles = self.get_obstacles()
            for obstacle in self.terrain.obstacles:
                if obstacle[1] - self.position[1] <= cf.OBSTACLE_PANIC_ACT_DISTANCE and self.titter == 90:
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
                    time_to_arrive = sy / cf.NOMINAL_VELOCITY
                    time_to_arrive_date = datetime.now() + timedelta(seconds=time_to_arrive)
                    self.terrain.holes_time_to_arrive[best_hole[0]] = time_to_arrive_date.timestamp()
                    y_velocity = sy / time_to_arrive # the same as nominal velocity
                    self.velocity = y_velocity
                else:
                    if datetime.now() - datetime.fromtimestamp(self.terrain.holes_time_to_arrive[best_hole[0]]) > timedelta(seconds = sa / cf.NOMINAL_VELOCITY): # if the previous agent has arrived
                        time_to_arrive = sy / cf.NOMINAL_VELOCITY
                        time_to_arrive_date = datetime.now() + timedelta(seconds=time_to_arrive)
                        self.terrain.holes_time_to_arrive[best_hole[0]] = time_to_arrive_date.timestamp()
                        y_velocity = sy / time_to_arrive # the same as nominal velocity
                        self.velocity = y_velocity
                    # else:
                    #     time_to_arrive = (sy + 2.5 * sa) / cf.NOMINAL_VELOCITY
                    #     time_to_arrive_date = datetime.now() + timedelta(seconds=time_to_arrive)
                    #     self.terrain.holes_time_to_arrive[best_hole[0]] = time_to_arrive_date.timestamp()
                    #     y_velocity =  sy / time_to_arrive
                    #     y_velocity = 0.6 * cf.NOMINAL_VELOCITY

                        # c = self.position[1] - np.tan(self.titter / (180 / np.pi)) * self.position[0]
                        # yn = lambda crossing_agent_position_x: np.tan(self.titter / (180 / np.pi)) * crossing_agent_position_x + c
                        # tsa = lambda velocity_y_component: 2 * (cf.AGENT_RADIUS + cf.OBSTACLE_ALLOWANCE) / velocity_y_component
                        # # tn = lambda crossing_agent: (yn(crossing_agent.position[0]) - crossing_agent.position[1]) / np.abs(np.sin(self.titter / (180 / np.pi)) * self.velocity)
                        # Tn = lambda crossing_agent_position_x: np.abs(crossing_agent_position_x - self.position[0]) /  np.abs(np.cos(self.titter / (180 / np.pi)) * self.velocity)
                        # y_velocity = (yn(self.position[0]) - self.position[1]) / time_to_arrive
                        # self.avoiding_position = self.position[0], yn(self.position[0])
                        # self.states.append(cf.FORWARD_TRANSLATION_AVOIDING)
                        # print(y_velocity)
                # self.states.append(cf.FORWARD_TRANSLATION_AVOIDING)
                # self.velocity = y_velocity

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


    # @halting_disabled()
    def slow_down_in_safety_point_path(self, distressed_agent) -> list:
        '''
        Returns a list of all the agents that are on a collision course to the dodging agent. Starting from the closest
        '''
        # if cf.HALTING in self.states:
        c = self.position[1] - np.tan(self.titter / (180 / np.pi)) * self.position[0]
        yn = lambda crossing_agent_position_x: np.tan(self.titter / (180 / np.pi)) * crossing_agent_position_x + c
        tsa = lambda velocity_y_component: 2 * (cf.AGENT_RADIUS + cf.OBSTACLE_ALLOWANCE) / velocity_y_component
        tn = lambda crossing_agent: (yn(crossing_agent.position[0]) - crossing_agent.position[1]) / np.abs(np.sin(self.titter / (180 / np.pi)) * self.velocity)
        Tn = lambda crossing_agent_position_x: np.abs(crossing_agent_position_x - self.position[0]) /  np.abs(np.cos(self.titter / (180 / np.pi)) * self.velocity)

        # slow_down_agents = []
        # for agent in self.terrain.agents:
        #     if agent == self:
        #         continue
        #     tnC = tn(agent)
        #     tsaC = tsa(np.abs(np.sin(agent.titter / (180 / np.pi)) * agent.velocity))
        #     TnC = Tn(agent.position[0])
        #     if np.tan(self.titter / (180 / np.pi)) * (agent.position[0] - self.position[0]) < 0 and tnC + tsaC < TnC: # if the agent is crossing the dodging agents path and it will reach the path of the dodging agent before it passes
        #         slow_down_agents.append(agent)
        # for agent in slow_down_agents:
        #     agent.velocity = (yn(agent.position[0]) - agent.position[1]) / (Tn(agent.position[0]) + tsa(np.abs(np.sin(agent.titter / (180 / np.pi)) * agent.velocity)))
        #     agent.avoiding_position = agent.position[0], yn(agent.position[0])
        #     agent.states.append(cf.FORWARD_TRANSLATION_AVOIDING)
        
        tnC = tn(self)
        tsaC = tsa(np.abs(np.sin(self.titter / (180 / np.pi)) * self.velocity))
        TnC = Tn(self.position[0])
        if np.tan(distressed_agent.titter / (180 / np.pi)) * (self.position[0] - distressed_agent.position[0]) < 0 and tnC + tsaC < TnC: # if the self is crossing the dodging agents path and it will reach the path of the dodging self before it passes
            self.velocity = (yn(self.position[0]) - self.position[1]) / (Tn(self.position[0]) + tsa(np.abs(np.sin(self.titter / (180 / np.pi)) * self.velocity)))
            self.avoiding_position = self.position[0], yn(self.position[0])
            self.states.append(cf.FORWARD_TRANSLATION_AVOIDING)
        
    def retard_in_safety_point_path_agents(self):
        '''
        Slow down agents in collision course with dodging agent
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
    
    def obstacle_dodging_velocity(self, destination: tuple, hole_start_position: tuple) -> float:
        sy = destination[1] - self.position[1]
        sa = 3 * (cf.AGENT_RADIUS + cf.OBSTACLE_ALLOWANCE) # agent safety distance
        if hole_start_position not in self.terrain.holes_time_to_arrive:
            time_to_arrive = sy / cf.NOMINAL_VELOCITY
            time_to_arrive_date = datetime.now() + timedelta(seconds=time_to_arrive)
            self.terrain.holes_time_to_arrive[hole_start_position] = time_to_arrive_date.timestamp()
            y_velocity = sy / time_to_arrive # the same as nominal velocity
            x_velocity = cf.NOMINAL_VELOCITY
        else:
            if datetime.now() - datetime.fromtimestamp(self.terrain.holes_time_to_arrive[hole_start_position]) > timedelta(seconds = sa / cf.NOMINAL_VELOCITY): # if the previous agent has arrived
                time_to_arrive = sy / cf.NOMINAL_VELOCITY
                time_to_arrive_date = datetime.now() + timedelta(seconds=time_to_arrive)
                self.terrain.holes_time_to_arrive[hole_start_position] = time_to_arrive_date.timestamp()
                y_velocity = sy / time_to_arrive # the same as nominal velocity
                x_velocity = cf.NOMINAL_VELOCITY
            else:
                time_to_arrive = (sy + sa) / cf.NOMINAL_VELOCITY
                time_to_arrive_date = datetime.now() + timedelta(seconds=time_to_arrive)
                self.terrain.holes_time_to_arrive[hole_start_position] = time_to_arrive_date.timestamp()
                y_velocity =  sy / time_to_arrive
                x_velocity = 0.3 * cf.NOMINAL_VELOCITY


        return x_velocity, y_velocity

    def approach_target(self):
        obstacles = [obstacle for obstacle in self.terrain.obstacles]
        sorted_obstacles = sorted(obstacles, key=lambda x: x[1], reverse=True)
        last_obstacle_top_y = sorted_obstacles[0][1] + sorted_obstacles[0][-1]
        if self.position[1] > last_obstacle_top_y and cf.APPROACHING_TARGET not in self.states and cf.APPROACHED_TARGET not in self.states and cf.FORWARD_TRANSLATION in self.states:
            self.states.append(cf.APPROACHING_TARGET)
            # dydx = (self.terrain.target['center'][1] - self.position[1]) / (self.terrain.target['center'][0] - self.position[0])
            # titter = np.arctan(dydx) * 180 / np.pi
            # self.titter = np.abs(titter) if dydx > 0 else 180 + titter

            r = self.terrain.target['width'] # + cf.SAFETY_RADIUS + cf.OBSTACLE_ALLOWANCE # agent target convergence radius
            if self.terrain.target['center'][0] - self.position[0] > 0: # agent is on the left of target
                titter_i = lambda i: 180 - i * 180 / len(self.terrain.agents) # the angle of approach from the perspective of the target center
                approach_position_x = lambda i: self.terrain.target['center'][0] + r * np.cos(titter_i(i) / (180 / np.pi))
                approach_position_y = lambda i: self.terrain.target['center'][1] - r * np.sin(titter_i(i) / (180 / np.pi))
                for slot_index in range(len(self.terrain.target_approach_slots)):
                    if not self.terrain.target_approach_slots[slot_index]:
                        self.terrain.target_approach_slots[slot_index] = True
                        self.approach_target_position = approach_position_x(slot_index), approach_position_y(slot_index)
                        dy = self.approach_target_position[1] - self.position[1] 
                        dx = self.approach_target_position[0] - self.position[0] 
                        self.titter = np.arctan(dy/dx) * 180 / np.pi
                        print('left: ', self.titter)
                        print('left: ', self.terrain.target_approach_slots)
                        break
            else: # agent is on the right of target
                titter_i = lambda i: i * 180 / len(self.terrain.agents) # the angle of approach from the perspective of the target center
                approach_position_x = lambda i: self.terrain.target['center'][0] + r * np.cos(titter_i(i) / (180 / np.pi))
                approach_position_y = lambda i: self.terrain.target['center'][1] - r * np.sin(titter_i(i) / (180 / np.pi))
                for slot_index in range(len(self.terrain.target_approach_slots)):
                    true_index = (slot_index + 1) * -1
                    if not self.terrain.target_approach_slots[true_index]:
                        self.terrain.target_approach_slots[true_index] = True
                        self.approach_target_position = approach_position_x(slot_index), approach_position_y(slot_index)
                        dy = self.approach_target_position[1] - self.position[1] 
                        dx = self.approach_target_position[0] - self.position[0] 
                        self.titter = 180 + np.arctan(dy/dx) * 180 / np.pi
                        print('right: ', self.titter)
                        print('right: ', self.terrain.target_approach_slots)
                        break
    
    def reached_target_do(self):
        if cf.APPROACHING_TARGET in self.states:
            # dy = self.approach_target_position[1] - self.position[1] 
            # dx = self.approach_target_position[0] - self.position[0] 
            # distance = np.sqrt(np.square(dx) + np.square(dy))
            if self.approach_target_position[1] - self.position[1] < 0.001:
                # self.translate(translation_interval=np.random.randint(0, 2) + np.random.random())
                self.states.remove(cf.APPROACHING_TARGET)
                self.states.append(cf.APPROACHED_TARGET)
                self.states.append(cf.HALTING)
                print('stopped')




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
