import os
import config as cf
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from swarm.agent import Agent
from matplotlib.patches import Rectangle


class Terrain:
    '''
    An obstacle course for the agents to traverse and conquer
    '''
    def __init__(self, width: float, height: float, obstacles: list):
        self.width = width
        self.height = height
        self.agents = [Agent(i, self) for i in range(5)]
        self.obstacles = obstacles

        # create and configure plot
        self.plot_figure = plt.figure(facecolor='white', figsize=(2, 2), dpi=300, frameon=True)
        self.plot_axis = self.plot_figure.add_subplot()
        self.plot_axis.set_ylim(0, self.height)
        self.plot_axis.set_xlim(0, self.width)
        self.plot_axis.yaxis.set_visible(False)
        self.plot_axis.xaxis.set_visible(False)
        self.plot_axis.spines['top'].set_visible(False)
        self.plot_axis.spines['bottom'].set_visible(False)
        self.plot_axis.set_facecolor('#8cc63f')

    def plot_terrain(self, animation_index):
        '''
        Plot the obstacle course and agents
        '''
        for agent in self.agents:
            agent.sense()
        for agent in self.agents:
            agent.translate()

        # self.plot_axis.spines.clear()
        for obstacle in self.obstacles:
            x, y, dx, dy = obstacle
            rectangle = Rectangle((x, y), dx, dy, fc='#764c29', ec='black', lw=0.484)
            self.plot_axis.add_patch(rectangle)
        
        agents_x = []
        agents_y = []
        for agent in self.agents:
            agents_x.append(agent.position[0])
            agents_y.append(agent.position[1])
        # print(agents_x)
        if len(self.plot_axis.lines) > 1:
            self.plot_axis.lines.pop(0)
        agent_positions = self.plot_axis.plot(agents_x, agents_y, 'o', markersize= 2 * cf.AGENT_RADIUS, c='#2e3192')
        # self.plot_axis.plot(3, 3, 'or', markersize=0.5)
        # plt.show()
        # agent_positions.set_data()
        return agent_positions

    def animate(self):
        if not os.path.exists('output'):
            os.makedirs('output')
        anim = animation.FuncAnimation(self.plot_figure, self.plot_terrain, frames=200, interval=cf.TRANSLATION_INTERVAL * 1000, blit=True)
        anim.save('output/swarm-control.mp4', writer = 'ffmpeg', fps = 20)

    def receive_distress(self, sender_id: int, distress_data: dict):
        '''
        Receive distress signal from agent and transmit to concerned agents
        '''
        distressed_agent_id = distress_data['agent_id']
        in_safey_point_path_agents = distress_data['agents_in_sp_path'] # concerned agents

        if distress_data['type'] == cf.DISTRESS_OBSTACLE_FOUND_SAFETY:
            safety_point = distress_data['safety_point']
            fastest_time = np.abs((self.agents[distressed_agent_id].position[0] - safety_point[0]) / cf.MAXIMUM_VELOCITY) + cf.SAFETY_RADIUS / cf.MAXIMUM_VELOCITY # time to cover horizontal and safety radius
            if in_safey_point_path_agents:
                if self.agents[in_safey_point_path_agents[0]].time_to_arrive(safety_point) > fastest_time: # distressed agent can reach safety point beofore others arrive
                    time_to_safety = (self.agents[in_safey_point_path_agents[0]].time_to_arrive(safety_point) - fastest_time) / 2
                    velocity_x_component = (self.agents[distressed_agent_id].position[0] - safety_point[0]) / time_to_safety # if vx is positive safety point is on the left. vice versa
                    velocity_y_component = (safety_point[1] - self.agents[distressed_agent_id].position[1]) / time_to_safety # vy is always positive. because safety point is always ahead
                    velocity_resultant = np.sqrt(np.square(velocity_x_component) + np.square(velocity_y_component))
                    rotation_angle = np.arctan(velocity_y_component / velocity_x_component) * 180 / np.pi
                    return {
                        'ts': time_to_safety,
                        'vx': velocity_x_component,
                        'vy': velocity_y_component,
                        'vr': velocity_resultant,
                        'titter': rotation_angle,
                }
                elif len(in_safey_point_path_agents) == 1:
                    time_to_safety = self.agents[in_safey_point_path_agents[0]].time_to_arrive(safety_point) + cf.SAFETY_RADIUS / cf.MAXIMUM_VELOCITY
                    velocity_x_component = (self.agents[distressed_agent_id].position[0] - safety_point[0]) / time_to_safety # if vx is positive safety point is on the left. vice versa
                    velocity_y_component = (safety_point[1] - self.agents[distressed_agent_id].position[1]) / time_to_safety # vy is always positive. because safety point is always ahead
                    velocity_resultant = np.sqrt(np.square(velocity_x_component) + np.square(velocity_y_component))
                    rotation_angle = np.arctan(velocity_y_component / velocity_x_component) * 180 / np.pi
                    return {
                        'ts': time_to_safety,
                        'vx': velocity_x_component,
                        'vy': velocity_y_component,
                        'vr': velocity_resultant,
                        'titter': rotation_angle,
                }
                else:
                    gaps = []
                    for agent_index in range(len(in_safey_point_path_agents) - 1):
                        gap = self.agents[in_safey_point_path_agents[agent_index]].position[1] - self.agents[in_safey_point_path_agents[agent_index + 1]].position[1]
                        gaps.append(gap)
                    found_wide_enough_gap = False
                    for gap_index in range(len(gaps)):
                        if gaps[gap_index] > 2 * cf.AGENT_RADIUS + 2 * cf.OBSTACLE_ALLOWANCE:
                            time_to_safety = self.agents[in_safey_point_path_agents[gap_index]].time_to_clear(safety_point) + (0.5 * gaps[gap_index]) / cf.NOMINAL_VELOCITY 
                            found_wide_enough_gap = True
                            break;

                    if not found_wide_enough_gap:
                        time_to_safety = self.agents[in_safey_point_path_agents[-1]].time_to_clear(safety_point)

                    velocity_x_component = (self.agents[distressed_agent_id].position[0] - safety_point[0]) / time_to_safety # if vx is positive safety point is on the left. vice versa
                    velocity_y_component = (safety_point[1] - self.agents[distressed_agent_id].position[1]) / time_to_safety # vy is always positive. because safety point is always ahead
                    velocity_resultant = np.sqrt(np.square(velocity_x_component) + np.square(velocity_y_component))
                    rotation_angle = np.arctan(velocity_y_component / velocity_x_component) * 180 / np.pi
                    return {
                        'ts': time_to_safety,
                        'vx': velocity_x_component,
                        'vy': velocity_y_component,
                        'vr': velocity_resultant,
                        'titter': rotation_angle,
                } 
            else:
                time_to_safety = 0.75 * fastest_time
                velocity_x_component = (self.agents[distressed_agent_id].position[0] - safety_point[0]) / time_to_safety # if vx is positive safety point is on the left. vice versa
                velocity_y_component = (safety_point[1] - self.agents[distressed_agent_id].position[1]) / time_to_safety # vy is always positive. because safety point is always ahead
                velocity_resultant = np.sqrt(np.square(velocity_x_component) + np.square(velocity_y_component))
                rotation_angle = np.arctan(velocity_y_component / velocity_x_component) * 180 / np.pi
                return {
                    'ts': time_to_safety,
                    'vx': velocity_x_component,
                    'vy': velocity_y_component,
                    'vr': velocity_resultant,
                    'titter': rotation_angle,
            }

        elif distress_data['type'] == cf.DISTRESS_OBSTACLE_NOT_FOUND_SAFETY:
            for agent in self.agents:
                if agent == self.agents[distressed_agent_id] or agent.position[1] > self.agents[distressed_agent_id].position[1]:
                    continue
                agent.velocity = 0
            dx_from_terrain_center = self.agents[distressed_agent_id].position[0] - self.width / 2
            search_direction = cf.SEARCH_DIRECTION_RIGHT if dx_from_terrain_center > 0 else cf.SEARCH_DIRECTION_LEFT
            velocity_x_component = cf.MAXIMUM_VELOCITY * 0.3
            velocity_y_component = 0
            velocity_resultant = velocity_x_component # because no y component
            rotation_angle = 0 if search_direction == cf.SEARCH_DIRECTION_RIGHT else 180
            return {
                'search_direction': search_direction,
                'vx': velocity_x_component,
                'vy': velocity_y_component,
                'vr': velocity_resultant,
                'titter': rotation_angle,
            }

    # def tester(self):
    #     print(self.agents[0].can_translate_on_x_axis_check())


