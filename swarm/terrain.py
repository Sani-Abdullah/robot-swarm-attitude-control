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
        # self.agents = [Agent(i, self, ) for i in range(5)]
        self.agents = [Agent(0, self, 5), Agent(1, self, 8), Agent(2, self, 11), Agent(3, self, 14), Agent(4, self, 17)]
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
        if len(self.plot_axis.lines) > 1:
            self.plot_axis.lines.pop(0)
        agent_positions = self.plot_axis.plot(agents_x, agents_y, 'o', markersize= 2 * cf.AGENT_RADIUS, c='#2e3192')
        # agent_positions = self.plot_axis.plot(12, 12, 'o', markersize= 2 * cf.AGENT_RADIUS, c='#ccc')
        # self.plot_axis.plot(3, 3, 'or', markersize=0.5)
        # plt.show()
        # agent_positions.set_data()
        return agent_positions

    def animate(self):
        if not os.path.exists('output'):
            os.makedirs('output')
        anim = animation.FuncAnimation(self.plot_figure, self.plot_terrain, frames=200, interval=cf.TRANSLATION_INTERVAL * 1000, blit=True)
        anim.save('output/swarm-control.mp4', writer = 'ffmpeg', fps = 40)

    def receive_distress(self, sender_id: int, distress_data: dict):
        '''
        Receive distress signal from agent and transmit to concerned agents
        '''
        pass


