import os
import config as cf
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from swarm.agent import Agent
from matplotlib.patches import Rectangle, Ellipse, Circle


class Terrain:
    '''
    An obstacle course for the agents to traverse and conquer
    '''
    def make_agents(self,  population):
        formation = lambda population: [(self.width / (population * 2)) + i for i in range(0, self.width, int(self.width/population))]
        formation = formation(population) if population <= 10 else  formation(10)
        mod_step = -2
        agents = []
        for agent_id in range(population):
            if agent_id % 10 == 0:
                mod_step += 2
                
            # agent_position = (mod_step, formation[agent_id % 10])
            agents.append({
                'id': agent_id,
                'x': formation[agent_id % 10],
                'y': cf.AGENT_RADIUS if mod_step == 0 else mod_step,
            })
        return [Agent(agent['id'], self, agent['x'], agent['y']) for agent in agents]
    
    # def init_terrain(self):
    #     self.agents = self.make_agents(self.population)
    #     for agent in self.agents:
    #         agent.init_cmap()
    #         agent.states = [cf.FORWARD_TRANSLATION]
    
    def __init__(self, width: float, height: float, population: int, obstacles: list):
        self.width = width
        self.height = height
        self.population = population
        # self.agents = [Agent(i, self, ) for i in range(5)]
        # safety_height = cf.AGENT_RADIUS + cf.OBSTACLE_ALLOWANCE
        # self.agents = [Agent(0, self, 5, safety_height), Agent(1, self, 8, 2 * safety_height), Agent(2, self, 11, 3 * safety_height), Agent(3, self, 14, safety_height), Agent(4, self, 19, 2 * safety_height), Agent(5, self, 17, 3.7 * safety_height)]
        self.agents = self.make_agents(population)
        self.obstacles = obstacles
        self.holes_time_to_arrive = {}
        self.target = { # Ellipse becomes a circle if width and height are the same
            'center': (8, 45),
            'width': 3,
            'height': 3,
            'radius': 3,
        }
        self.target_approach_slots = [False for i in range(len(self.agents))]

        self.result = {
            'population': len(self.agents),
            'detected': 0,
            'imminent': 0,
            'collisions': 0,
        }

        for agent in self.agents:
            agent.init_cmap()

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
        # self.plot_axis.spines.clear()
        for obstacle in self.obstacles:
            x, y, dx, dy = obstacle
            rectangle = Rectangle((x, y), dx, dy, fc='#764c29', ec='black', lw=0.484)
            self.plot_axis.add_patch(rectangle)
        target_ellipse = Ellipse(self.target['center'], self.target['width'], self.target['height'], fc='#a7a9ac', lw=0)
        self.plot_axis.add_patch(target_ellipse)
        # target_circle = Circle(self.target['center'], self.target['radius'], fc='#a7a9ac')
        # self.plot_axis.add_patch(target_circle)
        self.plot_axis.annotate('Target', self.target['center'], color='w', weight='light', fontsize=4.5, ha='center', va='center')

    def plot_terrain(self):
        '''
        Plot the obstacle course and agents
        '''
        for agent in self.agents:
            agent.sense()
        for agent in self.agents:
            agent.translate()

        
        agents_x = []
        agents_y = []
        for agent in self.agents:
            agents_x.append(agent.position[0])
            agents_y.append(agent.position[1])

        # if self.missions_completed_trigger():
        #     print('result: ', self.result)

        if len(self.plot_axis.lines) > 1:
            self.plot_axis.lines.pop(0)
        agent_positions = self.plot_axis.plot(agents_x, agents_y, 'o', markersize= 2 * cf.AGENT_RADIUS, c='#2e3192')
        # agent_positions = self.plot_axis.plot(12, 12, 'o', markersize= 2 * cf.AGENT_RADIUS, c='#ccc')
        # self.plot_axis.plot(3, 3, 'or', markersize=0.5)
        plt.show()
        # agent_positions.set_data()
        return agent_positions

    def animate(self):
        # if not os.path.exists('output'):
        #     os.makedirs('output')

        # anim = animation.FuncAnimation(self.plot_figure, self.plot_terrain, frames=600, interval=cf.TRANSLATION_INTERVAL * 1000, blit=True)
        # anim.save('output/swarm-control.mp4', writer = 'ffmpeg', dpi=300, fps = 40)

        # iterations = 2
        # for _ in range(iterations):
        for _ in range(600):
            for agent in self.agents:
                agent.sense()
            for agent in self.agents:
                agent.translate()
        # self.init_terrain()

        return self.results_make()



    def missions_completed_trigger(self):
        for agent in self.agents:
            if cf.APPROACHED_TARGET not in agent.states:
                return False
            return True

    def results_make(self) -> dict:
        # self.result['detected'] /= iterations
        # self.result['imminent'] /= iterations
        # self.result['collisions'] /= iterations
        print(self.result)

        return self.result

    def receive_distress(self, sender_id: int, distress_data: dict):
        '''
        Receive distress signal from agent and transmit to concerned agents
        '''
        if distress_data['type'] == cf.DISTRESS_SLOW_DOWN_IN_PATH_OBSTACLES:
            for agent in self.agents:
                if agent == self.agents[sender_id]:
                    continue
                agent.slow_down_in_safety_point_path(self.agents[sender_id])


