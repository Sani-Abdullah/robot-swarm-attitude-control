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
        self.target_approach_step_count = 0
        # self.target_approach_slots = [False for i in range(len(self.agents))]
        self.target_approach_slots = [False for i in range(5)]

        self.result = {
            'population': len(self.agents),
            'detected': 0,
            'imminent': 0,
            'collisions': 0,
        }

        for agent in self.agents:
            agent.init_cmap()

        self.plot_count = 0
        self.expedition_collision = True

        # create and configure plot
        self.plot_figure = plt.figure(facecolor='white', dpi=300, frameon=False)
        self.plot_figure.add_subplot(2, 3, 1)
        self.plot_figure.add_subplot(2, 3, 2)
        self.plot_figure.add_subplot(2, 3, 3)
        self.plot_figure.add_subplot(2, 3, 4)
        self.plot_figure.add_subplot(2, 3, 5)
        self.plot_figure.add_subplot(2, 3, 6)

        axis_time = 0
        for axis in self.plot_figure.axes:
            axis.set_ylim(0, self.height)
            axis.set_xlim(0, self.width)
            axis.yaxis.set_visible(False)
            axis.xaxis.set_visible(False)
            axis.spines['top'].set_visible(False)
            axis.spines['bottom'].set_visible(False)
            axis.set_facecolor('#8cc63f')
            # axis.set_title(f'time = {axis_time}', fontsize=9, fontweight=1)
            axis.set_title(label=f'time = {axis_time}',fontsize=8, fontweight=1, pad='5.0', loc='left', fontstyle='italic', backgroundcolor='#fbb040', color='black', fontname='Times New Roman')
            axis_time += 1
        # self.plot_axis.spines.clear()

        # adding obstacles
        for axis in self.plot_figure.axes:
            for obstacle in self.obstacles:
                x, y, dx, dy = obstacle
                rectangle = Rectangle((x, y), dx, dy, fc='#764c29', ec='black', lw=0.484)
                axis.add_patch(rectangle)

        # adding the target
        for axis in self.plot_figure.axes:
            target_ellipse = Ellipse(self.target['center'], self.target['width'], self.target['height'], fc='#a7a9ac', lw=0)
            axis.add_patch(target_ellipse)
            axis.annotate('Target', self.target['center'], color='w', weight='light', fontsize=4.5, ha='center', va='center')
        # target_circle = Circle(self.target['center'], self.target['radius'], fc='#a7a9ac')
        # self.plot_axis.add_patch(target_circle)

    def plot_terrain(self):
        '''
        Plot the obstacle course and agents
        '''
        # plot_count = 0
        # while self.expedition_collision:
        #     collided_count  = 0
        for animation_index in range(500):
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

            # if len(self.plot_axis.lines) > 1:
            #     self.plot_axis.lines.pop(0)
            # agent_positions = self.plot_axis.plot(agents_x, agents_y, 'o', markersize= 2 * cf.AGENT_RADIUS, c='#2e3192')

            # agent_positions = self.plot_axis.plot(12, 12, 'o', markersize= 2 * cf.AGENT_RADIUS, c='#ccc')
            # self.plot_axis.plot(3, 3, 'or', markersize=0.5)
            if animation_index % 83 == 0:
                self.plot_figure.axes[self.plot_count].plot(agents_x, agents_y, 'o', markersize= 2 * cf.AGENT_RADIUS, c='#2e3192')
                self.plot_count += 1

                # euclidean_distance = lambda a, b: np.sqrt(np.square(a[0] - b[0]) + np.square(a[1] - b[1]))
                # for agent in self.agents:
                #     neighbours = agent.get_neighbours()
                #     for neighbour in neighbours:
                #         if euclidean_distance(agent.position, neighbour.position) < 0.8:
                #             collided_count += 1
                if self.plot_count > 5:
                    break

        # if collided_count == 0:
        #     self.expedition_collision = False
        # else:
        #     self.plot_count = 0

        plt.show()
            # agent_positions.set_data()
            # return agent_positions

    def animate(self):
        # if not os.path.exists('output'):
        #     os.makedirs('output')

        anim = animation.FuncAnimation(self.plot_figure, self.plot_terrain, frames=600, interval=cf.TRANSLATION_INTERVAL * 1000, blit=True)
        # anim.save('output/swarm-control.mp4', writer = 'ffmpeg', dpi=300, fps = 40)

        # for _ in range(600):
        #     for agent in self.agents:
        #         agent.sense()
        #     for agent in self.agents:
        #         agent.translate()

        # return self.results_make()



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


