from agent import Agent
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

class Terrain:
    '''
    An obstacle course for the agents to traverse and conquer
    '''
    def __init__(self, width: float, height: float, obstacles: list):
        self.width = width
        self.height = height
        self.agents = [Agent(i, self) for i in range(50)]
        self.obstacles = obstacles
        self.plot_figure = plt.figure(facecolor='white', figsize=(2, 2), dpi=300, frameon=True)
        self.plot_axis = self.plot_figure.add_subplot()

    def plot_terrain(self):
        self.plot_axis.set_ylim(0, self.height)
        self.plot_axis.set_xlim(0, self.width)
        self.plot_axis.yaxis.set_visible(False)
        self.plot_axis.xaxis.set_visible(False)
        self.plot_axis.spines['top'].set_visible(False)
        self.plot_axis.spines['bottom'].set_visible(False)
        # self.plot_axis.spines.clear()
        for obstacle in self.obstacles:
            x, y, dx, dy = obstacle
            rectangle = Rectangle((x, y), dx, dy, fc='#764c29', ec='black', lw=0.484)
            self.plot_axis.add_patch(rectangle)
        self.plot_axis.plot(3, 3, 'or', markersize=0.5)
        self.plot_axis.set_facecolor('#8cc63f')
        plt.show()
        

    def tester(self):
        print(self.agents[0].can_translate_on_x_axis_check())


