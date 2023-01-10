from agent import Agent

class Terrain:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.agents = [Agent(i, self) for i in range(50)]

    def tester(self):
        print(self.agents[0].can_translate_on_x_axis_check())


