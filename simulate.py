from swarm.terrain import Terrain

obstacles = [(16, 10, 4, 5), (0, 35, 10, 7)]
# obstacles = [(3, 25, 14, 4)]
terrain = Terrain(20, 50, obstacles)
# terrain.tester()
# terrain.plot_terrain()
terrain.animate()