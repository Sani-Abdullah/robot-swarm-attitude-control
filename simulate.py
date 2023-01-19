from swarm.terrain import Terrain

obstacles = [(0, 25, 10, 3), (16, 10, 4, 3)] # (5, 27, 10, 3), 
# obstacles = [(3, 25, 14, 4)]
# obstacles = [(0, 25, 20, 4)]

terrain = Terrain(20, 50, obstacles)
# terrain.tester()
# terrain.plot_terrain()
terrain.animate()