from swarm.terrain import Terrain
import os
import json

configurations = {
    'a': [(3, 20, 4, 2), (10, 16, 6, 8)],
    'a2': [(3, 25, 14, 4)],
    'b': [(0, 15, 8, 5), (12, 29, 4, 7)],
    'c': [(5, 23, 4, 4), (16, 29, 4, 11)],
    'd': [(0, 25, 10, 3), (16, 15, 4, 3)],
    'e': [(0, 25, 20, 4)],
    'f': []

}
obstacles = [(0, 25, 10, 3), (16, 10, 4, 3)]  # (5, 27, 10, 3),
# obstacles = [(3, 25, 14, 4)]
# obstacles = [(0, 25, 20, 4)]

# terrain = Terrain(width=20, height=50, population=20, obstacles=configurations['c'])
# terrain.tester()
# terrain.plot_terrain()
# result = terrain.animate()


def consolidate_result(config, result):
    # The json file path
    result_file_path = 'result/result.json'

    # Create the file if it doesn't exist
    if not os.path.exists(result_file_path):
        os.makedirs('result')
        temp_file = open(result_file_path, 'w')
        json.dump({}, temp_file)
        temp_file.close()

    # Load data from the file
    result_file = open(result_file_path, 'r')
    this_data = json.load(result_file)
    result_file.close()

    if config in this_data and str(result['population']) in this_data[config]:
        this_data[config][str(result['population'])].update(
            {
                'detected': this_data[config][str(result['population'])]['detected'] + result['detected'],
                'imminent': this_data[config][str(result['population'])]['imminent'] + result['imminent'],
                'collisions': this_data[config][str(result['population'])]['collisions'] + result['collisions'],
            }
        )
    elif config in this_data:
        this_data[config].update({
            result['population']: {
                'detected': result['detected'],
                'imminent': result['imminent'],
                'collisions': result['collisions'],
            }
        })
    else:
        this_data.update({
            config: {
                result['population']: {
                    'detected': result['detected'],
                    'imminent': result['imminent'],
                    'collisions': result['collisions'],
                }
            }
        })

    # Dump new data to the file
    with open(result_file_path, 'w') as result_file:
        json.dump(this_data, result_file)


if __name__ == '__main__':
    iterations = 20
    for configuration in ['a', 'b', 'c', 'd', 'e', 'f']:
        for population in [2, 5, 10, 20, 25, 50]:
            for _ in range(iterations):
                terrain = Terrain(width=20, height=50, population=population,
                                  obstacles=configurations[configuration])
                result = terrain.animate()
                consolidate_result(configuration, result)
