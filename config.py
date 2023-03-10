# agent states
FORWARD_TRANSLATION = 1
DODGING_OBSTACLE = 2
HALTING = 3
SEARCHING_FOR_HOLE = 4

# common velocity for all agents
NOMINAL_TITTER = 90 # degrees
NOMINAL_VELOCITY = 0.2 # m/s
MAXIMUM_VELOCITY = 0.35 # m/s

# distress types
DISTRESS_OBSTACLE_FOUND_SAFETY = 100
DISTRESS_OBSTACLE_NOT_FOUND_SAFETY = 200
DISTRESS_NONE = 300

# safety not found search directions
SEARCH_DIRECTION_LEFT = 1000
SEARCH_DIRECTION_RIGHT = 2000


# agent safety data
SAFETY_RADIUS = 0.5 # metres
AGENT_RADIUS = 1.3 # metres
OBSTACLE_ALLOWANCE = 0.1 # metres

# sensing
OBSTACLE_PANIC_ACT_DISTANCE = 6 # metres # distance between agent and obstacle that will trigger distress

# translaion
TRANSLATION_INTERVAL = 1 # seconds