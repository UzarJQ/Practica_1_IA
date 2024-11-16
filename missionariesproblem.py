from state import MissionariesState
from missionariesworld import MissionariesWorld
from search import *


init_state = MissionariesState(87, 87, 0, 0, 'left',4)
goal_state = MissionariesState(0, 0, 87, 87, 'right',4)

# ------------------------------------------------------------
# Validate initial and final state
# Validar que
# 1. la cantidad de misioneros y canibales sea > 0
# 2. la capacidad del bote sea > 0
# 3. la cantidad de canibales sea > capacidad del bote
# 4. la cantidad de misioneros sea >= cantidad de canibales
# 5. La cantidad de misioneros y canibales al iniciar sea igual a la cantidad de la solucion
valid_problem = (
    init_state.miss[0] > 0 and
    init_state.cann[0] > 0 and
    init_state.capacity > 0 and
    init_state.cann[0] > init_state.capacity and
    init_state.miss[0] >= init_state.cann[0]
)

if not valid_problem:
    raise Exception("Invalid initial and final states")


#------------------------------------------------------------

# Breadth First Search algorithm
solution_bf, expanded, generated = breadth_first(init_state, goal_state)
if solution_bf != None:
    print ("breadth_first found a solution...")
else:
    print ("breadth_first failed...")
show_solution(solution_bf, expanded, generated)

# Depth First Search algorithm
solution_df, expanded, generated = depth_first(init_state, goal_state)
if solution_df != None:
    print ("depth_first found a solution...")
else:
    print ("depth_first failed...")
show_solution(solution_df, expanded, generated)

# Uniform Cost Search algorithm
solution_uc, expanded, generated = uniform_cost(init_state, goal_state)
if solution_uc != None:
    print ("uniform_cost found a solution...")
else:
    print ("uniform_cost failed...")
show_solution(solution_uc, expanded, generated)

#------------------------------------------------------------

# greedy Search algorithm
solution_greedy, expanded, generated = greedy(init_state, goal_state, h1)
if solution_greedy != None:
    print ("greedy found a solution...")
else:
    print ("greedy failed...")
show_solution(solution_greedy, expanded, generated)

# A* Search algorithm
solution_astar, expanded, generated = a_star(init_state, goal_state, h2)
if solution_greedy != None:
    print ("A* found a solution...")
else:
    print ("A* failed...")
show_solution(solution_astar, expanded, generated)

#------------------------------------------------------------

# Steps for the MissionariesWorld
solution = solution_bf
steps = []
while solution != None:
    if solution.action != None:
        steps.insert(0, solution.action)
    solution = solution.parent
    
# Possible solution to the original problem (3 missionaries, 3 cannibals, boat capacity 2)   
#steps = ['>02', '<01', '>02', '<01', '>20', '<11', '>20', '<01', '>02', '<01', '>02']
try:
    world = MissionariesWorld(init_state, goal_state, steps)
except Exception as ex:
    print ("Error in MissionariesWorld -->", ex.message)

