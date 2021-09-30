from source1 import *

#get the map from handout code
map1=Map_Obj(task=1)
kart=map1.get_maps()[1]
#get the start and goal nodes and unzip their components
start=map1.get_start_pos()
startnode=(start[0],start[1])
goal=map1.get_goal_pos()
goalnode=(goal[0],goal[1])
print(startnode,goalnode)
#run the algorithm
res=astar(kart,startnode,goalnode)
#and print the result
print(res)
visual=kart
#visualize the path
for koord in res:
    visual[koord[0]][koord[1]] = '-> '
#show the solution
map1.print_map(visual)
map1.show_map(visual)