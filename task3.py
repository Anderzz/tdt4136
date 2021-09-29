from source2 import *

map1=Map_Obj(task=3)
kart=map1.get_maps()[1]
costs=map1.get_maps()[0]
start=map1.get_start_pos()
startnode=(start[0],start[1])
goal=map1.get_goal_pos()
goalnode=(goal[0],goal[1])
print(startnode,goalnode)
res=astar(kart, costs, startnode, goalnode)
print(res)
visual=kart
for koord in res:
    visual[koord[0]][koord[1]] = '-> '
      
map1.print_map(visual)
map1.show_map(visual)