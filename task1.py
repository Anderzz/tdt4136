from source1 import *

map1=Map_Obj(task=1)
kart=map1.get_maps()[1]
start=map1.get_start_pos()
startnode=(start[0],start[1])
goal=map1.get_goal_pos()
goalnode=(goal[0],goal[1])
print(startnode,goalnode)
res=astar(kart,startnode,goalnode)
print(res)
visual=kart
for koord in res:
    visual[koord[0]][koord[1]] = '-> '
      
map1.print_map(visual)
map1.show_map(visual)