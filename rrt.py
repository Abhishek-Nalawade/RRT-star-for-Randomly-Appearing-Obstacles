import numpy as np
import cv2
import math
import random

#out = cv2.VideoWriter('Exploration.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 60, (400,300))
#out = cv2.VideoWriter('Output.mp4',cv2.VideoWriter_fourcc(*'mp4v'), 60, (400,300))



# def obstacles(st,canvas_size, canvas, duplicate_canvas):
#     if st[1] >= (90) and st[1] <= 110 and st[0] >= (40) and st[0] <= (60):
#         canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
#         duplicate_canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
#         #print("coordinate is in obstacle space")
#         return None
#     elif ((st[1] - (160))**2 + (st[0] - (50))**2) <= 225:
#         canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
#         duplicate_canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
#         #print("coordinate is in obstacle space")
#         return None
#     elif st[1] < 0 or st[1] >= canvas_size[1]:
#         canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
#         duplicate_canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
#         #print("coordinate is out of the map boundary")
#         return None
#     elif st[0] < 0 or st[0] >= canvas_size[0]:
#         canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
#         duplicate_canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
#         #print("coordinate is out of the map boundary")
#         return None
#     else :
#         return st


def obstacles(st,canvas_size, canvas, duplicate_canvas):
    if ((st[1] - 90)**2 + (st[0] - 70)**2) <= 1225:
        canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
        duplicate_canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
        #print("coordinate is in circle")
        return None

    # elif (st[0] + st[1] >= 391) and (st[1] - st[0] <= 265) and (st[0] + 0.49646*st[1] <= 305.20202) and (0.89003*st[1] - st[0] >= 148.7438):
    #     canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
    #     duplicate_canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
    #     #print("coordinate is in polygon")
    #     return None
    #
    # elif (st[0] + 0.49646*st[1] >= 305.20202) and (st[0] + 0.81259*st[1] <= 425.66019) and (st[0] + 0.17512 * st[1] <= 199.99422):
    #     canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
    #     duplicate_canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
    #     #print("coordinate is in polygon")
    #     return None
    #
    # elif (st[0] + 13.49145*st[1] <= 5256.7216) and (1.43169*st[1] - st[0] >= 368.82072) and (st[0] + 0.81259*st[1] >= 425.66019):
    #     canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
    #     duplicate_canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
    #     #print("coordinate is in polygon")
    #     return None

    elif (((st[1] - 246) / 60) ** 2) + (((st[0] - 145) / 30) ** 2) <= 1:
        canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
        duplicate_canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
        #print("coordinate is in ellipse")
        return None

    elif (st[1] >= 200 and st[1] <= 210 and st[0] <= 280 and st[0] >= 230):
        canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
        duplicate_canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
        #print("coordinate is in C shape")
        return None

    elif (st[1] >= 200 and st[1] <= 230 and st[0] <= 280 and st[0] >= 270):
        canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
        duplicate_canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
        #print("coordinate is in C shape")
        return None

    elif (st[1] >= 200 and st[1] <= 230 and st[0] <= 240 and st[0] >= 230):
        canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
        duplicate_canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
        #print("coordinate is in C shape")
        return None

    elif (st[0]) + (1.42814 * st[1]) >= 176.5511 and (st[0]) - (0.7 * st[1]) >= 74.39 and (st[0]) + (1.42814 * st[1]) <= 428.06815 and (st[0]) - (0.7 * st[1]) <= 98.80545:
        canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
        duplicate_canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
        #print("coordinate is in rectangle")
        return None

    elif st[1] < 0 or st[1] >= canvas_size[1]:
        canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
        duplicate_canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
        #print("coordinate is out of the map boundary")
        return None
    elif st[0] < 0 or st[0] >= canvas_size[0]:
        canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
        duplicate_canvas[(canvas_size[0])-st[0]][st[1]][0] = 255
        #print("coordinate is out of the map boundary")
        return None

    else :
        return st




def goal_threshold(m,n, canvas_size, goal1,random_point_radius, canvas, duplicate_canvas):
    if ((n - goal1[0][1])**2 + (m - goal1[0][0])**2) <= (random_point_radius/1.5)**2:
        #goal_canvas[(canvas_size[0])-m,n,0] = 1
        canvas[(canvas_size[0]-1)-m,n,1] = 255
        duplicate_canvas[(canvas_size[0]-1)-m,n,2] = 1

    return None


def get_random_point(canvas_size):
    child = list()
    child.append(random.randint(0, canvas_size[0]-1))
    child.append(random.randint(0, canvas_size[1]-1))
    #print(child)
    #print("random ",child)
    return child


def check_if_in_obstacle_space(child, canvas_size, duplicate_canvas):
    if duplicate_canvas[(canvas_size[0]-1) - child[0], child[1],0] == 255:
        return None
    return child

################
####currently there can be a situation that the for loop breaks when there is
####only one close node and it has an obstacle in the route
####which can lead to having no new node at all to connect in next functions
####find a solution for it
def get_all_nearby_points(child, canvas_size, duplicate_canvas):
    for i in range(0, canvas_size[1]+20, 20):
        y = (canvas_size[0]-1) - child[0]
        x = child[1]
        boxy_lower = y-i
        boxy_upper = y+i
        boxx_lower = x-i
        boxx_upper = x+i
        if boxy_lower < 0:
            boxy_lower = 0
        if boxy_upper > canvas_size[0]-1:
            boxy_upper = canvas_size[0]-1
        if boxx_lower < 0:
            boxx_lower = 0
        if boxx_upper > canvas_size[1]-1:
            boxx_upper = canvas_size[1]-1
        box = duplicate_canvas[boxy_lower:boxy_upper, boxx_lower:boxx_upper,:]
        close_points_coor = np.where(box[:,:,1]==1)

        #print(len(close_points_coor[0]))
        #print(boxy_lower,boxy_upper,boxx_lower,boxx_upper)
        #print("shape ",box.shape)
        if len(close_points_coor[0]) > 0:
            #print("hi")
            close_points_coor = np.array(close_points_coor)
            close_points_coor[0] = (canvas_size[0]-1)-(boxy_lower + close_points_coor[0])     #brings the coordinates to axes with origin at top left
            close_points_coor[1] = boxx_lower + close_points_coor[1]
            break

    #forming coordinates in the form [y,x]
    close_points = list()
    for i in range(len(close_points_coor[0])):
        close_points.append([int(close_points_coor[0][i]),int(close_points_coor[1][i])])
    #print("from here ",close_points)
    return child, close_points


def euclidean_distance_cost(close_node, current_node, parent_cost):
    #print(parent_cost)
    dist = ((close_node[0] - current_node[0])**2 + (close_node[1] - current_node[1])**2)**(1/2)
    final_cost = parent_cost + dist
    #print("hi ",final_cost)
    return final_cost


def find_the_closest_point(child, close_points,canvas_size, cost_and_theta_canvas):
    costs = list()
    #print("close ",close_points)
    for i in close_points:
        parent_cost = cost_and_theta_canvas[(canvas_size[0]-1) - i[0], i[1], 0]
        costs.append(euclidean_distance_cost(i, child, parent_cost))
    #print(costs)
    ind = costs.index(min(costs))
    closest_point = close_points[ind]
    #print("hello ",closest_point)
    #for visualization you can connect the point and then trim the distance
    return child, closest_point


###returns None when for all radius the point is in the obstacle or is visited already
def trim_distance_from_closest_node(child, closest_point, random_point_radius, canvas_size, duplicate_canvas):
    r = random_point_radius
    dist = ((child[0]-closest_point[0])**2 + (child[1]-closest_point[1])**2)**(1/2)
    dn = closest_point[1]-child[1]
    if dn != 0:
        angle = math.atan((closest_point[0]-child[0])/(dn))
    elif child[0] < closest_point[0] and child[1] == closest_point[1]:
        angle = -(math.pi)/2
    else:
        angle = (math.pi)/2

    if dist > random_point_radius:
        change_y = (r * math.sin(angle))
        change_x = (r * math.cos(angle))


        if child[0] < closest_point[0] and child[1] < closest_point[1]:     #third quadrant
            change_y = -(r * math.sin(angle))
            change_x = -(r * math.cos(angle))
            #print("negative y ",change_y)
        elif child[0] > closest_point[0] and child[1] < closest_point[1]:   #second quadrant
            change_y = -(r * math.sin(angle))
            change_x = -(r * math.cos(angle))

        new_y = change_y + closest_point[0]
        new_x = change_x + closest_point[1]
        child[0] = int(new_y)
        child[1] = int(new_x)
        #print("trim",child)
    #if canvas[(canvas_size[0]-1) - child[0], child[1],1] == 255:
    if child[0] > canvas_size[0]-1:
        child[0] = canvas_size[0]-1
    if child[1] > canvas_size[1]-1:
        child[1] = canvas_size[1]-1
    if child[0]< 0:
        child[0] = 0
    if child[1] < 0:
        child[1] = 0
    if duplicate_canvas[(canvas_size[0]-1) - child[0], child[1],0] == 255 or duplicate_canvas[(canvas_size[0]-1) - child[0], child[1],1] == 1:
        while r > 0:
            change_y = (r * math.sin(angle))
            change_x = (r * math.cos(angle))


            if child[0] < closest_point[0] and child[1] < closest_point[1]:     #third quadrant
                change_y = -(r * math.sin(angle))
                change_x = -(r * math.cos(angle))
                #print("negative y ",change_y)
            elif child[0] > closest_point[0] and child[1] < closest_point[1]:   #second quadrant
                change_y = -(r * math.sin(angle))
                change_x = -(r * math.cos(angle))

            new_y = change_y + closest_point[0]
            new_x = change_x + closest_point[1]
            child[0] = int(new_y)
            child[1] = int(new_x)
            if child[0] > canvas_size[0]-1:
                child[0] = canvas_size[0]-1
            if child[1] > canvas_size[1]-1:
                child[1] = canvas_size[1]-1
            if child[0]< 0:
                child[0] = 0
            if child[1] < 0:
                child[1] = 0
            if duplicate_canvas[(canvas_size[0]-1) - child[0], child[1],0] == 255 or duplicate_canvas[(canvas_size[0]-1) - child[0], child[1],1] == 1:
                if r == 1:
                    return None
                r = r - 1
                continue
            break         #if above condition is not satisfied then it breaks out and returns the child
    #print(child, closest_point)
    return child, closest_point


def check_if_obstacle_in_route(child, closest_point, canvas_size, duplicate_canvas):
    dist = ((child[0]-closest_point[0])**2 + (child[1]-closest_point[1])**2)**(1/2)
    #print("tp ",dist)
    dn = closest_point[1]-child[1]
    if dn != 0:
        angle = math.atan((closest_point[0]-child[0])/(dn))
    elif child[0] < closest_point[0] and child[1] == closest_point[1]:
        angle = -(math.pi)/2
    else:
        angle = (math.pi)/2

    while dist > 0:
        #print("tp ",dist)
        change_y = (dist * math.sin(angle))
        change_x = (dist * math.cos(angle))


        if child[0] < closest_point[0] and child[1] < closest_point[1]:     #third quadrant
            change_y = -(dist * math.sin(angle))
            change_x = -(dist * math.cos(angle))
            #print("negative y ",change_y)
        elif child[0] > closest_point[0] and child[1] < closest_point[1]:   #second quadrant
            change_y = -(dist * math.sin(angle))
            change_x = -(dist * math.cos(angle))
            #print("negative x ",change_x)

        new_y = change_y + closest_point[0]
        new_x = change_x + closest_point[1]
        new_y = int(new_y)
        new_x = int(new_x)
        dist = dist - 1
        if new_y > canvas_size[0]-1:
            new_y = canvas_size[0]-1
        if new_x > canvas_size[1]-1:
            new_x = canvas_size[1]-1
        if new_y < 0:
            new_y = 0
        if new_x < 0:
            new_x = 0
        #print("from while ",new_y, new_x)
        if duplicate_canvas[(canvas_size[0]-1) - new_y, new_x,0] == 255:
            return None

    return child, closest_point

def connect_child_and_closest_point(child, closest_point, canvas_size, canvas, duplicate_canvas, cost_and_theta_canvas, visited_child_list, visited_parent_list):
    cost = ((child[0]-closest_point[0])**2+(child[1]-closest_point[1])**2)**(1/2)
    cost_and_theta_canvas[(canvas_size[0]-1)-child[0], child[1], 0] = cost + cost_and_theta_canvas[(canvas_size[0]-1)- closest_point[0], closest_point[1],0]
    cv2.line(canvas,(closest_point[1],(canvas_size[0]-1)-closest_point[0]),(child[1],(canvas_size[0]-1)-child[0]),(0,0,255),1)
    duplicate_canvas[(canvas_size[0]-1)-closest_point[0], closest_point[1], 1] = 1
    duplicate_canvas[(canvas_size[0]-1)-child[0], child[1], 1] = 1
    visited_child_list.append(child)
    visited_parent_list.append(closest_point)
    cv2.imshow("final",canvas)
    cv2.waitKey(1)
    #print("its conversion: ",child)
    return child




def rewire(child, canvas_size, canvas, duplicate_canvas, cost_and_theta_canvas, visited_child_list, visited_parent_list, random_point_radius):
    costs = list()
    y = (canvas_size[0]-1) - child[0]
    x = child[1]
    box_dimension_by_2 = random_point_radius + 1
    boxy_lower = y - box_dimension_by_2
    boxy_upper = y + box_dimension_by_2
    boxx_lower = x - box_dimension_by_2
    boxx_upper = x + box_dimension_by_2
    if boxy_lower < 0:
        boxy_lower = 0
    if boxy_upper > canvas_size[0]-1:
        boxy_upper = canvas_size[0]-1
    if boxx_lower < 0:
        boxx_lower = 0
    if boxx_upper > canvas_size[1]-1:
        boxx_upper = canvas_size[1]-1
    box = duplicate_canvas[boxy_lower:boxy_upper, boxx_lower:boxx_upper,:]
    possible_points_for_rewiring = np.where(box[:,:,1]==1)
    possible_points_for_rewiring = np.array(possible_points_for_rewiring)
    possible_points_for_rewiring[0] = (canvas_size[0]-1)-(boxy_lower + possible_points_for_rewiring[0])     #brings the coordinates to axes with origin at top left
    possible_points_for_rewiring[1] = boxx_lower + possible_points_for_rewiring[1]
    for i in range(len(possible_points_for_rewiring[0])):
        old_cost = cost_and_theta_canvas[(canvas_size[0]-1) - possible_points_for_rewiring[0][i], possible_points_for_rewiring[1][i], 0]
        child_cost = cost_and_theta_canvas[(canvas_size[0]-1)-child[0], child[1], 0]
        old_point = [possible_points_for_rewiring[0][i],possible_points_for_rewiring[1][i]]
        new_cost = euclidean_distance_cost(old_point, child, child_cost)
        if new_cost < old_cost:
            valid_route = check_if_obstacle_in_route(child, old_point, canvas_size, duplicate_canvas)
            if valid_route == None:
                continue
            ind = visited_child_list.index(old_point)
            old_parent = visited_parent_list.pop(ind)
            cost_and_theta_canvas[(canvas_size[0]-1)-old_point[0],old_point[1],0] = new_cost
            #print(old_parent, old_point)
            cv2.line(canvas,(old_parent[1],(canvas_size[0]-1)-old_parent[0]),(old_point[1],(canvas_size[0]-1)-old_point[0]),(0,0,0),1)
            #canvas[(canvas_size[0]-1)-old_parent[0],old_parent[1],1] = 1
            #canvas[(canvas_size[0]-1)-old_point[0],old_point[1],1] = 1
            visited_parent_list.insert(ind, child)
            cv2.line(canvas,(child[1],(canvas_size[0]-1)-child[0]),(old_point[1],(canvas_size[0]-1)-old_point[0]),(0,0,255),1)
            #canvas[(canvas_size[0]-1)-old_point[0],old_point[1],1] = 1
            #canvas[(canvas_size[0]-1)-child[0],child[1],1] = 1
            #print("rewired")
    return child


def compare_with_goal(child,minimum_goal_cost, canvas_size, duplicate_canvas, cost_and_theta_canvas):
    if duplicate_canvas[(canvas_size[0]-1)-child[0], child[1], 2] == 1:
        new_goal_cost = cost_and_theta_canvas[(canvas_size[0]-1)-child[0], child[1], 0]
        print("\n Goal has been reached \n")
        if minimum_goal_cost == None or new_goal_cost < minimum_goal_cost:
            minimum_goal_cost = new_goal_cost
            print("\n New path has been found \n")
            return child, minimum_goal_cost
        return None, None
    return None, None

def backtracking(child, canvas_size, canvas, visited_child_list, visited_parent_list):
    path = list()
    ind = visited_child_list.index(child)
    parent = visited_parent_list[ind]
    cv2.line(canvas,(child[1],(canvas_size[0]-1)-child[0]),(parent[1],(canvas_size[0]-1)-parent[0]),(0,255,0),1)
    #canvas[(canvas_size[0]-1)-parent[0],parent[1],1] = 1
    #canvas[(canvas_size[0]-1)-child[0],child[1],1] = 1
    path.append(child)
    #path.append(parent)
    while parent != None:
        child_ind = visited_child_list.index(parent)
        child1 = visited_child_list[child_ind]
        parent = visited_parent_list[child_ind]
        path.append(child1)
        if parent != None:
            cv2.line(canvas,(child1[1],(canvas_size[0]-1)-child1[0]),(parent[1],(canvas_size[0]-1)-parent[0]),(0,255,0),1)
            #canvas[(canvas_size[0]-1)-parent[0],parent[1],1] = 1
            #canvas[(canvas_size[0]-1)-child1[0],child1[1],1] = 1
    return path


def moving_robot(path, canvas_size, canvas, duplicate_canvas, cost_and_theta_canvas, random_point_radius,second_time):
    #print(path)
    #path.append(start[0])
    path.reverse()
    #print(path)
    canvas_copy = canvas.copy()
    velocity = 5
    n=1

    end_point = path[-1]
    pt = end_point
    count = len(path)
    i = 0
    while i < count-1:
        #if i == count - 1:
        #    break
        displ_to_cover = ((path[i][0]-path[i+1][0])**2+(path[i][1]-path[i+1][1])**2)**(1/2)
        dn = path[i+1][1]-path[i][1]
        if dn != 0:
            theta = math.atan((path[i+1][0]-path[i][0])/(dn))
        elif path[i+1][0] < path[i][0] and path[i+1][1] == path[i][1]:      #  condition for negative Y axis
            theta = -(math.pi)/2
        else:
            theta = math.pi/2


        # when second point is reached on the path a random point is selected from
        # the rest of the point for random obstacle
        if i == 2:
            remaining_path = path[i+1:-2]
            pt = random.choice(remaining_path)
            radius = 8
            cv2.circle(canvas, (pt[1],(canvas_size[0]-1)-pt[0]), radius, (255,0,0),-1)
            cv2.circle(duplicate_canvas, (pt[1],(canvas_size[0]-1)-pt[0]), radius, (255,0,0),-1)
            cv2.circle(canvas_copy, (pt[1],(canvas_size[0]-1)-pt[0]), radius, (255,0,0),-1)
        time = displ_to_cover/velocity
        t = 0
        final = 0
        while t < time:
            dt = 0.01
            displ_covered = velocity * dt
            final = final + displ_covered
            change_y = final * math.sin(theta)
            change_x = final * math.cos(theta)
            if path[i+1][0] < path[i][0] and path[i+1][1] < path[i][1]:     # third quadrant
                change_y = -(final * math.sin(theta))
                change_x = -(final * math.cos(theta))
                #print("negative y ",change_y)
            elif path[i+1][0] > path[i][0] and path[i+1][1] < path[i][1]:   # second quadrant
                change_y = -(final * math.sin(theta))
                change_x = -(final * math.cos(theta))
                #print("negative x ",change_x)
            elif path[i+1][0] == path[i][0] and path[i+1][1] < path[i][1]:      # negative X axis
                change_y = (final * math.sin(theta))    # this value is 0
                change_x = -(final * math.cos(theta))

            new_y = change_y + path[i][0]
            new_x = change_x + path[i][1]
            t = t + dt
            new_y = int(new_y)
            new_x = int(new_x)
            #print(new_y,new_x)
            canvas_diff = canvas - canvas_copy
            canvas_diff[canvas_diff[:,:,1]<0] = -255
            canvas_diff[canvas_diff[:,:,2]<0] = -255
            canvas = canvas - canvas_diff
            cv2.imshow("canvas_before",canvas)

            cv2.circle(canvas, (new_x,(canvas_size[0]-1)-new_y), 1, (128,128,128), 5)
            previous_canvas = canvas.copy()
            cv2.imshow("final",canvas)
            cv2.imshow("diff",canvas_diff)
            cv2.waitKey(1)
            current = [new_y,new_x]


            if n==1:
                chck = check_if_obstacle_in_route(current, pt, canvas_size, duplicate_canvas)
                if chck == None and i >= 2:
                    dist = ((current[0]-pt[0])**2+(current[1]-pt[1])**2)**(1/2) - radius
                    #print("dist ",dist)
                    random_obstacle_threshold = 20
                    if dist < random_obstacle_threshold:
                        print("called")
                        print("before ",path)
                        print("current point ",current)
                        path = remapping(current, pt, radius, path, canvas_size, canvas, duplicate_canvas, cost_and_theta_canvas, random_point_radius,second_time,path[i+1])
                        print("after ",path)
                        count = len(path)
                        n=0
                        break
        i += 1
    return


def remapping(current, obstacle_center, obstacle_radius, path, canvas_size, canvas, duplicate_canvas, cost_and_theta_canvas, random_point_radius,second_time,node_from_which_to_delete_path):
    new_visited_child_list = list()
    new_visited_parent_list = list()
    #cost_and_theta_canvas[(canvas_size[0]-1)-current_location[0], current_location[1],0] = 0
    #rewire(current_location, canvas_size, canvas, duplicate_canvas, cost_and_theta_canvas, new_visited_child_list, new_visited_parent_list, random_point_radius)
    print("path before RRT ",path)
    del_ind = path.index(node_from_which_to_delete_path)
    print("del from index ",del_ind)
    current_location = current.copy()
    y = current_location[0]
    x = current_location[1]
    box_dimension_by_2 = (10*random_point_radius)
    boxy_lower = y - box_dimension_by_2
    boxy_upper = y + box_dimension_by_2
    boxx_lower = x - box_dimension_by_2
    boxx_upper = x + box_dimension_by_2

    if boxy_lower < 0:
        boxy_lower = 0
    if boxy_upper > canvas_size[0]-1:
        boxy_upper = canvas_size[0]-1
    if boxx_lower < 0:
        boxx_lower = 0
    if boxx_upper > canvas_size[1]-1:
        boxx_upper = canvas_size[1]-1

    boxy_lowerx = canvas_size[0] - 1 - boxy_lower
    boxy_upperx = canvas_size[0] - 1 - boxy_upper
    boxx_lowerx = boxx_lower
    boxx_upperx = boxx_upper

    box = duplicate_canvas[boxy_upperx:boxy_lowerx, boxx_lowerx:boxx_upperx,:]
    box[box[:,:,1]==1] =2
    box[box[:,:,2]==1] =2
    main_box = canvas[boxy_upperx:boxy_lowerx, boxx_lowerx:boxx_upperx,:]
    size = box.shape
    ind = path.index(obstacle_center)
    temp_goal = None
    print("path just before calling ",path)
    for i in range(len(path[ind:])):
        if duplicate_canvas[(canvas_size[0]-1)-path[ind+i][0], path[ind+i][1], 0] == 255:
            print("continued")
            continue
        else:
            ax = path[ind:]
            temp_goal = ax[i+1]    #.copy()
            break
    final_ind = path.index(temp_goal)
    print("goal index ", final_ind)
    #print(current_location)
    #######reason for out oof bound error
    current_location[0] = current_location[0] - boxy_lower
    current_location[1] = current_location[1] - boxx_lower
    temp_goal[0] = temp_goal[0] - boxy_lower
    temp_goal[1] = temp_goal[1] - boxx_lower
    #print(current_location)
    second_time = 1
    #cv2.imshow("box",main_box)
    #cv2.waitKey(0)
    print("just before calling ",path)
    new_route = RRT_star([current_location], [temp_goal], size, main_box, box,second_time)
    new_route = new_route[0]
    print("new route ",new_route)
    print("path after RRT ",path)
    for i in range(len(new_route)):
        new_route[i][0] = boxy_lower + new_route[i][0]
        new_route[i][1] = boxx_lower + new_route[i][1]
    print("remapped route ", new_route)
    while True:
        tmp = path[del_ind]
        if tmp == temp_goal:
            path.pop(del_ind)
            break
        path.pop(del_ind)
    print("deleted ",path)

    for i in new_route:
        path.insert(del_ind, i)
    print("before returning ",path)

    #moving_robot(new_route, main_canvas_size, main_canvas, main_duplicate_canvas, main_cost_and_theta_canvas, R1)
    return path



main_canvas_size = [300, 400, 3]

main_canvas = np.zeros((main_canvas_size[0], main_canvas_size[1], main_canvas_size[2]))
main_duplicate_canvas = np.zeros((main_canvas_size[0], main_canvas_size[1], main_canvas_size[2]))
main_visited_child_list = list()
main_visited_parent_list = list()

n = 1
while n > 0:
    start = list()
    goal = list()
    x1 = input("Enter the x co-ordinate of the start point: ")
    y1 = input("Enter the y co-ordinate of the start point: ")
    x2 = input("Enter the x co-ordinate of the goal point: ")
    y2 = input("Enter the y co-ordinate of the goal point: ")
    start.append([int(y1), int(x1)])
    goal.append([int(y2), int(x2)])
    lis = [start, goal]
    valid = list()
    for i in lis:
        valid.append(obstacles(i[0], main_canvas_size, main_canvas, main_duplicate_canvas))
    if valid[0] == None or valid[1] == None:
        print("Error: One of the entered point is either in obstacle space or out of map boundary")
        continue
    else:
        n = 0


'''#marking the start point on the canvas
duplicate_canvas[(canvas_size[0]-1)-start[0][0], start[0][1], 1] = 1
#print(np.where(canvas[:,:,1]==1))
#marking the obstacles
for i in range(canvas_size[0]):
    for j in range(canvas_size[1]):
        obstacles([i,j])
        goal_threshold(i,j)'''

#cv2.imshow("canvas",canvas)
#cv2.waitKey(0)
#cv2.destroyAllWindows()


'''#adding the start node to the visited lists
visited_child_list.append(start[0])
visited_parent_list.append(None)'''


#main loop
def RRT_star(start1, goal1, canvas_size, main_canvas1, main_duplicate_canvas1, second_time):
    random_point_radius = 10
    #action_set = [[rpm1,0],[0,rpm1],[rpm1,rpm1],[rpm2,0],[0,rpm2],[rpm2,rpm2],[rpm1,rpm2],[rpm2,rpm1]]
    canvas = np.zeros((canvas_size[0], canvas_size[1], canvas_size[2]))
    duplicate_canvas = np.zeros((canvas_size[0], canvas_size[1], canvas_size[2]))
    cost_and_theta_canvas = np.zeros((canvas_size[0], canvas_size[1], 2))
    goal_canvas = np.zeros((canvas_size[0], canvas_size[1], 1))
    visited_child_list = list()
    visited_parent_list = list()
    minimum_goal_cost = None



    #marking the start point on the canvas
    duplicate_canvas[(canvas_size[0]-1)-start1[0][0], start1[0][1], 1] = 1
    #print(np.where(canvas[:,:,1]==1))
    # marking the obstacles
    if second_time == 0:
        for i in range(canvas_size[0]):
            for j in range(canvas_size[1]):
                obstacles([i,j],canvas_size,canvas, duplicate_canvas)
                goal_threshold(i,j,canvas_size,goal1,random_point_radius,canvas, duplicate_canvas)

    # marks the goal threshold for second time
    if second_time == 1:
        random_point_radius = 8
        canvas = main_canvas1
        duplicate_canvas = main_duplicate_canvas1
        duplicate_canvas[(canvas_size[0]-1)-start1[0][0], start1[0][1], 1] = 1
        for i in range(canvas_size[0]):
            for j in range(canvas_size[1]):
                #obstacles([i,j],canvas_size,canvas, duplicate_canvas)
                goal_threshold(i,j,canvas_size,goal1,random_point_radius,canvas, duplicate_canvas)
                c=1
        #cv2.imshow("tp", canvas)
        #cv2.waitKey(0)
    #adding the start node to the visited lists
    visited_child_list.append(start1[0])
    visited_parent_list.append(None)

    check = 0       #used for backtracking
    while True:
        pnt = get_random_point(canvas_size)
        new_pnt = check_if_in_obstacle_space(pnt, canvas_size, duplicate_canvas)
        if new_pnt == None:
            #print("from here")
            continue
        new_pnt, neighbourhood = get_all_nearby_points(new_pnt,canvas_size,duplicate_canvas)
        new_pnt, parent = find_the_closest_point(new_pnt, neighbourhood,canvas_size, cost_and_theta_canvas)
        child_and_parent = trim_distance_from_closest_node(new_pnt,parent, random_point_radius, canvas_size, duplicate_canvas)
        if child_and_parent == None:
            #print("no")
            continue
        valid_child_and_parent = check_if_obstacle_in_route(child_and_parent[0],child_and_parent[1], canvas_size, duplicate_canvas)
        if valid_child_and_parent == None:
            #print("yes")
            continue
        final_child = connect_child_and_closest_point(valid_child_and_parent[0], valid_child_and_parent[1], canvas_size, canvas, duplicate_canvas, cost_and_theta_canvas, visited_child_list, visited_parent_list)
        final_child = rewire(final_child, canvas_size, canvas, duplicate_canvas, cost_and_theta_canvas, visited_child_list, visited_parent_list, random_point_radius)
        final_child, minimum_goal_cost = compare_with_goal(final_child, minimum_goal_cost, canvas_size, duplicate_canvas, cost_and_theta_canvas)
        if final_child != None:
            for_backtracking_child = final_child
        if final_child != None or check == 1:
            check = 1
            route = backtracking(for_backtracking_child, canvas_size, canvas, visited_child_list, visited_parent_list)
        key = cv2.waitKey(1)
        if key == 27:
            break
    return route, canvas, duplicate_canvas, cost_and_theta_canvas, canvas_size, random_point_radius


def rotating_random_point(curren, final_orien):
    omega = 5
    time1 = (abs(curren - final_orien))/omega
    t1 = 0
    dt = 0.01
    new_final_theta = 0
    while t1 < time1:
        change_theta = omega * dt
        new_final_theta = new_final_theta + change_theta
        t1 = t1 + dt
    new_final_theta = new_final_theta + curren
    return new_final_theta

def translating_random_point(coor,orien):
    current_orient = 0
    for i in range(len(coor)):
        if current_orient != orient[0][0]:
            new_final_theta = rotating_random_point(current_orient,orient_to)
        t = 0
        final = 0
        while t < time:
            dt = 0.01
            displ_covered = velocity * dt
            final = final + displ_covered
            change_y = final * math.sin(theta)
            change_x = final * math.cos(theta)
            if coor[i+1][0] < path[i][0] and path[i+1][1] < path[i][1]:     #third quadrant
                change_y = -(final * math.sin(theta))
                change_x = -(final * math.cos(theta))
                theta_temp = (math.pi)+theta


                #print("negative y ",change_y)
            elif path[i+1][0] > path[i][0] and path[i+1][1] < path[i][1]:   #second quadrant
                change_y = -(final * math.sin(theta))
                change_x = -(final * math.cos(theta))
                theta_temp = (math.pi)-abs(theta)




##################
# excutes the main code
second_time = 0
optimal_route, main_canvas, main_duplicate_canvas, main_cost_and_theta_canvas, main_canvas_size, R1 = RRT_star(start, goal, main_canvas_size, main_canvas, main_duplicate_canvas,second_time)
moving_robot(optimal_route, main_canvas_size, main_canvas, main_duplicate_canvas, main_cost_and_theta_canvas, R1,second_time)
'''for i in optimal_route:
    print(i)
    tp = main_cost_and_theta_canvas
    tp1 = main_canvas_size
    print("its cost: ",tp[(tp1[0]-1)-i[0],i[1],0])'''
