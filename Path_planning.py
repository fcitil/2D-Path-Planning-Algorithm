import numpy as np
import math
from shapely.geometry import LineString
import matplotlib.pyplot as plt

def emptyArray(n):
    array = np.empty((1, n))
    array = np.delete(array,0,0)
    return array

# Environmental Declarations
pathPoints = emptyArray(2)
steps = 0
epsilonVal = 0.1
Safety_Margin = 0.3
obstacles = emptyArray(6)
obstacle_count = 0

############################

def addObstacle(x, y, w, h):
    global obstacle_count
    global obstacles
    newrow = [x-w/2-Safety_Margin, y-h/2-Safety_Margin, w + Safety_Margin*2, h + Safety_Margin*2, x, y]
    #newrow = [x - w, y - h / 2 , w, h, x, y]
    obstacles = np.vstack([obstacles, newrow])
    obstacle_count = np.size(obstacles,0)


# Obstacle Definitions
addObstacle(3, 17, 2, 1)
addObstacle(16, 4, 4, 2)
addObstacle(7, 9, 3, 4)
addObstacle(13, 13, 2, 3)
addObstacle(16, 14, 1, 1)
addObstacle(14, 17, 3, 2)
######################

# Points
source_point = [1,5]
destination_point = [18,18]
########
pathPoints = np.vstack([pathPoints, source_point])

def calculateDistance(pt1x,pt1y,pt2x,pt2y):
    return math.sqrt(pow((pt1x-pt2x), 2) + pow((pt1y-pt2y), 2))

def checkCollision(source_point, destination_point):
    global obstacles
    global obstacle_count
    intersect_points = emptyArray(2)
    whereintsct_values = []
    obs_values = []

    doesTouch = False
    whereintsct = 0
    intersectpt = []

    for obs in range(np.size(obstacles,0)):

        pathLine = LineString([(source_point[0],source_point[1]), (destination_point[0],destination_point[1])])

        x0 = obstacles[obs,0]
        y0 = obstacles[obs,1]
        w = obstacles[obs,2]
        h = obstacles[obs,3]

        #creat
        leftLine = LineString([(x0, y0), (x0, y0+h)])
        bottomLine = LineString([(x0, y0), (x0+w, y0)])
        topLine = LineString([(x0, y0+h), (x0+w, y0+h)])
        rightLine = LineString([(x0+w, y0), (x0+w, y0+h)])

        # 1=left, 2=bottom, 3=top, 4=right for whereintsct (points on the
        # comments is the source point

        if (source_point[0] < destination_point[0]) and (source_point[1] < destination_point[1]): #bl
            if bottomLine.intersection(pathLine):
                print('Collision occured due obstacle {} from bottom!'.format(obs))
                doesTouch = True
                whereintsct = 2
                intersectpt = [bottomLine.intersection(pathLine).x, bottomLine.intersection(pathLine).y]
                intersect_points = np.vstack([intersect_points, intersectpt])
                whereintsct_values.append(whereintsct)
                obs_values.append(obs)
            elif leftLine.intersection(pathLine):
                print('Collision occured due obstacle {} from left!'.format(obs))
                doesTouch = True
                whereintsct = 1
                intersectpt = [leftLine.intersection(pathLine).x, leftLine.intersection(pathLine).y]
                intersect_points = np.vstack([intersect_points, intersectpt])
                whereintsct_values.append(whereintsct)
                obs_values.append(obs)     
            else:
                doesTouch = False

        elif (source_point[0] < destination_point[0]) and (source_point[1] > destination_point[1]): #tl
            if leftLine.intersection(pathLine):
                print('Collision occured due obstacle {} from left!'.format(obs))
                doesTouch = True
                whereintsct = 1
                intersectpt = [leftLine.intersection(pathLine).x, leftLine.intersection(pathLine).y]
                intersect_points = np.vstack([intersect_points, intersectpt])
                whereintsct_values.append(whereintsct)
                obs_values.append(obs)
            elif topLine.intersection(pathLine):
                print('Collision occured due obstacle {} from top!'.format(obs))
                doesTouch = True
                whereintsct = 3
                intersectpt = [topLine.intersection(pathLine).x, topLine.intersection(pathLine).y]
                intersect_points = np.vstack([intersect_points, intersectpt])
                whereintsct_values.append(whereintsct)
                obs_values.append(obs)     
            else:
                doesTouch = False

        elif (source_point[0] > destination_point[0]) and (source_point[1] > destination_point[1]): #tr
            if rightLine.intersection(pathLine):
                print('Collision occured due obstacle {} from right!'.format(obs))
                doesTouch = True
                whereintsct = 4
                intersectpt = [rightLine.intersection(pathLine).x, rightLine.intersection(pathLine).y]
                intersect_points = np.vstack([intersect_points, intersectpt])
                whereintsct_values.append(whereintsct)
                obs_values.append(obs)
            elif topLine.intersection(pathLine):
                print('Collision occured due obstacle {} from top!'.format(obs))
                doesTouch = True
                whereintsct = 3
                intersectpt = [topLine.intersection(pathLine).x, topLine.intersection(pathLine).y]
                intersect_points = np.vstack([intersect_points, intersectpt])
                whereintsct_values.append(whereintsct)
                obs_values.append(obs)     
            else:
                doesTouch = False

        elif (source_point[0] > destination_point[0]) and (source_point[1] < destination_point[1]): #tr
            if rightLine.intersection(pathLine):
                print('Collision occured due obstacle {} from right!'.format(obs))
                doesTouch = True
                whereintsct = 4
                intersectpt = [rightLine.intersection(pathLine).x, rightLine.intersection(pathLine).y]
                intersect_points = np.vstack([intersect_points, intersectpt])
                whereintsct_values.append(whereintsct)
                obs_values.append(obs)
            elif bottomLine.intersection(pathLine):
                print('Collision occured due obstacle {} from bottom!'.format(obs))
                doesTouch = True
                whereintsct = 2
                intersectpt = [bottomLine.intersection(pathLine).x, bottomLine.intersection(pathLine).y]
                intersect_points = np.vstack([intersect_points, intersectpt])
                whereintsct_values.append(whereintsct)
                obs_values.append(obs)     
            else:
                doesTouch = False

    if (np.size(intersect_points,0) > 0):
        min_dist = calculateDistance(source_point[0], source_point[1], intersect_points[0,0], intersect_points[0,1])
        min_index = 0
        doesTouch = True

        for pnt in range(np.size(intersect_points, 0)):

            curr_dist = calculateDistance(source_point[0], source_point[1], intersect_points[pnt,0], intersect_points[pnt,1])
            if(curr_dist < min_dist):
                min_dist = curr_dist
                min_index = pnt

        intersectpt = intersect_points[min_index]
        whereintsct = whereintsct_values[min_index]
        obs = obs_values[min_index]
    else:
        doesTouch = False
    
    return [doesTouch, obs, whereintsct, intersectpt]

def pathDrawer(fromWhere, toGo):
    global obstacle_count
    global pathPoints
    global steps
    steps += 1

    [isCollided, obs, whereintsct, intersectpt] = checkCollision(fromWhere, toGo)
    if isCollided == True:
        slope = (toGo[1]-fromWhere[1])/(toGo[0]-fromWhere[0])

        if(whereintsct == 1): # left case
            if slope > 0:
                beforePt = [(intersectpt[0]-epsilonVal, intersectpt[1]-epsilonVal)] # point a little before collision
                pathPoints = np.vstack([pathPoints, beforePt[0]])
                new_point = [(beforePt[0][0], obstacles[obs,1]+obstacles[obs, 3]+epsilonVal)]
            else:
                beforePt = [(intersectpt[0]-epsilonVal, intersectpt[1]+epsilonVal)] # point a little before collision
                pathPoints = np.vstack([pathPoints, beforePt[0]])
                new_point = [(beforePt[0][0], obstacles[obs,1]-epsilonVal)]

        elif(whereintsct == 2): # bottom case
            if slope > 0:
                beforePt = [(intersectpt[0]-epsilonVal, intersectpt[1]-epsilonVal)] # point a little before collision
                pathPoints = np.vstack([pathPoints, beforePt])
                new_point = [(obstacles[obs,0]+obstacles[obs,2]+epsilonVal, beforePt[0][1])]
            else:
                beforePt = [(intersectpt[0]+epsilonVal, intersectpt[1]+epsilonVal)] # point a little before collision
                pathPoints = np.vstack([pathPoints, beforePt[0]])
                new_point = [(obstacles[obs,0]-epsilonVal, beforePt[0][1])]

        elif(whereintsct == 3): # top case
            if slope > 0:
                beforePt = [(intersectpt[0]+epsilonVal, intersectpt[1]+epsilonVal)] # point a little before collision
                pathPoints = np.vstack([pathPoints, beforePt[0]])
                new_point = [(obstacles[obs,0]-epsilonVal, beforePt[0][1])]
            else:
                beforePt = [(intersectpt[0]-epsilonVal, intersectpt[1]+epsilonVal)] # point a little before collision
                pathPoints = np.vstack([pathPoints, beforePt[0]])
                new_point = [(obstacles[obs,0]+obstacles[obs,2]+epsilonVal, beforePt[0][1])]

        elif(whereintsct == 4): # right case
            if slope > 0:
                beforePt = [(intersectpt[0]+epsilonVal, intersectpt[1]+epsilonVal)] # point a little before collision
                pathPoints = np.vstack([pathPoints, beforePt[0]])
                new_point = [(beforePt[0][0], obstacles[obs,1]-epsilonVal)]
            else:
                beforePt = [(intersectpt[0]+epsilonVal, intersectpt[1]-epsilonVal)] # point a little before collision
                pathPoints = np.vstack([pathPoints, beforePt[0]])
                new_point = [(beforePt[0][0], obstacles[obs,1]+obstacles[obs, 3]+epsilonVal)]
        pathDrawer(list(beforePt[0]), list(new_point[0]))
        pathDrawer(list(new_point[0]), toGo)
    else:
        pathPoints = np.vstack([pathPoints, toGo])


def plotPath():
    # straight line from source point to destination point
    plt.plot([source_point[0], destination_point[0]],[source_point[1], destination_point[1]], 'g--', lw=0.5)
    # path line
    plt.plot(np.hsplit(pathPoints,2)[0],np.hsplit(pathPoints,2)[1], lw=2 )
    plt.show()

def rectangleDraw():
    for obs in obstacles:
        x0 = obs[0]
        y0 = obs[1]
        w = obs[2]
        h = obs[3]

        rectangle_safety = plt.Rectangle((x0,y0), w, h, fc='red',alpha=0.5)
        rectangle = plt.Rectangle((x0 + Safety_Margin, y0 + Safety_Margin),
                                         w - Safety_Margin*2, h - Safety_Margin*2, fc='red')
        plt.gca().add_patch(rectangle_safety)
        plt.gca().add_patch(rectangle)

plt.figure()
#plt.xlim([0, 20])
#plt.ylim([0, 20])

if __name__ == "__main__":
    pathDrawer(source_point, destination_point)
    rectangleDraw()
    plotPath()
