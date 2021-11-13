import math
import queue
import time

mapOfGridCoordsAndActions = {}
minCostToReachGridPoint = {}
minCostParent = {}

algoChoice = "BFS" #Default algo to use
worldDimensions = [0,0,0]
startGrid = ""
goalGrid = ""
numberOfActionableGrids = 0

def processInputFile():
    inputFile = open("input.txt", "r")

    #first line
    global algoChoice
    algoChoice = inputFile.readline().strip()
    # second line
    global worldDimensions
    worldDimensions = inputFile.readline().strip().split(" ")
    # third line
    global startGrid
    startGrid = inputFile.readline().strip()
    minCostToReachGridPoint[startGrid] = 0
    minCostParent[startGrid] = startGrid
    # fourth line
    global goalGrid
    goalGrid = inputFile.readline().strip()
    # fifth line
    global numberOfActionableGrids
    numberOfActionableGrids = int(inputFile.readline().strip())

    for line in inputFile:
        gridInfoArr = line.split(" ")
        gridCoords = gridInfoArr[0] + " " + gridInfoArr[1] + " " + gridInfoArr[2]
        if len(gridInfoArr) > 3:
            listOfActions = gridInfoArr[3:len(gridInfoArr)]
            cleanedListOfActions = []
            for action in listOfActions:
                cleanedListOfActions.append(int(action.strip()))
        mapOfGridCoordsAndActions[gridCoords] = cleanedListOfActions

    inputFile.close()

def actOnPoint(x,y,z, action):
    action = int(action)

    xCoord = int(x)
    yCoord = int(y)
    zCoord = int(z)

    #this could be enumerated more easily, but for the sake of readability:
    if action == 1:
        xCoord = xCoord + 1
    elif action == 2:
        xCoord = xCoord - 1
    elif action == 3:
        yCoord = yCoord + 1
    elif action == 4:
        yCoord = yCoord - 1
    elif action == 5:
        zCoord = zCoord + 1
    elif action == 6:
        zCoord = zCoord - 1
    elif action == 7:
        xCoord = xCoord + 1
        yCoord = yCoord + 1
    elif action == 8:
        xCoord = xCoord + 1
        yCoord = yCoord - 1
    elif action == 9:
        xCoord = xCoord - 1
        yCoord = yCoord + 1
    elif action == 10:
        xCoord = xCoord - 1
        yCoord = yCoord - 1
    elif action == 11:
        xCoord = xCoord + 1
        zCoord = zCoord + 1
    elif action == 12:
        xCoord = xCoord + 1
        zCoord = zCoord - 1
    elif action == 13:
        xCoord = xCoord - 1
        zCoord = zCoord + 1
    elif action == 14:
        xCoord = xCoord - 1
        zCoord = zCoord - 1
    elif action == 15:
        yCoord = yCoord + 1
        zCoord = zCoord + 1
    elif action == 16:
        yCoord = yCoord + 1
        zCoord = zCoord - 1
    elif action == 17:
        yCoord = yCoord - 1
        zCoord = zCoord + 1
    elif action == 18:
        yCoord = yCoord - 1
        zCoord = zCoord - 1

    #TODO: Check world boundaries here and fail if needed

    return xCoord,yCoord,zCoord

def moveToNextReachableGridPointGivenDirection(coordinateStr, action):
    xyzCoordinates = coordinateStr.split(" ")

    minCostToReachOrigin = minCostToReachGridPoint[coordinateStr]

    xCurrentCoord = int(xyzCoordinates[0])
    yCurrentCoord = int(xyzCoordinates[1])
    zCurrentCoord = int(xyzCoordinates[2])
    isValidNextPoint = True

    #Do first movement
    xCurrentCoord,yCurrentCoord, zCurrentCoord = actOnPoint(xCurrentCoord,yCurrentCoord,zCurrentCoord,action)
    currentGridPointToCheck = str(xCurrentCoord) + " " + str(yCurrentCoord) + " " + str(zCurrentCoord)

    if xCurrentCoord >= int(worldDimensions[0]) or xCurrentCoord < 0 or yCurrentCoord >= int(
            worldDimensions[1]) or yCurrentCoord < 0 or zCurrentCoord >= int(worldDimensions[2]) or zCurrentCoord < 0:
        # Greater than or equal to since we index at 0, and the homework specifically says if given boundary is the limit
        # additionally, the grid always starts at 0,0,0 so any negative values are always
        #print("Point is outside world boundaries: " + currentGridPointToCheck + "\t world bounds: " + str(
        #    worldDimensions))
        isValidNextPoint = False
        return coordinateStr, minCostToReachOrigin, isValidNextPoint

    while currentGridPointToCheck not in mapOfGridCoordsAndActions:
        xCurrentCoord, yCurrentCoord, zCurrentCoord = actOnPoint(xCurrentCoord,yCurrentCoord, zCurrentCoord, action)
        currentGridPointToCheck = str(xCurrentCoord) + " " + str(yCurrentCoord) + " " + str(zCurrentCoord)

        if xCurrentCoord >= int(worldDimensions[0]) or xCurrentCoord < 0 or yCurrentCoord >= int(worldDimensions[1]) or yCurrentCoord < 0 or zCurrentCoord >= int(worldDimensions[2]) or zCurrentCoord < 0:
            #Greater than or equal to since we index at 0, and the homework specifically says if given boundary is the limit
            #additionally, the grid always starts at 0,0,0 so any negative values are always
            #print("Point is outside world boundaries: " + currentGridPointToCheck + "\t world bounds: " + str(worldDimensions))
            isValidNextPoint = False
            return coordinateStr, minCostToReachOrigin, isValidNextPoint

    minCostToReachNewCurrent = minCostToReachOrigin + computeMoveCostBetweenAnyTwoPoints(currentGridPointToCheck, coordinateStr)


    return currentGridPointToCheck, minCostToReachNewCurrent, isValidNextPoint

def threeDBFSToAll():
    queueOfNodesToCheck = queue.Queue()
    aStarTerminalQueuePhase = False

    if algoChoice == "BFS":
        queueOfNodesToCheck.put(startGrid)
    elif algoChoice == "UCS" or algoChoice == "A*":
        queueOfNodesToCheck = queue.PriorityQueue()
        queueOfNodesToCheck.put((0, startGrid))

    while not queueOfNodesToCheck.empty():
        print(list(queueOfNodesToCheck.queue))
        currentNodeCoordsStr = queueOfNodesToCheck.get()

        if algoChoice == "UCS" or algoChoice == "A*":
            currentNodeCoordsStr = currentNodeCoordsStr[1]
        listOfActions = mapOfGridCoordsAndActions[currentNodeCoordsStr]
        minCostToGetToCurrentNode = minCostToReachGridPoint[currentNodeCoordsStr]
        print("for node: " + currentNodeCoordsStr)

        for action in listOfActions:
            someNextGridPoint, costToReachSNGP, isValidNextPoint = moveToNextReachableGridPointGivenDirection(currentNodeCoordsStr, action)

            #if point is outside boundaries, we dont ever want to explore it or it's children
            if isValidNextPoint == True:
                #TODO: Ensure we cover cycle detection here
                if someNextGridPoint == goalGrid:
                    #If we are using A*, then the path we found is the optimal path given our heuristic is admissible and consistent
                    if algoChoice == "A*":
                        minCostToReachGridPoint[someNextGridPoint] = costToReachSNGP
                        minCostParent[someNextGridPoint] = currentNodeCoordsStr

                        #rather than end immeditale, we need to check the remaining queue, in case we're competing with
                        #another path that is just larger and may become smaller with diagonals vs straights
                        aStarTerminalQueuePhase = True
                    else:
                        if someNextGridPoint not in minCostToReachGridPoint:
                            # if our node hasnt been found yet we need to add it to our minimum path and cost lists
                            minCostToReachGridPoint[someNextGridPoint] = costToReachSNGP
                            minCostParent[someNextGridPoint] = currentNodeCoordsStr
                            # then add it to the queue so we can explore its children
                            if algoChoice == "BFS":
                                queueOfNodesToCheck.put(someNextGridPoint)
                            elif algoChoice == "UCS":
                                queueOfNodesToCheck.put((costToReachSNGP, someNextGridPoint))
                            elif algoChoice == "A*":
                                if aStarTerminalQueuePhase == False:
                                    queueOfNodesToCheck.put(
                                        (aStarHeuristic(someNextGridPoint, costToReachSNGP), someNextGridPoint))
                        elif costToReachSNGP < minCostToReachGridPoint[someNextGridPoint]:
                            # if our newfound path to an already found node is cheaper, we need to update our min path to that node
                            minCostToReachGridPoint[someNextGridPoint] = costToReachSNGP
                            minCostParent[someNextGridPoint] = currentNodeCoordsStr

                else:
                    if someNextGridPoint not in minCostToReachGridPoint:
                        #if our node hasnt been found yet we need to add it to our minimum path and cost lists
                        minCostToReachGridPoint[someNextGridPoint] = costToReachSNGP
                        minCostParent[someNextGridPoint] = currentNodeCoordsStr
                        #then add it to the queue so we can explore its children
                        if algoChoice == "BFS":
                            queueOfNodesToCheck.put(someNextGridPoint)
                        elif algoChoice == "UCS":
                            queueOfNodesToCheck.put((costToReachSNGP, someNextGridPoint))
                        elif algoChoice == "A*":
                            if aStarTerminalQueuePhase == False:
                                queueOfNodesToCheck.put((aStarHeuristic(someNextGridPoint, costToReachSNGP), someNextGridPoint))

                    elif costToReachSNGP < minCostToReachGridPoint[someNextGridPoint]:
                        #if our newfound path to an already found node is cheaper, we need to update our min path to that node
                        minCostToReachGridPoint[someNextGridPoint] = costToReachSNGP
                        minCostParent[someNextGridPoint] = currentNodeCoordsStr

def aStarHeuristic(pointToCheck, costToReachPoint):
    gn = costToReachPoint
    pointCoords = pointToCheck.split(" ")
    goalCoords = goalGrid.split(" ")

    #Heuristic 1: Straight line distance
    hn = math.sqrt(
        ((int(pointCoords[0]) - int(goalCoords[0])) ** 2) +
        ((int(pointCoords[1]) - int(goalCoords[1])) ** 2) +
        ((int(pointCoords[2]) - int(goalCoords[2])) ** 2)
    )

    fn = gn + hn
    print("coords: " + str(pointCoords) +"\tcomputed fn: " + str(fn))
    return fn


def computeMoveCostBetweenAnyTwoPoints(nodeA, nodeB):
    current = nodeA.split(" ")
    next = nodeB.split(" ")

    moveCost = 0
    xdiff = abs(int(current[0]) - int(next[0]))
    ydiff = abs(int(current[1]) - int(next[1]))
    zdiff = abs(int(current[2]) - int(next[2]))

    if algoChoice == "BFS":
        # we still count diagaonal moves as 1 move, rather than 1 move up and 1 move
        # accross (2 total), or the diagonal distance (>1)
        #moveCost = max(xdiff, ydiff, zdiff)
        moveCost = 1

    elif algoChoice == "UCS" or algoChoice == "A*":
        #We were told on the HW and from the Piazza answers that ALWAYS diagonal movements cost 14, 'straight'
        #movements cost 10
        sum = xdiff + ydiff + zdiff

        #since _diff values are abs val differences, if the total change is equal to only the change in 1,
        #then we know we have a straight movement along a single axis (not diagonal) - if two axis values were to change then we would have a diagonal
        if sum == xdiff or sum == ydiff or sum == zdiff:
            moveCost = 10
        else:
            moveCost = 14

        #moveCost = math.sqrt(
        #    ((int(current[0]) - int(previous[0])) ** 2) +
        #    ((int(current[1]) - int(previous[1])) ** 2) +
        #    ((int(current[2]) - int(previous[2])) ** 2)
        #)

    return moveCost

def createOutputFile():
    outputFile = open("output.txt", "w+")

    #for k, v in minCostParent.items(): print("key: " + k + " val: " + v + "\t")

    if goalGrid in minCostToReachGridPoint:
        goalCost = minCostToReachGridPoint[goalGrid]
        outputFile.write(str(goalCost) + "\n")
    else:
        #Then we never have any path from the start to goal
        outputFile.write("FAIL")
        return

    if goalGrid in minCostParent: #minCostPATHToReachGridPoint:
        #goalPath = minCostPATHToReachGridPoint[goalGrid]

        goalPath = []

        pathBuildCurrentNode = goalGrid
        while pathBuildCurrentNode != startGrid:
            goalPath.insert(0,pathBuildCurrentNode)
            pathBuildCurrentNode = minCostParent[pathBuildCurrentNode]

        goalPath.insert(0,startGrid)
        #pathArr = goalPath.split(",")
        outputFile.write(str(len(goalPath)) + "\n")
        #outputFile.write(startGrid + " 0\n")

        for i in range(0,len(goalPath)):
            if i == 0:
                moveCost = 0
            else:
                moveCost = computeMoveCostBetweenAnyTwoPoints(goalPath[i], goalPath[i-1])

            #print("Move Cost: " + str(moveCost) + "\tFirst: " + str(goalPath[i]) + "\tSecond: " + str(goalPath[i-1]))
            if i == len(goalPath) - 1:
                #If its the last item, we dont want to print a new line, at least given the example outputs we were given
                outputFile.write(goalPath[i] + " " + str(moveCost))
            else:
                outputFile.write(goalPath[i] + " " + str(moveCost) + "\n")


if __name__ == '__main__':
    print("Processing Input - Time: " + str(time.ctime()))
    processInputFile()

    print("Running Logic - Time: " + str(time.ctime()))
    threeDBFSToAll()

    print("Creating Output - Time: " + str(time.ctime()))
    createOutputFile()


