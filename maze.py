class Node:
    def __init__(self, state, parent, action, exactCost, goal):
        self.state = state
        self.parent = parent
        self.action = action
        self.exactCost = exactCost
        estimatedCost = abs(self.state[0] - goal[0]) + abs(self.state[1] - goal[1])
        self.cost = exactCost + estimatedCost


class BFS:
    def __init__(self):
        self.mazeQueue = []

    def add(self, node):
        self.mazeQueue.append(node)

    def remove(self):
        if self.empty():
            raise Exception("mazeQueue is empty")
        else:
            node = self.mazeQueue[0]
            self.mazeQueue = self.mazeQueue[1:]
            return node

    def empty(self):
        return len(self.mazeQueue) == 0

    def contains_state(self, state):
        return any(node.state == state for node in self.mazeQueue)

    def printMethod(self):
        print("\nBFS Maze...")
        print


class DFS:
    def __init__(self):
        self.mazeStack = []

    def add(self, node):
        self.mazeStack.append(node)

    def remove(self):
        if self.empty():
            raise Exception("mazeStack is empty")
        else:
            node = self.mazeStack[-1]
            self.mazeStack = self.mazeStack[:-1]
            return node

    def empty(self):
        return len(self.mazeStack) == 0

    def contains_state(self, state):
        return any(node.state == state for node in self.mazeStack)

    def printMethod(self):
        print("\nDFS Maze...")


class Astar:
    def __init__(self):
        self.aStarArr = []

    def add(self, node):
        self.aStarArr.append(node)

    def remove(self):
        if self.empty():
            raise Exception("A* array is empty")
        else:
            lowCost = self.aStarArr[0].cost
            lowNode = self.aStarArr[0]
            index = 0
            for x in range(len(self.aStarArr)):
                if self.aStarArr[x].cost < lowCost:
                    lowNode = self.aStarArr[x]
                    index = x
            self.aStarArr.pop(index)
            return lowNode

    def empty(self):
        return len(self.aStarArr) == 0

    def contains_state(self, state):
        return any(node.state == state for node in self.aStarArr)

    def printMethod(self):
        print("\nA* Maze...")
        print


class Maze:
    def __init__(self):
        # Read in contents of maze
        f = open("maze.txt", "r")
        self.rowList = [[int(n) for n in line.split()] for line in f]
        f.close()

        for x in range(len(self.rowList[0])):
            if self.rowList[-1][x] == 0:
                self.start = (9, x)
            if self.rowList[0][x] == 0:
                self.end = (0, x)

    def neighbors(self, state):
        row, col = state
        neighborCells = [
            ("up", (row - 1, col)),
            ("down", (row + 1, col)),
            ("left", (row, col - 1)),
            ("right", (row, col + 1)),
        ]

        result = []
        for action, (r, c) in neighborCells:

            if 0 <= r < 10 and 0 <= c < 10 and self.rowList[r][c] != 1:
                result.append((action, (r, c)))
        return result

    def pathFinder(self, method):

        frontier = method
        # Keep track of visited cells
        self.num_visited = 0
        self.visited = []

        # Initialize frontier to just the starting position
        start = Node(
            state=self.start, parent=None, action=None, exactCost=1, goal=self.end
        )

        # For Depth-First Search, frontier should be set equal to DFS(),
        # for Breadth-First Search, frontier should be set equal to BFS()
        # and for A* Search, frontier shoul be set equal to Astar()
        frontier.printMethod()
        frontier.add(start)

        # Loop until maze is complete
        while True:

            # If the frontier is empty, then no path exists
            if frontier.empty():
                raise Exception("No solution!")

            # Choose a node from the frontier to explore
            node = frontier.remove()
            self.num_visited += 1

            # If node is equal to the end goal, then we found the solution
            if node.state == self.end:
                # populate cells array with the solution path
                cells = []
                cells.append(self.start)
                while node.parent is not None:
                    cells.append(node.state)
                    node = node.parent
                cells.reverse()
                self.solution = cells
                return

            # Mark node as visited
            self.visited.append(node.state)

            # Add neighbors to frontier
            for action, state in self.neighbors(node.state):
                if not frontier.contains_state(state):
                    for x in range(len(self.visited)):
                        if state == self.visited[x]:
                            exists = True
                            break
                        else:
                            exists = False
                    if exists == False:
                        self.cost = node.exactCost + 1
                        child = Node(
                            state=state,
                            parent=node,
                            action=action,
                            exactCost=self.cost,
                            goal=self.end,
                        )
                        frontier.add(child)

    def printMaze(self):
        for x in range(len(self.visited)):
            y = self.visited[x][0]
            z = self.visited[x][1]
            self.rowList[y][z] = 2

        solution = self.solution
        for x in range(len(solution)):
            i = solution[x][0]
            j = solution[x][1]
            self.rowList[i][j] = 5

        for x in range(len(self.rowList)):
            for y in range(len(self.rowList[0])):
                print(self.rowList[x][y], end="")
            print()


method = DFS()
m = Maze()
m.pathFinder(method)
m.printMaze()

method = BFS()
m = Maze()
m.pathFinder(method)
m.printMaze()

method = Astar()
m = Maze()
m.pathFinder(method)
m.printMaze()
