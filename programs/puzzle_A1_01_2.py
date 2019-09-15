import os
import copy
import sys


class State(object):

    def __init__(self, rep, cost, coor = None, heuristic = None):
        self.cost = cost
        self.rep = rep
        self.eligible_actions = ["UP", "RIGHT", "DOWN", "LEFT"]
        if coor is None:
            for i in range(0, 3):
                for j in range (0, 3):
                    if rep[i][j] == 0:
                        self.coor_empty = [i, j]
                        break
        else:
            self.coor_empty = coor
        if self.coor_empty[0] == 0:
            self.eligible_actions.remove("DOWN")
        if self.coor_empty[0] == 2:
            self.eligible_actions.remove("UP")
        if self.coor_empty[1] == 0:
            self.eligible_actions.remove("RIGHT")
        if self.coor_empty[1] == 2:
            self.eligible_actions.remove("LEFT")

        if heuristic is None:
            self.heuristic = self.estimate(self.rep)
        else:
            self.heuristic = heuristic
        # print(self.heuristic)
        # print("+++")
        self.path = []

    def setPath(self, path):
        self.path = path

    def estimate(self, rep):
        answer = 0
        for i in range(0, 3):
            for j in range(0, 3):
                value = rep[i][j]
                goal_position = [int((value - 1) / 3), (value - 1) % 3]
                if rep[i][j] != 0 and [i, j] != goal_position:
                    answer+=1 
        return answer

    def act(self, action):

        if not action in self.eligible_actions:
            return "Not a legitimate action"

        next_state = copy.deepcopy(self.rep)
        next_coor = copy.copy(self.coor_empty)
        next_path = copy.copy(self.path)
        next_path.append(action)

        if action == "UP":
            next_state[self.coor_empty[0] + 1][self.coor_empty[1]] = 0
            next_state[self.coor_empty[0]][self.coor_empty[1]] = self.rep[self.coor_empty[0] + 1][self.coor_empty[1]]
            next_coor[0] = next_coor[0] + 1
            # if (self.rep[self.coor_empty[0] + 1][self.coor_empty[1]] - 1) / 3 <= self.coor_empty[0]:
            #     next_heuristic -= 1
            # else:
            #     next_heuristic += 1
        if action == "RIGHT":
            next_state[self.coor_empty[0]][self.coor_empty[1] - 1] = 0
            next_state[self.coor_empty[0]][self.coor_empty[1]] = self.rep[self.coor_empty[0]][self.coor_empty[1] - 1]
            next_coor[1] = next_coor[1] - 1
            # if (self.rep[self.coor_empty[0]][self.coor_empty[1] - 1] - 1) % 3 >= self.coor_empty[1]:
            #     next_heuristic -= 1
            # else:
            #     next_heuristic += 1
        if action == "DOWN":
            next_state[self.coor_empty[0] - 1][self.coor_empty[1]] = 0
            next_state[self.coor_empty[0]][self.coor_empty[1]] = self.rep[self.coor_empty[0] - 1][self.coor_empty[1]]
            next_coor[0] = next_coor[0] - 1
            # if (self.rep[self.coor_empty[0] - 1][self.coor_empty[1]] - 1) / 3 >= self.coor_empty[0]:
            #     next_heuristic -= 1
            # else:
            #     next_heuristic += 1
        if action == "LEFT":
            next_state[self.coor_empty[0]][self.coor_empty[1] + 1] = 0
            next_state[self.coor_empty[0]][self.coor_empty[1]] = self.rep[self.coor_empty[0]][self.coor_empty[1] + 1]
            next_coor[1] = next_coor[1] + 1
            # if (self.rep[self.coor_empty[0]][self.coor_empty[1] + 1] - 1) % 3 <= self.coor_empty[0]:
            #     next_heuristic -= 1
            # else:
            #     next_heuristic += 1
        next_heuristic = self.estimate(next_state)
        next = State(next_state, self.cost + 1, next_coor, next_heuristic)
        next.setPath(next_path)
        # print(next)
        return next

    def compareTo(self, other):
        if not isinstance(other, State):
            raise NameError("Not a state")

        # print(self.cost)
        return self.heuristic + self.cost - other.heuristic - other.cost

    def hash(self):
        answer = 0
        multiplier = 1
        for i in range(0, 3):
            for j in range(0, 3):
                answer = answer + multiplier * self.rep[i][j]
                multiplier = multiplier * 9
        return answer

    def toString(self):
        s = ""
        for i in range(0, 3):
            for j in range(0, 3):
                if self.rep[i][j] == 0:
                    s += "."
                else:
                    s += str(self.rep[i][j])
            s += "\n"
        s += "f = " + str(self.cost + self.heuristic)
        return s


class PriorityQueue(object):
    def __init__(self):
        self.queue = []
        self.count = 0

    def add(self, state):
        self.queue.append(state)
        self.count = self.count + 1
        if self.count <= 1:
            return
        index = self.count - 1
        parent = (index - 1) / 2
        # print(index, ' ', parent)
        while self.queue[int(index)].compareTo(self.queue[int(parent)]) < 0:
            temp = self.queue[index]
            self.queue[index] = self.queue[parent]
            self.queue[parent] = temp
            index = parent
            if index == 0:
                return
            parent = (index - 1) / 2

    def pop(self):
        answer = self.queue[0]
        self.queue[0] = self.queue[self.count - 1]
        self.queue.pop(self.count - 1)
        self.count = self.count - 1
        index = 0
        while True:
            if index * 2 + 1 >= self.count:
                break
            if index * 2 + 2 >= self.count:
                if self.queue[index].compareTo(self.queue[index * 2 + 1]) > 0:
                    temp = self.queue[index]
                    self.queue[index] = self.queue[index * 2 + 1]
                    self.queue[index * 2 + 1] = temp
                break
            if self.queue[index * 2 + 1].compareTo(self.queue[index * 2 + 2]) > 0:
                if self.queue[index].compareTo(self.queue[index * 2 + 2]) > 0:
                    temp = self.queue[index]
                    self.queue[index] = self.queue[index * 2 + 2]
                    self.queue[index * 2 + 2] = temp
                    index = index * 2 + 2
                else:
                    break
            else:
                if self.queue[index].compareTo(self.queue[index * 2 + 1]) > 0:
                    temp = self.queue[index]
                    self.queue[index] = self.queue[index * 2 + 1]
                    self.queue[index * 2 + 1] = temp
                    index = index * 2 + 1
                else:
                    break
        return answer


class Puzzle(object):
    def __init__(self, init_state, goal_state):
        # You may add more attributes as necessary
        self.init_state = init_state
        self.goal_state = goal_state
        self.actions = list()
        self.solvable = True

    def solve(self):
        initial = State(init_state, 0)
        p_queue = PriorityQueue()
        p_queue.add(initial)
        explored = {initial.hash()}
        while p_queue.count > 0:
            state = p_queue.pop()
            if self.isGoal(state):
                return state.path
            for action in state.eligible_actions:
                next_state = state.act(action)
                if next_state.hash() in explored:
                    continue
                explored.add(next_state.hash())
                p_queue.add(next_state)
        return ['UNSOLVABLE']
        
    def isGoal(self, state):
        for i in range(0, 3):
            for j in range(0, 3):
                if state.rep[i][j] != self.goal_state[i][j]:
                    return False
        return True

    # You may add more (helper) methods if necessary.
    # Note that our evaluation scripts only call the solve method.
    # Any other methods that you write should be used within the solve() method.

if __name__ == "__main__":
    # init_state = [[1,2,3],[4,5,6],[8,7,0]]
    # goal_state = [[1,2,3],[4,5,6],[7,8,0]]
    # puzzle = Puzzle(init_state, goal_state)
    # ans = puzzle.solve()
    # print(ans)


    # do NOT modify below
    if len(sys.argv) != 3:
        raise ValueError("Wrong number of arguments!")

    try:
        f = open(sys.argv[1], 'r')
    except IOError:
        raise IOError("Input file not found!")

    init_state = [[0 for i in range(3)] for j in range(3)]
    goal_state = [[0 for i in range(3)] for j in range(3)]
    lines = f.readlines()

    i,j = 0, 0
    for line in lines:
        for number in line:
            if '0'<= number <= '8':
                init_state[i][j] = int(number)
                j += 1
                if j == 3:
                    i += 1
                    j = 0

    for i in range(1, 9):
        goal_state[(i-1)//3][(i-1)%3] = i
    goal_state[2][2] = 0

    puzzle = Puzzle(init_state, goal_state)
    ans = puzzle.solve()

    with open(sys.argv[2], 'a') as f:
        for answer in ans:
            f.write(answer+'\n')
