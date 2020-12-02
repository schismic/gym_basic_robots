import gym
import gym_warehouse

# node/tile
class Tile:
    def __init__(self, parent, position):
        self.position = position
        self.parent = parent
        self.g = None
        self.f = None

# heuristic function: euclidian
def h (tile1, tile2):
    return abs(tile1[0]-tile2[0])**2 + abs(tile1[1]-tile2[1])**2

# function to find positions
def find(target, s):
    for x, y in enumerate(s):
        try:
            i = y.index(target)
        except ValueError:
            continue
        yield x, i
    
# a* search
def findPath(pos, target_pos, s):
    # initialize the start
    start_tile = Tile(None, pos)
    start_tile.g = start_tile.f = 0
    target_tile = Tile(None, target_pos)
    target_tile.g = target_tile.f = 0
    
    frontier = []
    expanded = []

    # start from the robot's position
    frontier.append(start_tile)
    
    while len(frontier)>0:

        # get the current tile (with the min cost)
        current_tile = frontier[0]
        current_index = 0
        for index, tile in enumerate(frontier):
            if tile.f < current_tile.f:
                current_tile = tile
                current_index = index

        # pop current and add it to the expanded
        frontier.pop(current_index)
        expanded.append(current_tile)

        # target reached?
        if current_tile.position==target_tile.position:
            path = []
            current = current_tile
            # build path
            while current.parent is not None:
                path.append(current.position)
                current = current.parent

            return path[::-1] # return reversed path

        # check reachable tiles
        children = []
        # down
        if current_tile.position[0] + 1 < len(s):
            down_object = s[current_tile.position[0] + 1][current_tile.position[1]]
            down_pos = [current_tile.position[0] + 1, current_tile.position[1]]
            if down_object != '*':
                down_tile = Tile(current_tile, down_pos)
                children.append(down_tile)

        # right
        if current_tile.position[1] + 1 < len(s[0]):
            right_object = s[current_tile.position[0]][current_tile.position[1]+1]
            right_pos = [current_tile.position[0], current_tile.position[1] + 1]
            if right_object != '*':
                right_tile = Tile(current_tile, right_pos)
                children.append(right_tile)
        
        # up
        if current_tile.position[0] - 1 >= 0:
            up_object = s[current_tile.position[0] - 1][current_tile.position[1]]
            up_pos = [current_tile.position[0] - 1, current_tile.position[1]]
            if up_object != '*':
                up_tile = Tile(current_tile, up_pos)
                children.append(up_tile)

        # left
        if current_tile.position[1] - 1 >= 0:
            left_object = s[current_tile.position[0]][current_tile.position[1] - 1]
            left_pos = [current_tile.position[0], current_tile.position[1] - 1]
            if left_object != '*':
                left_tile = Tile(current_tile, left_pos)
                children.append(left_tile)

        for child in children:
            # child expanded?
            in_expanded = False
            for exp_child in expanded:
                if child.position==exp_child.position:
                    in_expanded = True
            
            child.g = current_tile.g + 1
            child.f = child.g + h(target_tile.position, child.position)

            # child already in frontier?
            in_frontier = False
            for front_child in frontier:
                if child.position== front_child.position and child.g >= front_child.g:
                    in_frontier = True

            if not in_expanded and not in_frontier:
                frontier.append(child)
    
    return False

def run_agent():
    # create the Gym environment
    env = gym.make('singlerobot-warehouse-v0')

    # sense
    s = env.look()
    
    # think
    # find the positions of the robot and the target
    robot = 'a'
    pos = list(next(find(robot, s), None))
    target = 'A'
    target_pos = list(next(find(target, s), None))

    # find path
    path = findPath(pos, target_pos, s)

    if not path:
        return False
    
    # act
    t = 0
    while True:
        env.render()
        
        action = env.ACTION_WAIT
        if path[t][0] == pos[0] + 1:
            action = env.ACTION_DOWN
        elif path[t][1] == pos[1]  + 1:
            action = env.ACTION_RIGHT
        elif path[t][0] == pos[0] - 1:
            action = env.ACTION_UP
        elif path[t][1] == pos[1]  - 1:
            action = env.ACTION_LEFT

        # move
        ob, rew, done = env.step(action)

        # update robot's position
        pos = path[t]

        # step
        t += 1
        
        if done:
            break

    env.close()


if __name__ == "__main__":
    run_agent()


