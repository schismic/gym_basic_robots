import gym
import gym_warehouse

# node/tile
class Tile:
    def __init__(self, parent, position):
        self.position = position
        self.parent = parent
        self.g = None
        self.f = None

# to print failure if there is no global path found
def printFailure():
    word = 'FAILURE'
    k = 0
    for i in range(19):
        print()
        for j in range(19):
            if j%2==0:
                print(word[k%7], end='')
                k +=1
            else:
                print('.', end='')

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
def findPath(pos, target_pos, s, allpaths, blocked_tiles):
    # initialize the start and the target
    time = 0
    start_tile = Tile(None, pos + [time])
    start_tile.g = start_tile.f = 0
    target_tile = Tile(None, target_pos)
    target_tile.g = target_tile.f = 0
    
    open_list = []
    closed_list = []

    # start from the robot's position
    open_list.append(start_tile)
    
    while len(open_list)>0:

        time += 1
        # get the current tile (with the min cost)
        current_tile = open_list[0]
        current_index = 0
        for index, tile in enumerate(open_list):
            if tile.f < current_tile.f:
                current_tile = tile
                current_index = index

        # pop current and add it to the closed list
        open_list.pop(current_index)
        closed_list.append(current_tile)

        # target reached?
        if current_tile.position[:2]==target_tile.position:
            path = []
            current = current_tile
            # gotta fix time jumps here
            # count time
            correct_time = 0
            while current.parent is not None:
                correct_time += 1
                current = current.parent
            current = current_tile
            # build path
            while current.parent is not None:
                path.append(current.position[:2] + [correct_time])
                correct_time -= 1
                current = current.parent
            # check if path needs changing
            # tile in another path?
            for other_path in allpaths:
                for tile in range(len(path)):
                    other_path_2d = [other_tile[:2] for other_tile in other_path]

                    if path[tile][:2] in other_path_2d:
                        # check if there's a swap
                        other_index = other_path_2d.index(path[tile][:2])
                        
                        if other_index+1 < len(other_path) and tile+1 < len(path):
                            if path[tile+1][:2] == other_path_2d[other_index+1] \
                            and path[tile][2]+2 == other_path[other_index+1][2]:
                                #gotta find a new path
                                new_path = findPath(pos, target_pos, s, allpaths, blocked_tiles + [path[tile][:2]])
                                return new_path
                        if other_index-1 > 0 and tile-1 > 0:
                            if path[tile-1][:2] == other_path_2d[other_index-1] \
                            and path[tile][2]-2 == other_path[other_index-1][2]:
                                #gotta find a new path
                                new_path = findPath(pos, target_pos, s, allpaths, blocked_tiles + [path[tile][:2]])
                                
                                return new_path
                                
                        # if it's just a collision
                        if path[tile] in other_path:
                            # only wait
                            path.insert(tile+1, path[tile+1][:2] + [path[tile][2]])
                            # update times
                            for update_tile in range(tile+1):
                                path[update_tile][2] += 1
                            
            return path[::-1] # return reversed path

        # check reachable tiles
        children = []
        # down
        if current_tile.position[0] + 1 < len(s):
            down_object = s[current_tile.position[0] + 1][current_tile.position[1]]
            down_pos = [current_tile.position[0] + 1, current_tile.position[1]]
            if down_object != '*' and down_pos not in blocked_tiles:
                down_tile = Tile(current_tile, down_pos + [time])
                children.append(down_tile)

        # right
        if current_tile.position[1] + 1 < len(s[0]):
            right_object = s[current_tile.position[0]][current_tile.position[1]+1]
            right_pos = [current_tile.position[0], current_tile.position[1] + 1]
            if right_object != '*' and right_pos not in blocked_tiles:
                right_tile = Tile(current_tile, right_pos + [time])
                children.append(right_tile)
        
        # up
        if current_tile.position[0] - 1 >= 0:
            up_object = s[current_tile.position[0] - 1][current_tile.position[1]]
            up_pos = [current_tile.position[0] - 1, current_tile.position[1]]
            if up_object != '*' and up_pos not in blocked_tiles:
                up_tile = Tile(current_tile, up_pos + [time])
                children.append(up_tile)

        # left
        if current_tile.position[1] - 1 >= 0:
            left_object = s[current_tile.position[0]][current_tile.position[1] - 1]
            left_pos = [current_tile.position[0], current_tile.position[1] - 1]
            if left_object != '*' and left_pos not in blocked_tiles:
                left_tile = Tile(current_tile, left_pos + [time])
                children.append(left_tile)

        for child in children:
            # child in closed list?
            in_closed = False
            for closed_child in closed_list:
                if child.position[:2]==closed_child.position[:2]:
                    in_closed = True
            
            child.g = current_tile.g + 1
            child.f = child.g + h(target_tile.position, child.position)

            # child in open list?
            in_open = False
            for open_child in open_list:
                if child.position[:2] == open_child.position[:2] and child.g >= open_child.g:
                    in_open = True

            if not in_closed and not in_open:
                open_list.append(child)
            
    return False
    
def run_agent():
    # create the Gym environment
    env = gym.make('multirobot-warehouse-v0')
    allpaths = []    # paths for all the robots
    positions = []    # positions of the robots
    targets =  []  # positions of the targets
    time = 0
    
    # sense
    s = env.look()
    
    # think 
    # find the positions of the robots
    robot = 'a'
    for i in range(4):
        positions.append(list(next(find(robot, s), None)))
        r = ord(robot)
        r += 1
        robot = chr(r)
    # find the positions of their targets
    target = 'A'
    for i in range(4):
        targets.append(list(next(find(target, s), None)))
        t = ord(target)
        t += 1
        target = chr(t)

    # find paths
    if len(allpaths)==0:
        for robot in range(4):
            path = findPath(positions[robot], targets[robot], s, allpaths, [])
            if path:
                allpaths.append(path)
            else:
                printFailure()
                return False
    # act
    while True:
        env.render()
        time += 1
        actions = [env.ACTION_WAIT, env.ACTION_WAIT, env.ACTION_WAIT, env.ACTION_WAIT]

        for robot in range(4):
            if time <= len(allpaths[robot]):
                for move in allpaths[robot]:
                     if move[2] == time:
                        tile_to_cross = move
                        if tile_to_cross[0] == positions[robot][0] + 1:
                            actions[robot] = env.ACTION_DOWN
                        elif tile_to_cross[1] == positions[robot][1]  + 1:
                            actions[robot] = env.ACTION_RIGHT
                        elif tile_to_cross[0] == positions[robot][0] - 1:
                            actions[robot] = env.ACTION_UP
                        elif tile_to_cross[1] == positions[robot][1]  - 1:
                            actions[robot] = env.ACTION_LEFT
                        else:
                            actions[robot] = env.ACTION_WAIT
                        positions[robot] = tile_to_cross[:2]
                
        ob, rew, done = env.step([actions[0],  actions[1], actions[2], actions[3]])
        
        if done:
            break

    env.close()


if __name__ == "__main__":
    run_agent()
