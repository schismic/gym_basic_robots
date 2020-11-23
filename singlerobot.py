import gym
import gym_warehouse

def h (pos1, pos2):
    return abs(pos1[0]-pos2[0])**2+ abs(pos1[1]-pos2[1])**2

def run_agent():
    # create the Gym environment
    env = gym.make('singlerobot-warehouse-v0')
    crossed_tiles = []   # keep taken paths here (g)
    pos =  None  # position of the robot
    target_pos =  None  # position of the target
    
    while True:
        env.render()    # you can used this for printing the environment

        # sense
        s = env.look()
        
        # think

        # find the position of the robot and the target in the first loop
        # (only in the first cause of efficiency)
        if not pos:
            robot = 'a'
            pos = [[x, y.index(robot)] for x, y in enumerate(s) if robot in y][0]
            crossed_tiles.append(pos)
        if not target_pos:
            target = 'A'
            target_pos = [[x, y.index(target)] for x, y in enumerate(s) if target in y][0]

        # check f (=g+h) for surrounding tiles
        tile_to_cross= None
        tile_f = float('inf')
        action = env.ACTION_WAIT

        # down
        if s[pos[0] + 1]:
            down_object = s[pos[0] + 1][pos[1]]
            down_pos = [pos[0] + 1, pos[1]]
            if down_pos not in crossed_tiles and not down_object=='*':
                down_f = len(crossed_tiles) + h(down_pos, target_pos)
                if down_f < tile_f:
                        tile_f = down_f
                        tile_to_cross = down_pos
                        action = env.ACTION_DOWN
        # right
        if s[0][pos[1] + 1]:
            right_object = s[pos[0]][pos[1]+1]
            right_pos = [pos[0], pos[1] + 1]
            if  right_pos not in crossed_tiles and not right_object=='*':
                right_f = len(crossed_tiles) + h(right_pos, target_pos)
                if right_f < tile_f:
                    tile_f = right_f
                    tile_to_cross = right_pos
                    action = env.ACTION_RIGHT
        # up
        if s[pos[0] - 1]:
            up_object = s[pos[0] - 1][pos[1]]
            up_pos = [pos[0] - 1, pos[1]]
            if  up_pos not in crossed_tiles and not up_object=='*':
                up_f = len(crossed_tiles) + h(up_pos, target_pos)
                if up_f < tile_f:
                    tile_f = up_f
                    tile_to_cross = up_pos
                    action = env.ACTION_UP
                
        # left
        if s[0][pos[1] - 1]:
            left_object = s[pos[0]][pos[1] - 1]
            left_pos = [pos[0], pos[1] - 1]
            if  left_pos not in crossed_tiles and not left_object=='*':
                left_f = len(crossed_tiles) + h(left_pos, target_pos)
                if left_f < tile_f:
                    tile_f = left_f
                    tile_to_cross = left_pos
                    action = env.ACTION_LEFT
        
        # act
        crossed_tiles.append(tile_to_cross)
        ob, rew, done = env.step(action)
        
        # update the position of the robot
        pos = tile_to_cross
        
        if done:
            break

    env.close()


if __name__ == "__main__":
    run_agent()


