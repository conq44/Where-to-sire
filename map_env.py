import numpy as np
import matplotlib.pyplot as plt
import time
from matplotlib import patches
from matplotlib.animation import FuncAnimation

""" grid-world environment """

OBSTACLE_SIZE = 3
START = (0, 0)


class GridEnv:

    def update_curpos(self, i):
        self.curr_pos = self.positions[i]
        plt.plot(self.curr_pos[0] + 0.5, self.curr_pos[1] + 0.5, 'ro')
        return self.curr_pos

    def valid_obstacle(self, obs_org):
        valid_obs = False
        x_o, y_o = obs_org
        if x_o + OBSTACLE_SIZE < self.size and y_o + OBSTACLE_SIZE < self.size:
            if not (x_o < self.goal_pos[0] <= x_o + OBSTACLE_SIZE and
                    y_o <= self.goal_pos[1] <= y_o + OBSTACLE_SIZE):
                if not (x_o <= self.start_pos[0] <= x_o + OBSTACLE_SIZE and
                        y_o <= self.start_pos[1] <= y_o + OBSTACLE_SIZE):
                    valid_obs = True
        return valid_obs

    def generate_obs(self):
        count = 0
        obs = []
        while count < self.num_obs:
            x_o = np.random.randint(0, self.size)
            y_o = np.random.randint(0, self.size)
            obs_org = (x_o, y_o)
            if self.valid_obstacle(obs_org):
                count += 1
                for i in range(OBSTACLE_SIZE):
                    for j in range(OBSTACLE_SIZE):
                        obs.append((x_o + i, y_o + j))
        return obs

    def __init__(self, size, num_obs, start_pos, goal_pos, is_render):
        self.size = size
        self.num_obs = num_obs
        self.start_pos = start_pos
        self.curr_pos = start_pos
        self.goal_pos = goal_pos
        self.render = is_render
        self.action = None
        self.obstacles = self.generate_obs()
        self.positions = []
        self.done = False
        self.reward = 0

    def render_env(self):
        global env_ani
        if self.render:
            fig = plt.figure()
            ax = fig.add_subplot(111, aspect='equal')
            for obs in self.obstacles:
                loc = obs
                ax.add_patch(patches.Rectangle(loc, 1, 1, color='grey', label='Obstacle'))
            ax.add_patch(patches.Rectangle(START, 1, 1, color='blue', label='Start'))
            ax.add_patch(patches.Rectangle(self.goal_pos, 1, 1, color='green', label='Goal'))
            ax.set(xlim=(0, self.size), ylim=(0, self.size))
            ax.set(xticks=[i for i in range(self.size)], yticks=[i for i in range(self.size)])
            plt.grid(linestyle="-", color='black')
            env_ani = FuncAnimation(fig, self.update_curpos)
            plt.show()

    def state_transition(self):
        x_c, y_c = self.curr_pos
        if self.action == 1:
            # north
            if y_c != self.size:
                new_pos = (x_c, y_c + 1)
        elif self.action == 2:
            # south
            if y_c != 0:
                new_pos = (x_c, y_c - 1)
        elif self.action == 3:
            # left
            if x_c != 0:
                new_pos = (x_c - 1, y_c)
        elif self.action == 4:
            # right
            if x_c != self.size:
                new_pos = (x_c + 1, y_c)
        else:
            new_pos = (x_c, y_c)
            print('self.action not valid')
        return new_pos

    def get_reward(self):
        new_pos = self.state_transition()
        if new_pos == self.goal_pos:
            # reward for reaching the goal, exit
            reward = 1.0
            done = True
        elif new_pos in self.obstacles:
            # penalty for hitting an obstacle, exit
            reward = -1.0
            done = True
        else:
            # penalty for another time-step
            reward = -0.001
            done = False
        return reward, done

    def step(self, action):
        self.action = action
        new_pos = self.state_transition()
        reward, done = self.get_reward()
        self.positions.append(new_pos)
        self.curr_pos = new_pos
        self.reward += reward
        return new_pos, self.reward, done


grid = GridEnv(20, 10, (0, 0), (15, 15), is_render=True)
for i in range(100):
    if i % 2 == 0:
        action = 1
    else:
        action = 4
    np, r, d = grid.step(action)
    print(np, r, d)
    if d:
        break
print(grid.positions)
grid.render_env()
