import numpy as np
import matplotlib.pyplot as plt
import time
from matplotlib import patches
from matplotlib.animation import FuncAnimation

""" grid-world environment """


class GridEnv:

    def update_curpos(self, i):
        self.curr_pos = self.positions[i]
        self.point.set_data(self.curr_pos[0] + 0.5, self.curr_pos[1] + 0.5)
        return self.point,

    def valid_obstacle(self, obs_org):
        valid_obs = False
        x_o, y_o = obs_org
        if x_o + self.obstacle_size <= self.size and y_o + self.obstacle_size <= self.size:
            print('here')
            if not (x_o <= self.goal_pos[0] <= x_o + self.obstacle_size and
                    y_o <= self.goal_pos[1] <= y_o + self.obstacle_size):
                if not (x_o <= self.start_pos[0] <= x_o + self.obstacle_size and
                        y_o <= self.start_pos[1] <= y_o + self.obstacle_size):
                    valid_obs = True
        print(obs_org, valid_obs)
        return valid_obs

    def generate_obs(self):
        count = 0
        obs = []
        while count < self.num_obs:
            x_o = np.random.randint(0, self.size)
            y_o = np.random.randint(0, self.size)
            obs_org = np.array([x_o, y_o])
            print(obs_org)
            if self.valid_obstacle(obs_org):
                if not np.all(np.any([obs_org == x for x in obs],axis=0)):
                    count += 1
                    for i in range(self.obstacle_size):
                        for j in range(self.obstacle_size):
                            obs.append([x_o + i, y_o + j])
        print(obs)
        return obs

    def __init__(self, size, obstacle_size, num_obs, start_pos, goal_pos, is_render):
        self.size = size
        self.obstacle_size = obstacle_size
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
            ax.add_patch(patches.Rectangle(self.start_pos, 1, 1, color='blue', label='Start'))
            ax.add_patch(patches.Rectangle(self.goal_pos, 1, 1, color='green', label='Goal'))
            ax.set(xlim=(0, self.size), ylim=(0, self.size))
            ax.set(xticks=[i for i in range(self.size)], yticks=[i for i in range(self.size)])
            plt.grid(linestyle="-", color='black')
            self.point, = plt.plot([],[],'ro')
            env_ani = FuncAnimation(fig, self.update_curpos, interval=200)
            plt.show()

    def state_transition(self):
        x_c, y_c = self.curr_pos[0], self.curr_pos[1]
        if self.action == 1:
            # north
            if y_c != self.size-1:
                new_pos = np.array([x_c, y_c + 1])
            else:
                new_pos = np.array([x_c, y_c])
        elif self.action == 2:
            # south
            if y_c != 0:
                new_pos = np.array([x_c, y_c - 1])
            else:
                new_pos = np.array([x_c, y_c])
        elif self.action == 3:
            # left
            if x_c != 0:
                new_pos = np.array([x_c - 1, y_c])
            else:
                new_pos = np.array([x_c, y_c])
        elif self.action == 4:
            # right
            if x_c != self.size-1:
                new_pos = np.array([x_c + 1, y_c])
            else:
                new_pos = np.array([x_c, y_c])
        else:
            new_pos = np.array([x_c, y_c])
            print('self.action not valid')
        return new_pos

    def get_reward(self):
        new_pos = self.state_transition()
        # print(new_pos)
        # print(np.all((np.any([new_pos == obs for obs in self.obstacles],axis=0))))
        if np.all(new_pos == self.goal_pos):
            # reward for reaching the goal, exit
            reward = 1.0
            print('found goal')
            done = True
        elif np.any(np.all([new_pos == obs for obs in self.obstacles], axis=1)):
            # penalty for hitting an obstacle, exit
            reward = -1.0
            print('hit obstacle')
            done = False
        elif np.all(new_pos == self.curr_pos):
            # penalty for hitting walls
            print('hit wall')
            reward = -1.0
            done = False
        else:
            # penalty for another time-step
            reward = 0
            done = False
        return reward, done

    def step(self, action):
        self.action = action
        new_pos = self.state_transition()
        reward, done = self.get_reward()
        self.positions.append(new_pos)
        self.curr_pos = new_pos
        self.reward = reward
        return new_pos, self.reward, done