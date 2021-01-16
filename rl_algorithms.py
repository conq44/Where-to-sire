import numpy as np
import matplotlib.pyplot as plt
import time
from matplotlib import patches
from matplotlib.animation import FuncAnimation
from map_env import *


# Parameters

# SIZE = 5
# OBSTACLE_SIZE = 1
# NUM_OBS = 3
# NUM_ACTIONS = 4
# START = [0, 0]
# END = [4, 4]
# EPSILON = 0.7
# DISC_FACTOR = 0.9
# lr = 0.1

SIZE = 5
OBSTACLE_SIZE = 1
NUM_OBS = 4
NUM_ACTIONS = 4
START = [0, 0]
END = [4, 4]
EPSILON = 0.7
DISC_FACTOR = 0.9
lr = 0.1
lam = 0.9
# Helper functions


def choose_action(values):
    """ chooses action based on epsilon greedy strategy """
    j = np.random.rand()
    if j < EPSILON:
        act = np.random.randint(1,4)
    else:
        act = np.argmax(values)
    return act+1

# Algorithms


def Q_learning():
    """ Performs Q learning and finds a path to the goal """
    count = 0
    Q = np.zeros((SIZE, SIZE, NUM_ACTIONS))
    grid = GridEnv(SIZE, OBSTACLE_SIZE, NUM_OBS, START, END, is_render=True)
    while count < 100000:
        current_state = [cord for cord in grid.curr_pos]
        values = Q[current_state[0], current_state[1]]
        action = choose_action(values)   # choose an action. epsilon greedy?
        new_state, immediate_reward, done = grid.step(action)   # take action, get reward and new state
        future_value = DISC_FACTOR*np.amax(Q[new_state[0], new_state[1]])   # discounted future reward from new state
        current_value = Q[current_state[0], current_state[1], action-1]   # the current state action pair Q value
        diff = future_value - current_value
        Q[current_state[0], current_state[1], action-1] += lr*(immediate_reward + diff)
        if done:
            grid.curr_pos = START
        else:
            grid.curr_pos = new_state
        count += 1
        if count == 100000:
            print(Q)
    grid.render_env()   # shows the exploration stage
    done = False
    grid.curr_pos = START
    grid.positions = []
    while not done:
        values = Q[grid.curr_pos[0], grid.curr_pos[1]]
        action = np.argmax(values)+1
        new_state, reward, done = grid.step(action)
        print(action, new_state)
        grid.curr_pos = new_state
    grid.render_env()   # shows the path found


def Sarsa_lambda():
    """ Performs Sarsa and finds a path to the goal """
    count = 0
    Q = np.zeros((SIZE, SIZE, NUM_ACTIONS))
    grid = GridEnv(SIZE, OBSTACLE_SIZE, NUM_OBS, START, END, is_render=True)
    N = np.zeros((SIZE, SIZE, NUM_ACTIONS))
    current_state = START   # s_0
    action = 3  # a_0
    episode_count = 0
    while count < 10000:
        new_state, immediate_reward, done = grid.step(action)
        next_values = Q[new_state[0], new_state[1]]
        next_action = choose_action(next_values)
        N[current_state[0], current_state[1], action-1] += 1
        diff = DISC_FACTOR*Q[new_state[0], new_state[1], next_action-1] - Q[current_state[0], current_state[1], action-1]
        delta = immediate_reward + diff
        # update
        Q += lr*delta*N
        N = DISC_FACTOR*lam*N
        count += 1
        if episode_count > SIZE*SIZE:
            done = True
        if done:
            # back to s_0,a_0
            grid.curr_pos = START
            current_state = START
            action = 3
            N = np.zeros((SIZE, SIZE, NUM_ACTIONS))
            episode_count = 0
        else:
            grid.curr_pos = new_state
            current_state = new_state
            action = next_action
            episode_count += 1
        if count == 10000:
            print(Q)
    grid.render_env()  # shows the exploration stage
    done = False
    grid.curr_pos = START
    grid.positions = []
    while not done:
        values = Q[grid.curr_pos[0], grid.curr_pos[1]]
        action = np.argmax(values) + 1
        new_state, reward, done = grid.step(action)
        print(action, new_state)
        grid.curr_pos = new_state
    grid.render_env()  # shows the path found


# Sarsa_lambda()
Q_learning()
