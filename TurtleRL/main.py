# -*- coding: utf-8 -*-
import numpy as np
from turtlebot_env.basic_env import environ
from agents.dqn import DQNAgent
import argparse

def arguments():
    return
if __name__ == '__main__':
    env = environ()
    agent = DQNAgent(env,args)
    agent.train()