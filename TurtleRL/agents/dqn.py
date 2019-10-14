# -*- coding: utf-8 -*-
import random
import gym
import numpy as np
from collections import deque
from keras.models import Sequential
from keras.layers import Dense
from keras.optimizers import Adam, RMSprop
from keras import backend as K
import copy
import tensorflow as tf
import csv


import environment
from environment import multimodal
import numpy as np
import random


import numpy as np
import math
#from keras.initializations import normal, identity
from keras.models import model_from_json, load_model
from keras.layers import Concatenate,Input, Dense, concatenate
#from keras.engine.training import collect_trainable_weights
from keras.models import Sequential
from keras.layers import Dense, Flatten, Input, merge, Lambda, Activation
from keras.models import Sequential, Model
from keras.optimizers import Adam, RMSprop
import keras.backend as K
import tensorflow as tf

HIDDEN1_UNITS = 150
HIDDEN2_UNITS = 75
HIDDEN3_UNITS = 50

EPISODES = 5000

def interpret_action(action):
    print("Interpreting action ",action)
    agent1action = [0.0,0.0,0.0]
    agent2action = [0.0,0.0,0.0] #For now do nothing with agent 2
    if(action[0]==0):
        agent1action[0]=1.1
    if(action[0]==1):
        agent1action[0]=0.0
    if(action[0]==2):
        agent1action[0]=-1.1

    if(action[1]==3):
        agent1action[1]=1.1
    if(action[1]==4):
        agent1action[1]=0.0
    if(action[1]==5):
        agent1action[1]=-1.1
    return agent1action,agent2action

class DQNAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.memory = deque(maxlen=2000)
        self.memlen = 0
        self.loss = 0
        self.gamma = 0.95    # discount rate
        self.epsilon = 0.1  # exploration rate
        self.epsilon_min = 0.05
        self.epsilon_decay = 0.99
        self.learning_rate = 0.001
        self.TAU = 0.01
        self.model = self._build_model()
        self.target_model = self._build_model()
        self.update_target_model()

    def _huber_loss(self, y_true, y_pred, clip_delta=1.0):
        error = y_true - y_pred
        cond  = K.abs(error) <= clip_delta

        squared_loss = 0.5 * K.square(error)
        quadratic_loss = 0.5 * K.square(clip_delta) + clip_delta * (K.abs(error) - clip_delta)

        return K.mean(tf.where(cond, squared_loss, quadratic_loss))

    def _build_model(self):
        # Neural Net for Deep-Q learning Model
        print("Now we build the model")
        
        color = Input(shape=[self.state_size*3])
        c00 = Dense(150, activation='relu')(color)
        c10 = Dense(75, activation='relu')(c00)
        c20 = Dense(50, activation='relu')(c10)

        depth = Input(shape=[self.state_size])
        d00 = Dense(100, activation='relu')(depth)
        d10 = Dense(75, activation='relu')(d00)
        d20 = Dense(50, activation='relu')(d10)

        M1 = concatenate([c20,d20]) #color and depth fusion

        ag1_traj = Input(shape=[40]) #Hardcoded for now (position x,y and orientation of the agent for past 10 steps)
        ag100 = Dense(30, activation='relu')(ag1_traj)
        ag110 = Dense(30, activation='relu')(ag100)
        ag120 = Dense(25, activation='relu')(ag110)

        h00 = Dense(50, activation='relu')(M1)
        h10 = Dense(30, activation='relu')(h00)
        h20 = Dense(25, activation='relu')(h10)

        h01 = Dense(30, activation='relu')(ag120)
        h11 = Dense(25, activation='relu')(h01)
        h21 = Dense(25, activation='relu')(h11)

        M3 = concatenate([h20,h21]) #multimodal state action fusion
        m1 = Dense(30, activation='relu')(M3)
        m2 = Dense(25, activation='relu')(m1)
        m2l = Dense(25, activation='relu')(m2)

        m3a1 = Dense(25, activation='relu')(m2l)
        X = Dense(3,activation='tanh')(m3a1) #x and y velocity

        m3a2 = Dense(25, activation='relu')(m2l)
        Y = Dense(3,activation='tanh')(m3a2) #x and y velocity

        VV = concatenate([X,Y])

        model = Model(input=[color,depth,ag1_traj],output=VV)

        model.compile(loss=self._huber_loss,optimizer=Adam(lr=self.learning_rate))
        #model.compile(loss=self._huber_loss,optimizer=RMSprop(lr=0.001, rho=0.9, epsilon=None, decay=0.0))
        print("Successfully built and compiled model")
        return model

    def update_target_model(self):
        # copy weights from model to target_model
        #self.target_model.set_weights(self.model.get_weights())

        weights=[]
        target_weights=[]

        weights = self.model.get_weights()
        target_weights = self.target_model.get_weights()

        for i in range(len(weights)):
            target_weights[i] = self.TAU * weights[i] + (1 - self.TAU)* target_weights[i]
        self.target_model.set_weights(target_weights)

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))
        self.memlen +=1

    def act(self, state):
        if np.random.rand() <= self.epsilon:
            print("Random action taken ")
            x_action = random.randrange(3)
            y_action = random.randrange(3) + 3
            return [x_action,y_action]
        act_values = self.model.predict(state)
        print("Model predicted ",act_values)
        x_action = np.argmax(act_values[0][:-3])
        y_action = np.argmax(act_values[0][-3:]) + 3 
        return [x_action,y_action]  # returns action

    def replay(self, batch_size):
        minibatch = random.sample(self.memory, batch_size)

        states_color=np.zeros((1,600))
        states_depth=np.zeros((1,200))
        states_traj=np.zeros((1,40))
        target_fs=np.zeros((1,6)) #7*3 for actions and 2 for switching

        for state, action, reward, next_state, done in minibatch:
            #print("Got state ",state)
            target = self.model.predict(state)
            if done:
                target[0][action[0]] = reward
                target[0][action[1]] = reward
            else:
                # a = self.model.predict(next_state)[0]
                t = self.target_model.predict(next_state)[0]
                target[0][action[0]] = reward + self.gamma * np.amax(t[0:3])
                target[0][action[1]] = reward + self.gamma * np.amax(t[3:6])
                # target[0][action] = reward + self.gamma * t[np.argmax(a)]
            
            states_color=np.vstack((states_color,state[0]))
            states_depth=np.vstack((states_depth,state[1]))
            states_traj=np.vstack((states_traj,state[2]))

            target_fs=np.vstack((target_fs,target))
        #self.model.fit(state, target, epochs=1, verbose=0) #need to add train on batch
        #self.history=self.model.fit([states_color[1:],states_depth[1:],states_traj[1:]], target_fs[1:], epochs=1, verbose=2)#One minibatch update
        self.loss=self.model.train_on_batch([states_color[1:],states_depth[1:],states_traj[1:]], target_fs[1:])#One minibatch update
        self.model.save_weights("weights/model_weights.h5")
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def load(self, name):
        self.model.load_weights(name)

    def save(self, name):
        self.model.save_weights(name)


if __name__ == "__main__":
    
    agent = DQNAgent(200, 6)
    env = multimodal()
    # agent.load("./save/cartpole-ddqn.h5")
    done = False
    batch_size = 32

    for e in range(EPISODES):
        state,reward,done,_ = env.reset()
        #state = np.reshape(state, [1, state_size])
        for time in range(500):
            # env.render()
            action = agent.act(state)
            a1t,a2t = interpret_action(action)
            next_state, reward, done, _ = env.step(a1t,a2t)
            print("Got reward ",reward)
            #reward = reward if not done else -10
            #next_state = np.reshape(next_state, [1, state_size])
            agent.remember(state, action, reward[0], next_state, done)
            state = copy.copy(next_state)
            if done:
                agent.update_target_model()
                print("episode: {}/{}, score: {}, e: {:.2}".format(e, EPISODES, time, agent.epsilon))
                break
            print("memory length ",agent.memlen)
            if agent.memlen>32 and agent.memlen%32==0:
                print("Replaying ...")
                agent.replay(batch_size)
            if agent.memlen>10000:
                agent.memlen=0

            fname = 'rewards.csv'
            file1 = open(fname, 'a')
            writer = csv.writer(file1)
            fields1=[e,time,reward[0],agent.loss,done]
            writer.writerow(fields1)
            file1.close()
