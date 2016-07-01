'''
Created on 08/08/2014

@author: Gabriel de O. Ramos <goramos@inf.ufrgs.br>
'''
from environment import Environment

class EnvTest1(Environment):
    
    def __init__(self):
        
        #create the environment as a dictionary
        self._env = {"s1" : ["s1>s2","s1>s3"], "s2": ["s2>s1", "s2>s3"], "s3": ["null"]}
        
        #definition of starting and goal states
        self._starting_state = "s1"
        self._goal_state = "s3"
        
    
    #return the actions available in the state
    def get_state_actions(self, state):
        return self._env[state]
    
    #perform action in state and return a new state 
    #and the corresponding reward
    def perform_action(self, state, action):
        new_state = action.split(">")[1]
        r = -5
        if new_state == self._goal_state: #goal state
            r = 10
        return [new_state, r]
        
    def get_starting_state(self):
        return self._starting_state
    
    def get_goal_state(self):
        return self._goal_state
    
    