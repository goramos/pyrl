'''
Created on 04/08/2014

@author: Gabriel de O. Ramos <goramos@inf.ufrgs.br>
'''
from exploration import ExplorationStrategy
from tools import sampling
import random

class EpsilonGreedy(ExplorationStrategy):
    
    def __init__(self, epsilon=1, decay_rate=0.99):#to avoid decay rate, set it to 0.0 
        self._epsilon_ini = epsilon
        self._epsilon = epsilon
        self._decay_rate = decay_rate
        
    #Return an action, given an actions dictionary in the form action:Q-value
    def choose(self, action_dict):
        i = -1
        
        #select an action
        r = random.random()
        if r < self._epsilon:
            #select an action uniformly at random (exploration)
            i = random.randint(0, len(action_dict)-1)
        else:
            #select an action greedily (exploitation)
            i = sampling.reservoir_sampling(action_dict.values(), True)
        
        #update epsilon value
        if self._decay_rate > 0.0:
            self._epsilon = self._epsilon * self._decay_rate
        
        #return selected action
        return action_dict.keys()[i]
    
    def reset_episodic(self):
        #epsilon greedy is not episodic, so nothing to do here
        pass
    
    def reset_all(self):
        self._epsilon = self._epsilon_ini