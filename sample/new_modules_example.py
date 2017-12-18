'''
Created on 9 de fev de 2017

@author: goramos
'''

from environment import Environment
from learner import Learner
import numpy as np

class SimpleEnv(Environment):
    
    def __init__(self):
        super(SimpleEnv, self).__init__()
        self.actions = ['actA', 'actB']
        self.calls_actB = 0
        
    def get_state_actions(self, state=None):
        return self.actions
    
    def run_episode(self):#, max_steps=-1):
        while True:
            self.run_step()
    
    def run_step(self):
        learner = self._learners.values()[0]
        s, a = learner.act_last()
        print 'Agent chosen %s' % a
        r = 1.0
        if a == 'actB':
            self.calls_actB += 1
            r = 1.0 / self.calls_actB
        learner.feedback_last(r, s)
    
    #IGNORE
    def has_episode_ended(self):
        return
    
class SimpleAg(Learner):
    
    def __init__(self, name, env):
        super(SimpleAg, self).__init__(name, env, self)
        #self.avgs[] = dict({a:0 for a in self._env.get_state_actions(self._starting_state)})
        
    def act_last(self, state=None, available_actions=None):
        actions = self._env.get_state_actions()
        rand = np.random.randint(len(actions))
        return state, actions[rand]
        
    def feedback_last(self, reward, new_state, prev_state=None, prev_action=None):
        print 'Received reward of %f' % reward
    
    #IGNORE
    def reset_all(self):
        pass
    def reset_episodic(self):
        pass
    def act1(self, state=None, available_actions=None):
        pass
    def act2(self, state=None, available_actions=None):
        pass
    def act3(self, state=None, available_actions=None):
        pass
    def act4(self, state=None, available_actions=None):
        pass
    def feedback1(self, reward, new_state, prev_state=None, prev_action=None):
        pass
    def feedback2(self, reward, new_state, prev_state=None, prev_action=None):
        pass
    def feedback3(self, reward, new_state, prev_state=None, prev_action=None):
        pass
    

if __name__ == '__main__':
    
    env = SimpleEnv()
    learner = SimpleAg('A1', env)
    
    env.run_episode()
