'''
Created on 08/08/2014

@author: Gabriel de O. Ramos <goramos@inf.ufrgs.br>
'''
from environment import Environment
import time#@UnusedImport

class CliffWalking(Environment):
    
    def __init__(self):
        
        super(CliffWalking, self).__init__()
        
        self.__create_env()
        
        #print the states and actions
        #for k in self._env.keys():
        #    print 'Actions in %s: %s' % (k, self._env[k])
    
    #create MDP as a dictionary, where:
    #  * each key corresponds to a state
    #  * the value of each key is another dictionary, where:
    #    - keys are actions
    #    - values the resulting states
    def __create_env(self):
        
        #number of columns (x) and rows (y) of the environment
        x_max = 12
        y_max = 4
        
        #definition of starting and goal states
        self._starting_state = '1_1'
        self._goal_state = '%i_1' % x_max
        self._cliff_states = []
        
        #the set of actions is the same in normal states
        ac = ['up', 'down', 'right', 'left']
        
        #the set of actions in the goal and cliff states
        acc = None#['null']
        
        #for each state x_y
        self._env = {}
        for x in xrange(1, x_max + 1):     #column
            for y in xrange(1, y_max + 1): #row
                
                state = '%i_%i' % (x, y)
                
                #available actions
                if y == 1 and x > 1:        #goal (x=x_max) and cliff (2,...,x_max-1) states
                    d = acc#dict.fromkeys(acc)
                    self._cliff_states.append(state)
                else:                       #normal states
                    d = dict.fromkeys(ac)
                    
                    #transition up
                    if y == y_max:
                        d['up'] = state
                    else:
                        d['up'] = '%i_%i' % (x, y+1)
                    
                    #transition down
                    if y == 1:
                        d['down'] = state
                    else:
                        d['down'] = '%i_%i' % (x, y-1)
                    
                    #transition right
                    if x == x_max:
                        d['right'] = state
                    else:
                        d['right'] = '%i_%i' % (x+1, y)
                    
                    #transition left
                    if x == 1:
                        d['left'] = state
                    else:
                        d['left'] = '%i_%i' % (x-1, y)
                
                #save to the dictionary
                self._env[state] = d
    
    def get_state_actions(self, state):
        if self._env[state]:
            return self._env[state].keys()
        else:
            return [None]
    
    def run_episode(self, max_steps=-1):
        
        self._episodes += 1
        self.reset_episode()
        
        #run the episode until the maximum number of steps is 
        #achieved (if specified) or all agents have arrived 
        while ((max_steps > -1 and self._steps < max_steps) or max_steps == -1) and not self.has_episode_ended():
            self.run_step()
        
    def run_step(self):
        
        self._steps += 1
        
        self._has_episode_ended = True#in the following loop, if any agent has not arrived, the value is changed to False 
        
        #run a step, which corresponds to a RL-cycle, i.e., 
        #the learner acts and receives the corresponding feedback
        learner_feedback = {}
        for l in self._learners.values():
            l.act1()
        for l in self._learners.values():
            l.act2()
        for l in self._learners.values():
            l.act3()
        for l in self._learners.values():
            l.act4()
        for l in self._learners.values():
            
            #get the leaner's action
            s, a = l.act_last()
            
            #define the resulting new state
            s_new = self._env[s][a]
            
            #calculate corresponding reward
            r = self.__calc_reward(s, a, s_new)
            
            #print "%s was in state %s and has chosen action %s, then moving to state %s with reward %f" % (l, s, a, s_new, r)
            #time.sleep(0.5)
            
            learner_feedback[l] = [r, s_new]
        
        #provide feedback to the learner
        for l in learner_feedback.keys():
            l.feedback1(r, s_new)
        for l in learner_feedback.keys():
            l.feedback2(r, s_new)
        for l in learner_feedback.keys():
            l.feedback3(r, s_new)
        for l in learner_feedback.keys():
            l.feedback_last(r, s_new)
            
            #if one of the learners have not arrived, change the corresponding flag
            if not l.has_arrived():
                self._has_episode_ended = False
    
    def has_episode_ended(self):
        return self._has_episode_ended
        
    def get_starting_state(self):
        return self._starting_state
    
    def get_goal_state(self):
        return self._goal_state
    
    #calculate the reward corresponding to performing action in state, after getting to new_state
    def __calc_reward(self, state, action, new_state):
        if new_state == self._goal_state:       #goal state
            return 100
        elif new_state in self._cliff_states:   #cliff states
            return -100
        elif state == new_state:                #collision with the wall (just to test, NOT PRESENT IN THE ORGINAL IDEA)
            return -10
        else:                                   #other states
            return -1
