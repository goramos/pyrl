'''
Created on 07/11/2014

@author: Gabriel de O. Ramos <goramos@inf.ufrgs.br>
'''
from learner import Learner
from tools import sampling

class WPL(Learner):
    
    def __init__(self, name, env, starting_state, goal_state, eta=0.002, alpha=0.1, gamma=0.999, epsilon=0.0001):
        
        super(WPL, self).__init__(name, env, self)
        
        self._starting_state = starting_state
        self._goal_state = goal_state
        
        self._eta = eta
        self._alpha = alpha
        self._gamma = gamma
        self._epsilon = epsilon
        
        
        #self._initialise_policy_gradient()
        
        #initialize the memory structures as dictionaries, with one entry per state
        #and one subentry for each action available in state 
        #  * policy: for each state (entry) store a probability vector (<a:p(a)> dictionary) over the actions available on it
        #  * expected payoff: for each state store a <a:E(a)> dictionary
        #in the beginning, only the entries corresponding to the initial state are populated; the other entries are populated on the fly
        self._policy = {}
        self._expected_payoff = {}
        self.__check_and_create_memory_entry(self._starting_state)
        
        
        self.reset_episodic()
    
    #check whether the given state is already in the memory, if not, create it
    #PS: as the memory entries are created on-the-fly, some states may not be in the memory yet
    def __check_and_create_memory_entry(self, state):
        try:
            #self._QTable[state].keys()
            
            self._policy[state].keys()
            
        except:
            #self._QTable[state] = dict({a:0 for a in self._env.get_state_actions(state)})
            
            ini_actions = self._env.get_state_actions(state)
            
            self._policy[state] = dict({a:(1.0/len(ini_actions)) for a in ini_actions}) #initially, the actions are equipossible
            
            self._expected_payoff[state] = dict({a:0 for a in ini_actions}) #initially, the expected payoff is zero
    
#     #initialize the Q-table.
#     #in the beginning, only the entries corresponding to initial state
#     #are populated. The other entries are populated on the fly.
#     def _initialise_policy_gradient(self):
#         #self._QTable = {}
#         #self._QTable[self._starting_state] = dict({a:0 for a in self._env.get_state_actions(self._starting_state)})
#         
#         ini_actions = self._env.get_state_actions(self._starting_state)
#         
#         self._policy = {}
#         self._policy[self._starting_state] = dict({a:(1/len(ini_actions)) for a in ini_actions}) #initially, the actions are equipossible  
#         
#         self._expected_payoff = {}
#         self._expected_payoff[self._starting_state] = dict({a:0 for a in ini_actions}) #initially, the expected payoff is zero
        
    def reset_all(self):
        #nothing to do here (instead of reset_all, the learner could be recreated)
        pass
    
    def reset_episodic(self):
        self._state = self._starting_state
        self._action = None
        self._accumulated_reward = 0.0#TODO - create getter for this
        self._has_arrived = False
    
    def act1(self, state=None, available_actions=None):
        # not necessary in this algorithm
        pass
    
    def act2(self, state=None, available_actions=None):
        # not necessary in this algorithm
        pass
    
    def act3(self, state=None, available_actions=None):
        # not necessary in this algorithm
        pass
    
    def act4(self, state=None, available_actions=None):
        # not necessary in this algorithm
        pass
    
    def act_last(self, state=None, available_actions=None):
        
        #the state may be passed as parameter if the reasoning is not being made
        #regarding the current state (as is the case in SUMO env, eg)
        if state == None:
            state = self._state 
        else:
            #self.__check_and_create_Q_table_entry(state)
            self.__check_and_create_memory_entry(state)
        
        #if not all actions are available, select the subset and corresponding Q-values
        available = self._policy[state]
        if available_actions != None:
            available = {}
            for a in available_actions:
                available[a] = self._policy[state][a]
        #print state
        #print 'available: %s'%available
        #print 'all: %s'%self._QTable[state]
        
        if not available:
            self._has_arrived = True
        else:
            #sample an action in state s according to probability vector self._policy[s]
            i = sampling.probability_vector(available.values())
            self._action = available.keys()[i]
        
        #print "Action %s taken in state %s" % (self._action, self._state)
        
        #return action to take
        return [state, self._action]
    
    def feedback1(self, reward, new_state, prev_state=None, prev_action=None):
        # not necessary in this algorithm
        pass
    
    def feedback2(self, reward, new_state, prev_state=None, prev_action=None):
        # not necessary in this algorithm
        pass
    
    def feedback3(self, reward, new_state, prev_state=None, prev_action=None):
        # not necessary in this algorithm
        pass
    
    def feedback_last(self, reward, new_state, prev_state=None, prev_action=None):
        
        #print reward
        #print new_state
        #print self._state
        #print self._action
        
        state = prev_state
        if state == None:
            state = self._state
        
        action = prev_action
        if action == None:
            action = self._action
        
        
        #print "After performing action %s in state %s, the new state is %s and the reward %f" % (action, state, new_state, reward)
        
        #check whether new_state is already in Q-table
        self.__check_and_create_memory_entry(state)
        self.__check_and_create_memory_entry(new_state)
        
        #update the expected payoff
        try:
            
            # two functions are available here: 
            # - the first is used in the original WPL paper, and 
            # - the second is the same used by Q-learning 
            
            #alpha * r + (1-alpha) * r #a common RL update function
            #self._expected_payoff[state][action] = self._alpha * reward + (1 - self._alpha) * self._expected_payoff[state][action]
            
            #p(s,a) = p(s,a) + alpha * (r + gamma * max(p(s',a')) - p(s,a)) #same as Q-learning
            max_new_value = 0.0
            try:
                max_new_value = max(self._expected_payoff[new_state].values())
            except:# there are not actions in the new state
                pass 
            self._expected_payoff[state][action] = self._expected_payoff[state][action] + self._alpha * (reward + self._gamma * max_new_value -  self._expected_payoff[state][action])
        except:
            print "*********** Expected payoff not updated! **************************************************"
        
        #calculate the average reward over the possible actions
        avg_reward = sum({v for v in self._expected_payoff[state].values()}) / len(self._expected_payoff[state])
        
        #calculate the gradient direction for each action
        delta = {}
        for a in self._policy[state]:
            delta[a] = self._expected_payoff[state][a] - avg_reward
            if delta[a] > 0:
                delta[a] *= 1 - self._policy[state][a]
            else:
                delta[a] *= self._policy[state][a]
        
        #update the probability vector (projection)
        for a in self._policy[state]:
            self._policy[state][a] = max(self._policy[state][a] + (self._eta * delta[a]), self._epsilon)
        
        #normalize policy to sum up to 1 #TODO - check whether this is correct
        sump = sum(self._policy[state].values())
        for a in self._policy[state]:
            self._policy[state][a] = self._policy[state][a]/sump
            #if sum(self._policy[state].values()) > 1.0:
            #    print "These values sum to %f (must sum to 1): %s"%(sum(self._policy[state].values()), self._policy[state].values())
        
        #update curr_state = new_state
        self._state = new_state
        
        #update the subset of actions that are available on the new state (None if all are available) 
        #self._available_actions = available_actions
        
        #update accumulated reward
        self._accumulated_reward += reward
        
        #check whether an ending state has been reached
        if new_state == self._goal_state or not self._policy[new_state].keys()[0]:
            self._has_arrived = True
        
    def has_arrived(self):
        return self._has_arrived
