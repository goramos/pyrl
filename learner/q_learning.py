'''
Created on 04/08/2014

@author: Gabriel de O. Ramos <goramos@inf.ufrgs.br>
'''
from learner import Learner

class QLearner(Learner):
    
    def __init__(self, name, env, starting_state, goal_state, alpha, gamma, exp_strategy):
        
        super(QLearner, self).__init__(name, env, self)
        
        self._starting_state = starting_state
        self._goal_state = goal_state
        
        self._exp_strategy = exp_strategy
        
        self._alpha = alpha
        self._gamma = gamma
        
        self._initialise_Q_table()
        
        self.reset_episodic()
    
    #initialize the Q-table.
    #in the beginning, only the entries corresponding to initial state
    #are populated. The other entries are populated on the fly.
    def _initialise_Q_table(self):#TODO - replace by __check_and_create_Q_table_entry
        self._QTable = {}
        
        self._QTable[self._starting_state] = dict({a:0 for a in self._env.get_state_actions(self._starting_state)})
        
    def reset_all(self):
        #nothing to do here (instead of reset_all, the learner could be recreated)
        pass
    
    def reset_episodic(self):
        self._state = self._starting_state
        self._action = None
        self._accumulated_reward = 0.0
        
        self._exp_strategy.reset_episodic()
        
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
            self.__check_and_create_Q_table_entry(state)
        
        #if not all actions are available, select the subset and corresponding Q-values
        available = self._QTable[state]
        if available_actions != None:#TODO
            available = {}
            for a in available_actions:
                available[a] = self._QTable[state][a]
        #print state
        #print 'available: %s'%available
        #print 'all: %s'%self._QTable[state]
        
        if not available:
            self._has_arrived = True
        else:
            #choose action according to the the exploration strategy
            self._action = self._exp_strategy.choose(available)
        
        #print "Action %s taken in state %s" % (self._action, self._state)
        
        #return action to take
        return [state, self._action]
    
    #check whether the given state is already in the Q-table, if not, create it
    #PS: as the Q-table is created on-the-fly, some states may not be in the table yet
    def __check_and_create_Q_table_entry(self, state):
        try:
            self._QTable[state].keys()
        except:
            self._QTable[state] = dict({a:0 for a in self._env.get_state_actions(state)})
    
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
        self.__check_and_create_Q_table_entry(state)
        self.__check_and_create_Q_table_entry(new_state)
        
        #update Q table with cur_state and action
        try:
            maxfuture = 0.0
            if self._QTable[new_state]: #dictionary not empty
                maxfuture = max(self._QTable[new_state].values())
            
            self._QTable[state][action] += self._alpha * (reward + self._gamma * maxfuture - self._QTable[state][action])
        except Exception, e:
            print "Nooooooooooooooooooooooooo!!!!"
            #print new_state
            #print self._env.get_state_actions(new_state)
            #print self._QTable
            #print str(e)
        
        #update curr_state = new_state
        self._state = new_state
        
        #update the subset of actions that are available on the new state (None if all are available) 
        #self._available_actions = available_actions
        
        #update accumulated reward
        self._accumulated_reward += reward
        
        #check whether an ending state has been reached
        if new_state == self._goal_state or not self._QTable[new_state].keys()[0]:
            self._has_arrived = True
        
    def has_arrived(self):
        return self._has_arrived

class QLearner2(Learner):
    
    def __init__(self, name, starting_state, goal_state, alpha, gamma, exp_strategy, get_actions_f):
        self._name = name
        
        self._starting_state = starting_state
        self._goal_state = goal_state
        self._state = None
        self._action = None
        self._accumulated_reward = 0.0
        
        self._exp_strategy = exp_strategy
        
        #function that return the available actions in a given state
        self._get_actions_f = get_actions_f
        
        self._alpha = alpha
        self._gamma = gamma
        
        self._initialise_Q_table()
        
        self.reset_episodic()
    
    #initialize the Q-table.
    #in the beginning, only the entries corresponding to initial state
    #are populated. The other entries are populated on the fly.
    def _initialise_Q_table(self):
        self._QTable = {}
        
        self._QTable[self._starting_state] = dict({a:0 for a in self._get_actions_f(self._starting_state)})
        
    def reset_episodic(self):
        self._state = self._starting_state
        self._action = None
        self._accumulated_reward = 0.0
        
        self._exp_strategy.reset_episodic()
        
        self._has_arrived = False
    
    def reset_all(self):
        #nothing to do here (instead of reset_all, the learner could be recreated)
        pass
    
    def act(self):
        
        #choose action according to the the exploration strategy
        self._action = self._exp_strategy.choose(self._QTable[self._state])
        
        #print "Action %s taken in state %s" % (self._action, self._state)
        
        #return action to take
        return [self._state, self._action]
        
    def feedback(self, new_state, reward):
        
        #print "After performing action %s in state %s, the new state is %s and the reward %f" % (self._action, self._state, new_state, reward)
        
        #check whether the new_state is already in the Q-table. if not,
        #create an entry for the state and corresponding actions. 
        try:
            self._QTable[new_state].keys()
        except:
            self._QTable[new_state] = dict({a:0 for a in self._get_actions_f(new_state)})
        
        #update Q table with cur_state and action
        self._QTable[self._state][self._action] += self._alpha * (reward + self._gamma * max(self._QTable[new_state].values()) - self._QTable[self._state][self._action])
        
        #update curr_state = new_state
        self._state = new_state
        
        #update accumulated reward
        self._accumulated_reward += reward
        
        #check whether an ending state has been reached
        if self._QTable[new_state].keys()[0] == 'null':
            self._has_arrived = True
        
        
    def has_arrived(self):
        return self._has_arrived
