'''
Created on 17/11/2014

@author: Gabriel de O. Ramos <goramos@inf.ufrgs.br>
'''
from learner import Learner

from itertools import chain, combinations
import collections
import copy
from scipy import stats

class OPPORTUNE(Learner):
    
    def __init__(self, name, env, starting_state, goal_state, alpha, gamma, Emax, exp_strategy, N, OCL):
        
        super(OPPORTUNE, self).__init__(name, env, self)
        
        self._starting_state = starting_state
        self._goal_state = goal_state
        
        self._exp_strategy = exp_strategy
        
        self._alpha = alpha
        self._gamma = gamma
        self._Emax = Emax
        
        self._N = list(N) #_N must be and remain ordered
        
        self._OCL = OCL # the communication layer with other learners
        self._OCL.register_learner(self._name, self)
        
        # used in the action negotiation process 
        self._bids = []
        self._replies = {}
        self._action_bidden = None
        self._accepted_bid = None
        
        self.reset_episodic()
        
        self._initialise_Q_table()
        
        #self.__print_QTable()
    
    def __print_QTable(self):
        print "--------------------------"
        for state in self._QTable:
            print state
            for action in self._QTable[state]:
                print "\t%s" % (action)
        print "--------------------------"
    
    #initialize the Q-table.
    #in the beginning, only the entries corresponding to initial state
    #are populated. The other entries are populated on the fly.
    def _initialise_Q_table(self):
        self._QTable = {} # Q-table
        self._RTable = {} # history rewards table
        
        #actions = {}
        #for a in self._env.get_state_actions(self._starting_state):
        #    v = JointAction({self._name : a})
        #    actions[v] = 0.0
        #self._QTable[self._S] = actions #TODO  RTable
        
        self.__initialise_Q_table_state(self._starting_state)
        
    def __initialise_Q_table_state(self, state):
        
        # identify the individual and joint state
        S = state
        if type(S) is not JointState:
            S = JointState({self._name : state})
        else:
            state = S[self._name]
        
        if S not in self._QTable:
        
            # create the set of actions
            A = {}
            AR = {}
            for a in self._env.get_state_actions(state):
                v = JointAction({self._name : a})
                A[v] = 0.0
                AR[v] = []
            
            self._QTable[S] = A
            self._RTable[S] = AR
        
        
    def reset_all(self):
        #nothing to do here (instead of reset_all, the learner could be recreated)
        pass
    
    def reset_episodic(self):
        
        # joint state and action
        self._S = JointState({self._name : self._starting_state})
        self._A = None
        
        self._state = self._starting_state
        self._action = None
        self._accumulated_reward = 0.0
        
        self._exp_strategy.reset_episodic()
        
        self._has_arrived = False
        
        # used to control the transition among the different act phases
        self._state_act = None
        self._available_act = None
        
        # used to control the transition among the different feedback phases
        self._feedback_state = None
        self._feedback_S = None
        self._feedback_action = None
        self._feedback_A = None
        self._feedback_new_state = None
        self._feedback_new_S = None
        
        
        # store the last state action pairs (for the cases where the 
        # reward is not given immediately after the action is chosen) 
        self._last_state_action_pairs = []
    
    def act1(self, state=None, available_actions=None):
        ''' 
        define the individual (real) state
        '''
        #print '_S = ',self._S
        #print '_state = ',self._state
        self._state_act = state
        
        #the state may be passed as parameter if the reasoning is not being made
        #regarding the current state (as is the case in SUMO env, eg)
        if self._state_act == None:
            self._state_act = self._state
        else:
            self.__check_and_create_Q_table_entry(self._state_act)
        
    def act2(self, state=None, available_actions=None):
        '''
        update the joint state, 
        update the set of available joint actions, 
        sample joint action to be played,
        and start the action negotiation process
        '''
        
        # if the state is joint
        if len(self._S) > 1:
            
            # update the perception of the joint state
            for neighbourID in self._S:
                self._S.update_state(neighbourID, self._OCL.get_learner(neighbourID)._state)
            
            #--------------------------------------------------------------
            # find a subset of _S that is in the Q table
            
            bestsubstate = None
            bestsubstatev = float('-inf')
            
            # check for all combinations of substates of _S
            for z in chain.from_iterable(combinations(self._S.get_learners(), r) for r in range(1,len(self._S.get_learners()))):
                substate = JointState({ x : self._S.get_state(x) for x in z })
                
                # if the substate is in Q-table
                if substate in self._QTable:
                    
                    # get the maximum Q-value in the substate
                    maxs = max(v for v in self._QTable[substate].values())
                    
                    # keep the best substate
                    if maxs > bestsubstatev:
                        bestsubstatev = maxs
                        bestsubstate = substate
            
            # if a valid substate has been found, then it replaces _S
            if bestsubstate:
                self._S = bestsubstate
            
            # otherwise, the single state (joint state with the learner itself, only) is selected to replace _S 
            else:
                self._S = JointState({ self._name : self._S.get_state(self._name) })
            
            #--------------------------------------------------------------
        
        
        # initialise the variables of the action negotiation process
        self._bids = []
        self._replies = {}
        self._action_bidden = None
        self._accepted_bid = None
        #print "_S' = ",self._S
        #print "_state_act' = ",self._state_act
        # if not all actions are available, then select the available subset and corresponding Q-values
        # (in the joint actions, the action corresponding to the present learner must be in the 
        # available_actions set)  
        SS = JointState({self._name : self._state_act})
        self.__initialise_Q_table_state(self._state_act)
        self._available_act = self._QTable[SS]
        #print self._name , ' = ' ,[ str(x) for x in self._available_act.keys()]
        if available_actions != None:
            self._available_act = {}
            for a in self._QTable[SS].keys():
                if a.get_action(self._name) in available_actions:
                    self._available_act[a] = self._QTable[SS][a]
        #print self._name , ' = ' ,[ str(x) for x in self._available_act.keys()]
        
        # if none action is available, then it means that the learner has reached its destination
        if not self._available_act:
            self._has_arrived = True
            
        else:
            
            #print "---------------"
            #for i in self._available_act.keys():
            #    print '%s = %f' % (i, self._available_act[i])
            #print "---------------"
            
            
            # choose joint action according to the the exploration strategy 
            # (and update the individual action according to the joint)
            self._A = self._exp_strategy.choose(self._available_act)
            self._action = self._A.get_action(self._name)
            
            # if the chosen action is joint, then it must be proposed to the involved neighbours
            if len(self._A.get_learners()) > 1:
                
                self._action_bidden = self._A
                
                for neighbourID in self._A.get_learners():
                    self._OCL.get_learner(neighbourID)._bids.append([
                        self._name, 
                        self._A.get_action(neighbourID), 
                        self._QTable[self._S][self._A]
                    ])
            else:
                # single action: no need to communicate, but can still receive bids
                # (nothing to do here)
                pass
        
    def act3(self, state=None, available_actions=None):
        
        if self._bids:
            best_bid = None
            for bid in self._bids:
                if not best_bid or (bid[2] > self._QTable[self._S][self._A] and bid[2] > best_bid[2]): #TODO check whether the aim is the greatest or lowest value
                    best_bid = bid
                # in principle, all bids are rejected; if a bid is accepted, 
                # then the reply is changed forward  
                self._OCL.get_learner(bid[0])._replies[self._name] = "NOT"
            
            if best_bid:
                self._OCL.get_learner(best_bid[0])._replies[self._name] = "OK"
                self._accepted_bid = best_bid
        
    def act4(self, state=None, available_actions=None):
        
        # CASE 1: agent has make a bid, and has not accepted another one
        if self._action_bidden and not self._accepted_bid:
            
            if self._action_bidden != self._A:
                raise Exception("This is not supposed to happen!")
            
            # process the cases in which the neighbour has not replied (this scenario 
            # only happens if the neighbour is not performing an act() right now)
            for a in self._action_bidden.get_learners():
                if a not in self._replies:
                    self._replies[a] = 'NOT'
            
            # count the number of neighbours that (i) did not replied or (ii) refused the bid 
            count_rejected = sum([1 for a in self._action_bidden.get_learners() if a not in self._replies or (a in self._replies and self._replies[a] == 'NOT')])
            
            # CASE 1.1: NOT all involved neighbours accepted the bid
            # (negotiation failed)
            if count_rejected > 0:
                self.__process_negotiation_failed()
            
            # CASE 1.2: ALL involved neighbours accepted the bid
            # (negotiation successful)
            else:
                # set the action and joint action of the involved neighbours
                for neighbourID in self._A.get_learners():
                    self._OCL.get_learner(neighbourID)._A = self._A
                    self._OCL.get_learner(neighbourID)._action = self._A.get_action(neighbourID)
                
        # CASE 2: agent has accepted a bid, but his bidder has accepted another one
        # (negotiation failed)
        elif self._accepted_bid and self._OCL.get_learner(self._accepted_bid[0])._accepted_bid:
            # negotiation failed
            self.__process_negotiation_failed()
        
    
    def act_last(self, state=None, available_actions=None):
        
        # store the (joint) state action pair to enable setting the feedback
        # to the correct state, action pair
        self._last_state_action_pairs.append([self._S, self._A])
        
        #print "==========================================================="
        #print "%s is in state %s and chosen action %s among %s (or %s)" % (self._name, self._state_act, self._action, available_actions, [ str(x) for x in self._available_act.keys()])
        
        # return the action to take
        return [self._state_act, self._action]
    
    def __process_negotiation_failed(self):
        
        # the learner has selected a joint action, then it had to be proposed to its neighbours
        # however, as the negotiation process has not succeeded, it needs to compute an individual action
        if self._action_bidden:
            
            # delimit the set of available actions to single ones (only one learner involved)
            #self._available_act = {x for x in self._available_act if len(x.get_learners()) == 1}
            self._available_act = {x:self._available_act[x] for x in self._available_act if len(x.get_learners()) == 1}
            
            # choose joint action according to the the exploration strategy 
            # (and update the individual action according to the joint)
            self._A = self._exp_strategy.choose(self._available_act)
            self._action = self._A.get_action(self._name)
            
        # the action initially chosen by the learner is not joint, 
        # so it does not need to compute a single action
        else:
            # nothing to do here
            pass
    
    #check whether the given state is already in the Q-table, if not, create it
    #PS: as the Q-table is created on-the-fly, some states may not be in the table yet
    def __check_and_create_Q_table_entry(self, state):
        try:
            self._QTable[state].keys()
        except:
            self.__initialise_Q_table_state(state)
    
    def __check_and_create_Q_table_entry_state_action(self, state, action):
        if action not in self._QTable[state]:
            self.__check_and_create_Q_table_entry(state)
            self._QTable[state][action] = 0.0
            self._RTable[state][action] = [] 
    
    def __increase_joint_state_action(self, S, A):
        
        if len(self._RTable[S][A]) > 0 and sum(self._RTable[S][A]) > 0 and stats.variation(self._RTable[S][A]) > self._Emax:
            Saux = S.clone()
            
            if len(Saux.get_learners()) != len(self._N) + 1:
                #while len(Saux.get_lerners()) == len(S.get_lerners()) or len(Saux.get_lerners()) < self._N + 1:
                for neighbourID in self._N:
                    if neighbourID not in Saux.get_learners():
                        Saux.update_state(neighbourID, self._OCL.get_learner(neighbourID)._feedback_state)
                        S = Saux
                        self.__check_and_create_Q_table_entry(S)
                        break
        else:
            Aaux = A.clone()
            
            if len(Aaux.get_learners()) != len(self._N) + 1:
                for neighbourID in self._N:
                    if neighbourID not in Aaux.get_learners():
                        Aaux.update_action(neighbourID, self._OCL.get_learner(neighbourID)._feedback_action)
                        A = Aaux
                        break
        
        self.__check_and_create_Q_table_entry_state_action(S, A)
        
        return [S, A]
    
    def feedback1(self, reward, new_state, prev_state=None, prev_action=None):
        
        self._feedback_new_state = new_state
        self.__check_and_create_Q_table_entry(self._feedback_new_state)
        
        self._feedback_state = prev_state
        if self._feedback_state == None:
            self._feedback_state = self._state
        self.__check_and_create_Q_table_entry(self._feedback_state)
        
        self._feedback_action = prev_action
        if self._feedback_action == None:
            self._feedback_action = self._action
        
        # get the (joint) state action pair corresponding to the received feedback
        self._feedback_S = self._S
        self._feedback_A = self._A
        self.__check_and_create_Q_table_entry_state_action(self._feedback_S, self._feedback_A)
        
        # in the case of delayed rewards, the state action pair must be get from the appropriate list
        if prev_state == None or prev_action == None: # (ideally, if one of these variables is valid, both are)
            self._feedback_S, self._feedback_A = self._last_state_action_pairs[0]
            del self._last_state_action_pairs[0]
        else:
            self._last_state_action_pairs[0] = [] # clear the list (as it is not used) to avoid wasting space
        
        # ideally, the conditions in the two ifs below should be false 
        #if self._feedback_S.get_state(self._name) != self._feedback_state:
        #    print "Error: states do not match! (%s and %s)" % (self._feedback_S.get_state(self._name), self._feedback_state) 
        #if self._feedback_A.get_action(self._name) != self._feedback_action:
        #    print "Error: actions do not match! (%s and %s)" % (self._feedback_A.get_action(self._name), self._feedback_action)
        
        #print "After performing action %s in state %s, the new state is %s and the reward %f" % (action, state, new_state, reward)
        
        # up to here: the joint state and action were identified
        
    def feedback2(self, reward, new_state, prev_state=None, prev_action=None):
        # not necessary in this algorithm
        pass
    
    def feedback3(self, reward, new_state, prev_state=None, prev_action=None):
        # not necessary in this algorithm
        pass
    
    def feedback_last(self, reward, new_state, prev_state=None, prev_action=None):
        
        #print "%s was in state %s, performed action %s, received reward of %f, and now is in state %s" % (self._name, prev_state, prev_action, reward, new_state)
        
        #------------------------------------------------------------------
        # the code below is equivalent to the procedure AvaliaPassoAnterior of the original description of the algorithm
        
        self._feedback_S, self._feedback_A = self.__increase_joint_state_action(self._feedback_S, self._feedback_A)
            
        #------------------------------------------------------------------
        # the code below corresponds to line 19 of the original description of the algorithm
        
        # find the joint state that has self._feedback_state and has maximum Q-value
        max_S = self._feedback_S
        max_S_v = float('-inf')
        for s in self._QTable.keys():
            if s.get_state(self._name) == new_state:
                if self._QTable[s].values():
                    m = max(self._QTable[s].values())
                    if m > max_S_v:
                        max_S = s
                        max_S_v = m
        
        # update the found state
        self._feedback_new_S = max_S.clone()
        for neighbourID in self._feedback_new_S.get_learners():
            self._feedback_new_S.update_state(neighbourID, self._OCL.get_learner(neighbourID)._feedback_new_state)
        
        # check whether new_state is already in Q-table
        self.__check_and_create_Q_table_entry(self._feedback_new_S)
        
        #------------------------------------------------------------------
        # the code below is equivalent to the procedure AtualizaTabelaQ of the original description of the algorithm
        
        # check all subsets of the joint state
        Actions = []
        for z in chain.from_iterable(combinations(self._feedback_A.get_learners(), r) for r in range(1,len(self._feedback_A.get_learners())+1)):
            if self._name in z:
                ac = JointAction({ x : self._feedback_A.get_action(x) for x in z })
                Actions.append(ac)
                
        # the maximum expected future reward of new state 
        max_new_r = 0.0
        if self._QTable[self._feedback_new_S].values():
            max_new_r = max(self._QTable[self._feedback_new_S].values())
        
        # check for all subsets of the joint state
        for z in chain.from_iterable(combinations(self._feedback_S.get_learners(), r) for r in range(1,len(self._feedback_S.get_learners())+1)):
            if self._name in z:
                st = JointState({ x : self._feedback_S.get_state(x) for x in z })
                
                # for each such subset, check for all combinations with the elements within Actions
                if st in self._QTable:
                    for ac in Actions:
                        
                        # if such state action combination exists
                        if ac in self._QTable[st].keys():
                            
                            self._QTable[st][ac] += self._alpha * (reward + self._gamma * max_new_r - self._QTable[st][ac])
                            self._RTable[st][ac].append(reward)
                            
                            
        
        #------------------------------------------------------------------
        
        
        #update curr_state = new_state
        
        self._state = new_state
        self._S = self._feedback_new_S
        
        #print self._state
        #print self._S
        
        
        #update the subset of actions that are available on the new state (None if all are available) 
        #self._available_actions = available_actions
        
        #update accumulated reward
        self._accumulated_reward += reward
        
        #check whether an ending state has been reached
        if new_state == self._goal_state or not self._QTable[self._feedback_new_S].keys()[0]:
            self._has_arrived = True
    
    def has_arrived(self):
        return self._has_arrived

class OPPORTUNECommLayer(object):
    '''
    Provides an interface among the OPPORTUNE learners. 
    The learners register to this class during instantiation time.
    '''
    
    def __init__(self):
        self.__learners = {}
    
    def register_learner(self, name, learner):
        self.__learners[name] = learner
        
    def get_learner(self, name):
        return self.__learners[name]

class JointState(object):
    def __init__(self, in_dict):
        '''
        A joint state is a dictionary, where keys and values represent, 
        respectively, the learners and their respective states. 
        '''
        self._dict = copy.deepcopy(in_dict)
    
    def update_state(self, learner, state):
        self._dict[learner] = state
    
    def get_learners(self):
        return self._dict.keys()
    
    def get_state(self, learner):
        return self._dict[learner]
    
    def __hash__(self):
        return hash(self.__str__())
    
    def __eq__(self, other):
        return self._dict == other._dict
    
    def __len__(self):
        return len(self._dict)
    
    def __str__(self):
        od = collections.OrderedDict(self._dict)
        return str(od)[12:-1]
    
    def clone(self):
        return JointState(self._dict)
    
class JointAction(object):
    def __init__(self, in_dict):
        '''
        A joint action is a dictionary, where keys and values represent, 
        respectively, the learners and their respective actions. 
        '''
        self._dict = copy.deepcopy(in_dict)
    
    def update_action(self, learner, action):
        self._dict[learner] = action
    
    def get_learners(self):
        return self._dict.keys()
    
    def get_action(self, learner):
        return self._dict[learner]
    
    def __hash__(self):
        return hash(self.__str__())
    
    def __eq__(self, other):
        return self._dict == other._dict
    
    def __len__(self):
        return len(self._dict)
    
    def __str__(self):
        od = collections.OrderedDict(self._dict)
        return str(od)[12:-1]
    
    def clone(self):
        return JointAction(self._dict)