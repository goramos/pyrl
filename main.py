'''
Created on 04/08/2014

@author: Gabriel de O. Ramos <goramos@inf.ufrgs.br>
'''
import sys#@UnusedImport

import environment as Env
import environment.NFG as NFG
from environment.cliffwalking import CliffWalking
from environment.sumo import SUMO
from environment.sumo import SUMORouteChoice
from environment.NFG.twoplayer_twoaction import TwoPlayerTwoAction

from learner.q_learning import QLearner
from learner.wpl import WPL
from learner.opportune import *#@UnusedWildImport

from exploration.epsilon_greedy import EpsilonGreedy
from exploration.boltzmann import Boltzmann

import tools.misc as misc#@UnusedImport
import external.KSP as KSP

from itertools import chain, combinations



def test_cliff():
    
    #a cliff walking environment
    env = CliffWalking()
    
    #an exploration strategy
    #exp = EpsilonGreedy(1, 0.99)
    exp = Boltzmann(0.1)
    
    #a Q-learner
    learner = QLearner("Agent-1", env, env.get_starting_state(), env.get_goal_state(), 0.3, 0.9, exp)
    #learner = WPL("Agent-1", env, env.get_starting_state(), env.get_goal_state())
    #print learner
    
    #number of episodes
    n_episodes = 1000
    
    #for each episode
    for i in xrange(n_episodes):
        #print "===== Episode %i ==========================================" % (i)
        env.run_episode()
        print "%i\t%i\t%s\t%f" % (i+1, env._steps, learner._state, learner._accumulated_reward)
        #print learner._policy[env.get_starting_state()]

def test_SUMO():
    
    #a SUMO environment
    #env = SUMO('nets/simple/simple-traci.sumocfg', 8813, False)
    env = SUMO('nets/OW/OW-traci.sumocfg', 8813, False)
    
    #an exploration strategy
    exp = EpsilonGreedy(1, 0.925)
    
    #for each vehicle in the route file
    for vehID in env.get_vehicles_ID_list():
        vehDic = env.get_vehicle_dict(vehID)
        #a reinforcement learner
        _ = QLearner(vehID, env, vehDic['origin'], vehDic['destination'], 0.3, 0.9, exp) 
        #_ = WPL(vehID, env, vehDic['origin'], vehDic['destination'], 0.002, 0.1)
    
    #number of episodes
    n_episodes = 100
    
    #sys.stdout = open('out.txt', 'w')
    #sys.stderr = open('err.txt', 'w')
    
    print 'ep\tavg tt\truntime'
    
    #for each episode
    for _ in xrange(n_episodes):
        #print "===== Episode %i ==========================================" % (i)
        env.run_episode(50000)
        #print "%i\t%s\t%f" % (env._steps, learner._state, learner._accumulated_reward)

def test_SUMORouteChoice():
    
    # a SUMO environment
    env = SUMORouteChoice('nets/OW/OW-traci.sumocfg', 8813, False)
    
    # convert the SUMO net file to the one accepted by KSP 
    #misc.convert_SUMO_to_KSP('nets/OW/OW-traci.sumocfg')
    
    # create a set of routes for each OD-pair (through KSP algorithm),
    # and define one such set for each OD-pair (these sets will correspond 
    # to the actions available on each state)
    pairs = env.get_OD_pairs()
    for origin, destination in pairs:
        RKSP = KSP.getKRoutes('nets/OW/OW.txt', origin, destination, 4)
        routes = [" ".join(r[0]) for r in RKSP]
        env.set_routes_OD_pair(origin, destination, routes)
    
    # an exploration strategy
    exp = EpsilonGreedy(1, 0.925)
    
    # for each vehicle in the route file
    for vehID in env.get_vehicles_ID_list():
        vehDic = env.get_vehicle_dict(vehID)
        
        # in the SUMORouteChoice environment the origin is an encoding of the OD-pair 
        origin = env.encode_OD(vehDic['origin'], vehDic['destination'])
        
        # create a learner
        _ = QLearner(vehID, env, origin, vehDic['destination'], 0.8, 0.9, exp)
        #_ = WPL(vehID, env, origin, vehDic['destination'], 0.002, 0.1)
        #print '%s (%s,%s) is in %s'%(Q.get_name(), vehDic['origin'], vehDic['destination'], Q.get_state())
    
    # number of episodes
    n_episodes = 100
    
    print 'ep\tavg tt\truntime'
    
    # for each episode
    for _ in xrange(n_episodes):
        env.run_episode(50000)
        #print env._learners['1.0']._QTable

def test_NFG():
    
    env = TwoPlayerTwoAction(NFG.GAME_MATCHING_PENNIES)
    
    _ = WPL("p1", env, Env.ENV_SINGLE_STATE, Env.ENV_SINGLE_STATE, 0.002, 0.1)
    _ = WPL("p2", env, Env.ENV_SINGLE_STATE, Env.ENV_SINGLE_STATE, 0.002, 0.1)
    
    for _ in xrange(10000):
        env.run_episode()
        
def test_OPPORTUNE():
    
    # a SUMO environment
    env = SUMO('nets/OW/OW-traci.sumocfg', 8813, False)
    
    # an exploration strategy
    exp = EpsilonGreedy(1, 0.925)
    
    #----------------------------------------------------------
    #create a list (vehD) of vehicles with the OD-pair of each vehicle (each entry is in the form <O, D, "O###D">), 
    #and also a list (OD_grouping) of vehicles grouped by OD-pair (each entry is the list of vehicles with same OD-pair);
    #the vehicles of the same OD-pair are considered neighbours
    vehD = {}
    OD_grouping = {}
    for vehID in env.get_vehicles_ID_list():
        vehDic = env.get_vehicle_dict(vehID)
        ODpair = '%s###%s' % (vehDic['origin'], vehDic['destination'])
        vehD[vehID] = [ODpair, vehDic['origin'], vehDic['destination']]
        if ODpair not in OD_grouping:
            OD_grouping[ODpair] = []
        OD_grouping[ODpair].append(vehID)
    
    #sort the lists of neighbours
    for k in OD_grouping.keys():
        OD_grouping[k].sort()
    
    # create the communication layer among the learners
    OCL = OPPORTUNECommLayer()
    
    #create the learners
    for vehID in env.get_vehicles_ID_list():
        
        # create the list of neighbours of vehID (in this example, such a 
        # list is comprised by all vehicles from the same OD pair as vehID) 
        Ni = list(OD_grouping[vehD[vehID][0]])
        Ni.remove(vehID)
        
        # create the learner corresponding to vehID
        _ = OPPORTUNE(vehID, env, vehD[vehID][1], vehD[vehID][2], 0.3, 0.9, 0.001, exp, Ni, OCL)
        
#         vehDic = env.get_vehicle_dict(vehID)
#         #a reinforcement learner
#         _ = QLearner(vehID, env, vehDic['origin'], vehDic['destination'], 0.3, 0.9, exp) 
    
    #----------------------------------------------------------
    
    # number of episodes
    n_episodes = 1000
    
    print 'ep\tavg tt\truntime'
    
    # for each episode
    for _ in xrange(n_episodes):
        env.run_episode(50000)
    
    
def test_OPPORTUNE_route_choice():
    
    # a SUMO environment
    env = SUMORouteChoice('nets/OW/OW-traci.sumocfg', 8813, False)
    
    # convert the SUMO net file to the one accepted by KSP 
    #misc.convert_SUMO_to_KSP('nets/OW/OW-traci.sumocfg')
    
    # create a set of routes for each OD-pair (through KSP algorithm),
    # and define one such set for each OD-pair (these sets will correspond 
    # to the actions available on each state)
    pairs = env.get_OD_pairs()
    for origin, destination in pairs:
        RKSP = KSP.getKRoutes('nets/OW/OW.txt', origin, destination, 4)
        routes = [" ".join(r[0]) for r in RKSP]
        env.set_routes_OD_pair(origin, destination, routes)
    
    # an exploration strategy
    exp = EpsilonGreedy(0.05, 0)
    
    #----------------------------------------------------------
    #create a list (vehD) of vehicles with the OD-pair of each vehicle (each entry is in the form <O, D, "O###D">), 
    #and also a list (OD_grouping) of vehicles grouped by OD-pair (each entry is the list of vehicles with same OD-pair);
    #the vehicles of the same OD-pair are considered neighbours
    vehD = {}
    OD_grouping = {}
    for vehID in env.get_vehicles_ID_list():
        vehDic = env.get_vehicle_dict(vehID)
        ODpair = env.encode_OD(vehDic['origin'], vehDic['destination'])
        vehD[vehID] = [ODpair, vehDic['origin'], vehDic['destination']]
        if ODpair not in OD_grouping:
            OD_grouping[ODpair] = []
        OD_grouping[ODpair].append(vehID)
    
    #sort the lists of neighbours
    for k in OD_grouping.keys():
        OD_grouping[k].sort()
    
    # create the communication layer among the learners
    OCL = OPPORTUNECommLayer()
    
    #create the learners
    for vehID in env.get_vehicles_ID_list():
        
        # create the list of neighbours of vehID (in this example, such a 
        # list is comprised by all vehicles from the same OD pair as vehID) 
        Ni = list(OD_grouping[vehD[vehID][0]])
        Ni.remove(vehID)
        
        # in the SUMORouteChoice environment the origin is an encoding of the OD-pair 
        origin = vehD[vehID][0]
        
        # create the learner corresponding to vehID
        _ = OPPORTUNE(vehID, env, origin, vehD[vehID][2], 0.5, 0.9, 0.05, exp, Ni, OCL)
    
    #----------------------------------------------------------
    
    # number of episodes
    n_episodes = 10000
    
    print 'ep\tavg tt\truntime'
    
    # for each episode
    for _ in xrange(n_episodes):
        env.run_episode(50000)
        #print env._learners['1.0']._QTable

def test_combinations_update_QTable():
    
    _name = '1.1'
    
    _S = JointState({'1.1': 'A1###L1', '2.0': 'A1###M1', '2.1': 'A1###M1'})
    _A = JointAction({'1.1': 'A', '2.0': 'B', '2.1': 'B'})
    
    _QTable = {
        JointState({'1.1': 'A1###L1'}): {JointAction({'1.1': 'A'}):1},
        JointState({'1.1': 'A1###L1', '2.0': 'A1###M1'}): {JointAction({'1.1': 'A', '2.0': 'A'}):2, JointAction({'1.1': 'A', '2.0': 'B'}):3},
        JointState({'1.1': 'A1###L1', '2.1': 'A1###M1'}): {JointAction({'1.1': 'A'}):2, JointAction({'1.1': 'A', '2.1': 'B'}):1, JointAction({'1.1': 'A', '2.1': 'C'}):4},
        JointState({'1.1': 'A1###L1', '2.0': 'A1###M1', '2.1': 'A1###M1'}): {JointAction({'1.1': 'A', '2.0': 'B', '2.1': 'B'}): 2, JointAction({'1.1': 'A', '2.0': 'B', '2.1': 'C'}): 7}
    }
    
    Actions = []
    for z in chain.from_iterable(combinations(_A.get_learners(), r) for r in range(1,len(_A.get_learners())+1)):
        if _name in z:
            ac = JointAction({ x : _A.get_action(x) for x in z })
            Actions.append(ac)
    #for a in Actions: print a
    
    for z in chain.from_iterable(combinations(_S.get_learners(), r) for r in range(1,len(_S.get_learners())+1)):
        if _name in z:
            st = JointState({ x : _S.get_state(x) for x in z })
            if st in _QTable:
                for ac in Actions:
                    if ac in _QTable[st].keys():
                        print "Q(%s, %s) = %i" % (st.get_learners(), ac.get_learners(), _QTable[st][ac])

def test_combinations():
    
    _name = '1.1'
    
    _S = JointState({'1.1': 'A1###L1', '2.0': 'A1###M1', '2.1': 'A1###M1'})
    
    _QTable = {
        JointState({'1.1': 'A1###L1'}): {'a':1, 'b':2},
        JointState({'1.1': 'A1###L1', '2.0': 'A1###M1'}): {'a':1, 'b':3},
        JointState({'1.1': 'A1###L1', '2.1': 'A1###M1'}): {'a':1, 'c':4}
    }
    
    bestsubstate = None
    bestsubstatev = float('-inf')
    
    # check for all combinations of substates of _S
    for z in chain.from_iterable(combinations(_S.get_learners(), r) for r in range(2,len(_S.get_learners()))):
        substate = JointState({ x : _S.get_state(x) for x in z })
        
        # if the substate is in Q-table
        if substate in _QTable:
            
            # get the maximum Q-value in the substate
            maxs = max(v for v in _QTable[substate].values())
            
            # keep the best substate
            if maxs > bestsubstatev:
                bestsubstatev = maxs
                bestsubstate = substate
                
    if bestsubstate:
        print "Best substate found is %s, whose maximum expected payoff is %f" % (bestsubstate, bestsubstatev)
    else:
        bestsubstate = JointState({ _name : _S.get_state(_name) })
        print "None of the substates is valid! The single state %s was selected, whose maximum expected payoff is %f" % (bestsubstate, max(v for v in _QTable[bestsubstate].values()))
        

def test_combinations_old():
    _S = {'1.1': 'A1###L1', '2.0': 'A1###M1', '2.1': 'A1###M1'}
    _Qtable = {"{'1.1': 'A1###L1'}": {'A1A AC CG GJ JL LL1': 0, 'A1A AC CG GJ JI IL LL1': 0, 'A1A AC CD DG GJ JI IL LL1': 0, 'A1A AC CF FI IL LL1': 0}}
    
    for z in chain.from_iterable(combinations(_S, r) for r in range(2,len(_S))):
        print z

def test():
    a = {1:'A', 2:'B', 3:'C', 4:'D'}
    print { x:a[x] for x in a if x > 2}

def test_old():
    _A = {'A':1, 'B':2, 'C':2}
    _replies = {'A':'OK', 'B':'OK', 'C':'OK'}
    
    # process the cases in which the neighbour has not replied (this scenario 
    # only happens if the neighbour is not performing and act() right now)
    for a in _A.keys():
        if a not in _replies:
            _replies[a] = 'NOT'  
    
    # count the number of neighbours that (i) did not replied or (ii) refused the bid 
    print sum([1 for a in _A.keys() if a not in _replies or (a in _replies and _replies[a] == 'NOT')])
    
if __name__ == '__main__':
    
    #test_cliff()
    #test_SUMO()
    test_SUMORouteChoice()
    #test_NFG()
    #test_OPPORTUNE()
    #test_OPPORTUNE_route_choice()
    #test_combinations()
    #test_combinations_update_QTable()
    #test()
