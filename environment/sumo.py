'''
Created on 11/08/2014

@author: Gabriel de O. Ramos <goramos@inf.ufrgs.br>
'''
from environment import Environment
import traci
import sumolib
from xml.dom import minidom
import sys, os
import subprocess
import atexit
from contextlib import contextmanager
import time

class SUMO(Environment):
    
    def __init__(self, cfg_file, port=8813, use_gui=False):
        
        super(SUMO, self).__init__()
        
        self.__create_env(cfg_file, port, use_gui)
        
    '''
    Create the environment as a MDP. The MDP is modeled as follows:
    * states represent nodes
    * actions represent the out links in each node
    * the reward of taking an action a (link) in state s (node) and going to a new state s' is the
      time spent on traveling on such a link multiplied by -1 (the lower the travel time the better)
    * the transitions between states are deterministic
    '''
    def __create_env(self, cfg_file, port, use_gui):
        
        #check for SUMO's binaries
        if use_gui:
            self._sumo_binary = sumolib.checkBinary('sumo-gui')
        else:
            self._sumo_binary = sumolib.checkBinary('sumo')
        
        #register SUMO/TraCI parameters
        self.__cfg_file = cfg_file
        self.__net_file = self.__cfg_file[:self.__cfg_file.rfind("/")+1] + minidom.parse(self.__cfg_file).getElementsByTagName('net-file')[0].attributes['value'].value
        self.__rou_file = self.__cfg_file[:self.__cfg_file.rfind("/")+1] + minidom.parse(self.__cfg_file).getElementsByTagName('route-files')[0].attributes['value'].value
        self.__port = port
        
        #..............................
        
        #read the network file
        self.__net = sumolib.net.readNet(self.__net_file)
        
        #create MDP as a dictionary, where:
        #  * keys represent the nodes' IDs
        #  * the value of each key is another dictionary, where:
        #    - keys are out-links' IDs
        #    - values are the other end of the links (resulting states)
        self.__env = {}
        for s in self.__net.getNodes(): #current states (current nodes)
            d = {}
            for a in s.getOutgoing(): #actions (out links)
                d[a.getID().encode('utf-8')] = a.getToNode().getID().encode('utf-8') #resulting states (arriving nodes)
            self.__env[s.getID().encode('utf-8')] = d
        
#         self.__env = {}
#         for s in self.__net.getNodes(): #current states (current nodes)
#             for si in s.getIncoming():
#                 state = '%s:::%s'%(s.getID().encode('utf-8'), si.getID().encode('utf-8'))
#                 d = {}
#                 for a in si.getOutgoing(): #actions (out links)
#                     res_state = '%s:::%s'%(a.getToNode().getID().encode('utf-8'), a.getID().encode('utf-8'))
#                     d[a.getID().encode('utf-8')] = res_state #resulting states (arriving nodes)
#                 self.__env[state] = d
#             d = {}
#             for a in s.getOutgoing(): #actions (out links)
#                 d[a.getID().encode('utf-8')] = a.getToNode().getID().encode('utf-8') #resulting states (arriving nodes)
#             self.__env[s.getID().encode('utf-8')] = d
        #print states and actions
#         for s in self.__env.keys():
#             print s
#             for a in self.__env[s]:
#                 print a#print '\t%s goes from %s to %s' % (a, self.__get_action(a).getFromNode().getID(), self.__get_action(a).getToNode().getID())
        
        #create the set of vehicles
        self.__create_vehicles()
        
    def __create_vehicles(self):
        
        # set of all vehicles in the simulation
        # each element in __vehicles correspond to another in __learners
        # the SUMO's vehicles themselves are not stored, since the simulation 
        # is recreated on each episode
        self.__vehicles = {}
        
        #process all route entries
        R = {}
        routes_parse = minidom.parse(self.__rou_file).getElementsByTagName('route')
        for route in routes_parse:
            if route.hasAttribute('id'):
                R[route.getAttribute('id').encode('utf-8')] = route.getAttribute('edges').encode('utf-8')
        
        # process all vehicle entries
        vehicles_parse = minidom.parse(self.__rou_file).getElementsByTagName('vehicle')
        for v in vehicles_parse:
            
            #vehicle's ID
            vehID = v.getAttribute('id').encode('utf-8')
            
            # process the vehicle's route
            route = ''
            if v.hasAttribute('route'): # list of edges or route ID
                route = v.getAttribute('route').encode('utf-8')
                if route in R: # route ID
                    route = R[route]
            else: # child route tag
                route = v.getElementsByTagName('route')[0].getAttribute('edges').encode('utf-8')
            
            # origin and destination nodes
            origin = self.__get_edge_origin(route.split(' ')[0])
            destination = self.__get_edge_destination(route.split(' ')[-1])
            
            #depart
            depart = v.getAttribute('depart').encode('utf-8')
            
            #vType 
            vType = v.getAttribute('vType').encode('utf-8')
            
            # create the entry in the dictionary 
            self.__vehicles[vehID] = {
                'origin': origin,
                'destination': destination,
                'current_link': None,
                'previous_node': origin,
                
                'next_chosen': False,
                
                'desired_departure_time': int(depart), #desired departure time
                'departure_time': -1.0, #real departure time (as soon as the vehicle is no more waiting)
                'arrival_time': -1.0,
                'travel_time': -1.0,
                'time_last_link': -1.0,
                
                'route': [origin],
            
                'vType': vType
            }
    
    def get_vehicles_ID_list(self):
        #return a list with the vehicles' IDs
        return self.__vehicles.keys()
    
    def get_vehicle_dict(self, vehID):
        # return the value in __vehicles corresponding to vehID 
        return self.__vehicles[vehID]
    
    def __get_edge_origin(self, edge_id):
        # return the FROM node ID of the edge edge_id
        return self.__net.getEdge(edge_id).getFromNode().getID().encode('utf-8')
    
    def __get_edge_destination(self, edge_id):
        # return the TO node ID of the edge edge_id
        return self.__net.getEdge(edge_id).getToNode().getID().encode('utf-8')
    
    #return an Edge instance from its ID
    def __get_action(self, ID):
        return self.__net.getEdge(ID)
    
    #return a Node instance from its ID
    def __get_state(self, ID):
        return self.__net.getNode(ID)
    
    #commands to be performed upon normal termination
    def __close_connection(self):
        traci.close()               #stop TraCI
        sys.stdout.flush()          #clear standard output
        self._sumo_process.wait()   #wait for SUMO's subprocess termination
    
    def get_state_actions(self, state):
        return self.__env[state].keys()
    
    def reset_episode(self):
        
        super(SUMO, self).reset_episode()
        
        #open a SUMO subprocess
        self._sumo_process = subprocess.Popen([self._sumo_binary, "-c", self.__cfg_file, "--remote-port", "%i"%(self.__port)], stdout=open(os.devnull, 'wb'), stderr=open(os.devnull, 'wb'))
        
        #initialise TraCI
        traci.init(self.__port)
        
        #register commands to be performed upon normal termination
        atexit.register(self.__close_connection)
        
        #------------------------------------
        for vehID in self.get_vehicles_ID_list():
            self.__vehicles[vehID]['current_link'] = None
            self.__vehicles[vehID]['previous_node'] = self.__vehicles[vehID]['origin']
            self.__vehicles[vehID]['departure_time'] = -1.0
            self.__vehicles[vehID]['arrival_time'] = -1.0
            self.__vehicles[vehID]['travel_time'] = -1.0
            self.__vehicles[vehID]['time_last_link'] = -1.0
            self.__vehicles[vehID]['route'] = [self.__vehicles[vehID]['origin']]
    
    @contextmanager
    def redirected(self):
        saved_stdout = sys.stdout
        sys.stdout = open(os.devnull, 'wb')
        yield
        sys.stdout = saved_stdout
    
    def run_episode(self, max_steps=-1):
        
        start = time.time()
        
        max_steps *= 1000 #traci returns steps in ms, not s
        self._has_episode_ended = False
        self._episodes += 1
        self.reset_episode()
        #print self.__vehicles['1.0']
        
        
        #----------------------------------------------------------------------------------
        # the initial action must be known in advance in order to create the vehicle's 
        # initial route (vehicles need a route to be created)
        learner_state_action = {}
        for vehID in self.get_vehicles_ID_list():
            self._learners[vehID].act1()
        for vehID in self.get_vehicles_ID_list():
            self._learners[vehID].act2()
        for vehID in self.get_vehicles_ID_list():
            self._learners[vehID].act3()
        for vehID in self.get_vehicles_ID_list():
            self._learners[vehID].act4()
        for vehID in self.get_vehicles_ID_list():
            # let the learner choose the first action
            state, action = self._learners[vehID].act_last()
            learner_state_action[vehID] = [state, action]
         
        #for vehID in learner_state_action.keys():
        #    self._learners[vehID].feedback1(0.0, learner_state_action[vehID][0])
        #for vehID in learner_state_action.keys():
        #    self._learners[vehID].feedback2(0.0, learner_state_action[vehID][0])
        #for vehID in learner_state_action.keys():
        #    self._learners[vehID].feedback3(0.0, learner_state_action[vehID][0])
        for vehID in learner_state_action.keys():
            # provide the initial feedback (in fact it makes no effect in the Q-table, but and in the other cases?)#TODO - provide or not?
            #self._learners[vehID].feedback_last(0.0, learner_state_action[vehID][0])
            
            # create an initial route for vehicle vehID, consisting of the first action, only
            traci.route.add('R-%s'%vehID, [learner_state_action[vehID][1]])
            with (self.redirected()):
                try:
                    traci.vehicle.setRouteID(vehID, 'R-%s'%vehID)
                    #print vehID , ': ' , 'R-%s'%vehID , ' = ' , traci.route.getEdges('R-%s'%vehID )
                except:
                    pass
        #----------------------------------------------------------------------------------
        
        #print "==========================================================="
        
        #main loop
        arrived=0
        departed=0
        while ((max_steps > -1 and traci.simulation.getCurrentTime() < max_steps) or max_steps <= -1) and (traci.simulation.getMinExpectedNumber() > 0 or traci.simulation.getArrivedNumber() > 0):
            
            #if (traci.simulation.getCurrentTime()/1000) % 100 == 0:
            #    print traci.simulation.getCurrentTime()/1000
            
            # loaded vehicles
            # the initial route must be set as soon as the vehicle is loaded, and BEFORE it enters the network
            for vehID in traci.simulation.getLoadedIDList():
                #rrraaab = traci.vehicle.getRoute(vehID)
                traci.vehicle.setRouteID(vehID, 'R-%s'%vehID)
                #if vehID == vtest:
                #    print "Route of %s was %s now is %s" % (vehID, rrraaab, traci.vehicle.getRoute(vehID))
            
            # run a single simulation step 
            traci.simulationStep()
            current_time = traci.simulation.getCurrentTime()/1000
            #if current_time % 500 == 0:
            #    print '> %i, %i (%i departed, %i arrived) ' % (self._episodes, current_time, departed, arrived)
            
            # departed vehicles (those that have are entering the network)
            for vehID in traci.simulation.getDepartedIDList():
                self.__vehicles[vehID]["departure_time"] = current_time
                departed += 1
            
            # arrived vehicles (those that have reached their destinations)
            vehicles_to_process_feedback = {}
            for vehID in traci.simulation.getArrivedIDList():
                arrived += 1
                
                self.__vehicles[vehID]["arrival_time"] = current_time
                self.__vehicles[vehID]["travel_time"] = self.__vehicles[vehID]["arrival_time"] - self.__vehicles[vehID]["departure_time"]
                
                reward = current_time - self.__vehicles[vehID]['time_last_link']
                reward *= -1
                
                vehicles_to_process_feedback[vehID] = [
                    reward, 
                    self.__get_edge_destination(self.__vehicles[vehID]['current_link']),
                    self.__get_edge_origin(self.__vehicles[vehID]['current_link']),
                    self.__vehicles[vehID]['current_link']
                ]
            self.__process_vehicles_feedback(vehicles_to_process_feedback, current_time)
            
            #=======================================================================
            vehicles_to_process_feedback = {}
            vehicles_to_process_act = {}
            
            for vehID in self.get_vehicles_ID_list(): # all vehicles
                if self.__vehicles[vehID]["departure_time"] != -1.0 and self.__vehicles[vehID]["arrival_time"] == -1.0: # who have departed but not yet arrived
                    road = traci.vehicle.getRoadID(vehID)
                    #print '%s: is in %s (%s in table), which is %s' % (vehID, road, self.__vehicles[vehID]["current_link"], self.__is_link(road))
                    if road != self.__vehicles[vehID]["current_link"] and self.__is_link(road): #but have just leaved a node
                        #update info of previous link
                        if self.__vehicles[vehID]['time_last_link'] > -1.0:
                            
                            #print 'I spent %ims at link %s' % (current_time-D[d]['time_last_link'], D[d]["current_link"])
                            reward = current_time - self.__vehicles[vehID]['time_last_link']
                            reward *= -1
                            
                            vehicles_to_process_feedback[vehID] = [
                                reward,
                                self.__get_edge_destination(self.__vehicles[vehID]['current_link']),
                                self.__get_edge_origin(self.__vehicles[vehID]['current_link']),
                                self.__vehicles[vehID]['current_link']
                            ]
                        
                        self.__vehicles[vehID]['time_last_link'] = current_time
                        
                        #update current_link
                        self.__vehicles[vehID]['current_link'] = road
                        
                        #get the next node, and add it to the route
                        node = self.__get_edge_destination(self.__vehicles[vehID]["current_link"])
                        self.__vehicles[vehID]['route'].append(self.__get_edge_destination(self.__vehicles[vehID]['current_link']))
                        
                        if node != self.__vehicles[vehID]['destination']:
                            
                            vehicles_to_process_act[vehID] = [
                                node, #next state
                                [x.getID().encode('utf-8') for x in self.__net.getEdge(self.__vehicles[vehID]['current_link']).getOutgoing()] #available actions
                            ]
            
            self.__process_vehicles_feedback(vehicles_to_process_feedback, current_time)
            self.__process_vehicles_act(vehicles_to_process_act, current_time)
                            
            #=======================================================================
        
        #for v in self.__vehicles.keys():
        #    print '%s: %s' % (v, self.__vehicles[v])
        sum_tt = 0
        for vehID in self.get_vehicles_ID_list():
            if self.__vehicles[vehID]['travel_time'] > -1.0:
                sum_tt += self.__vehicles[vehID]['travel_time']
            else: # for those vehicles that have not reached their destination
                sum_tt += current_time - self.__vehicles[vehID]["departure_time"]
            #print '%i\t%s\t%s\t%s' % (self._episodes, vehID, self.__vehicles[vehID]['travel_time'], self.__vehicles[vehID]['route'])
        print '%i\t%s\t%s' % (self._episodes, (sum_tt/len(self.get_vehicles_ID_list()))/60, time.time() - start)
        
        self.__close_connection()
        
        self._has_episode_ended = True
    
    def __process_vehicles_feedback(self, vehicles, current_time):
        
        # feedback1
        for vehID in vehicles.keys():
            self._learners[vehID].feedback1(vehicles[vehID][0], vehicles[vehID][1], vehicles[vehID][2], vehicles[vehID][3])
        
        # feedback2
        for vehID in vehicles.keys():
            self._learners[vehID].feedback2(vehicles[vehID][0], vehicles[vehID][1], vehicles[vehID][2], vehicles[vehID][3])
        
        # feedback3
        for vehID in vehicles.keys():
            self._learners[vehID].feedback3(vehicles[vehID][0], vehicles[vehID][1], vehicles[vehID][2], vehicles[vehID][3])
        
        # feedback_last
        for vehID in vehicles.keys():
            self._learners[vehID].feedback_last(vehicles[vehID][0], vehicles[vehID][1], vehicles[vehID][2], vehicles[vehID][3])
            
    def __process_vehicles_act(self, vehicles, current_time):
        
        # act1
        for vehID in vehicles.keys():
            self._learners[vehID].act1(vehicles[vehID][0], vehicles[vehID][1])
        
        # act2
        for vehID in vehicles.keys():
            self._learners[vehID].act2(vehicles[vehID][0], vehicles[vehID][1])
        
        # act3
        for vehID in vehicles.keys():
            self._learners[vehID].act3(vehicles[vehID][0], vehicles[vehID][1])
        
        # act4
        for vehID in vehicles.keys():
            self._learners[vehID].act4(vehicles[vehID][0], vehicles[vehID][1])
            
        # act_last
        for vehID in vehicles.keys():
            _, action = self._learners[vehID].act_last(vehicles[vehID][0], vehicles[vehID][1])
            #print "%s is in state %s and chosen action %s among %s" % (vehID, vehicles[vehID][0], action, vehicles[vehID][1])
            
            if not vehicles[vehID][1]:
                traci.vehicle.remove(vehID, traci.constants.REMOVE_ARRIVED)
                self.__vehicles[vehID]["arrival_time"] = current_time
                self.__vehicles[vehID]["travel_time"] = self.__vehicles[vehID]["arrival_time"] - self.__vehicles[vehID]["departure_time"]
                continue
            
            #update route
            cur_route = traci.vehicle.getRoute(vehID)
            cur_route.append(action)
            #print 'route+ = %s'%cur_route
            if cur_route[0] != self.__vehicles[vehID]['current_link']:
                del cur_route[0]
            #print 'route- = %s'%cur_route
            traci.vehicle.setRoute(vehID, cur_route)
    
    def __is_link(self, edge_id):
        try:
            _ = self.__net.getEdge(edge_id)
            return True
        except:
            return False
    
    def run_step(self):
        raise Exception('run_step is not available in %s class' % self)
        return
    
    def has_episode_ended(self):
        return self._has_episode_ended
    
    def __calc_reward(self, state, action, new_state):
        raise Exception('__calc_reward is not available in %s class' % self)
        return
            
class SUMORouteChoice(Environment):
    
    def __init__(self, cfg_file, port=8813, use_gui=False):
        
        super(SUMORouteChoice, self).__init__()
        
        self.__create_env(cfg_file, port, use_gui)
        
    '''
    Create the environment as a MDP. The MDP is modeled as follows:
    * each OD-pair is a state (from vehicles point of view, it the MDP is stateless)
    * actions are represented by routes from an origin to a destination (multiple actions are available within each state)
    * the reward of taking an action a (route) is the travel time multiplied by -1 (the lower the travel time the better)
    * the transitions between states are deterministic
    '''
    def __create_env(self, cfg_file, port, use_gui):
        
        #check for SUMO's binaries
        if use_gui:
            self._sumo_binary = sumolib.checkBinary('sumo-gui')
        else:
            self._sumo_binary = sumolib.checkBinary('sumo')
        
        #register SUMO/TraCI parameters
        self.__cfg_file = cfg_file
        self.__net_file = self.__cfg_file[:self.__cfg_file.rfind("/")+1] + minidom.parse(self.__cfg_file).getElementsByTagName('net-file')[0].attributes['value'].value
        self.__rou_file = self.__cfg_file[:self.__cfg_file.rfind("/")+1] + minidom.parse(self.__cfg_file).getElementsByTagName('route-files')[0].attributes['value'].value
        self.__port = port
        
        #read the network file
        self.__net = sumolib.net.readNet(self.__net_file)
        
        # create MDP as a dictionary, where:
        #   * keys represent the nodes' IDs (current state)
        #   * the value of each key is another dictionary, where:
        #     - keys are the other end of the links (resulting states)
        #     - values are arrays of string, which represent the possible routes
        # from the <s, s', a> triples, s' and a are set through method set_routes_OD_pair(...)
        self.__env = {}
        
        #create the set of vehicles and the OD-matrix
        self.__create_vehicles()
        
    # concatenate the origin and destination strings in the form 'O###D'
    # where O and D are the origin and destination, respectively
    def encode_OD(self, origin, destination):
        return '%s###%s' % (origin, destination)
    
    # decode the string into an origin and a destination, as opposed
    # to the concatenation made in encode_OD(...) method
    def decode_OD(self, string):
        sp = string.split('###')
        return sp[0], sp[1]
    
    # define the set of routes of each OD-pair
    def set_routes_OD_pair(self, origin, destination, routes):
        # PS: the definition of the __env variable is available in __create_env(...) method
        
        # the dictionary key is encoded by encode_OD(...) method
        key = self.encode_OD(origin, destination)
        
        # create the entry if it does not yet exist 
        if key not in self.__env:
            self.__env[key] = [origin, destination, []]
        
        # for each route an entry <s', a> is created (s'=resulting state, a=routes)
        for r in routes:
            self.__env[key][2].append(r)
        
    # create the set of vehicles and the OD-matrix
    def __create_vehicles(self):
        
        # set of all vehicles in the simulation
        # each element in __vehicles correspond to another in __learners
        # the SUMO's vehicles themselves are not stored, since the simulation 
        # is recreated on each episode
        self.__vehicles = {}
        
        # the OD-matrix as a dictionary
        # each element key is a OD-pair in the form 'O###D' (see encode_OD(...) method)
        # each value is a triple <origin, destination, vehicles>, where 
        # the latter correspond to the number of vehicles in that OD-pair
        self.__OD_matrix = {}
        
        #process all route entries
        R = {}
        routes_parse = minidom.parse(self.__rou_file).getElementsByTagName('route')
        for route in routes_parse:
            if route.hasAttribute('id'):
                R[route.getAttribute('id').encode('utf-8')] = route.getAttribute('edges').encode('utf-8')
        
        # process all vehicle entries
        vehicles_parse = minidom.parse(self.__rou_file).getElementsByTagName('vehicle')
        for v in vehicles_parse:
            
            #vehicle's ID
            vehID = v.getAttribute('id').encode('utf-8')
            
            # process the vehicle's route
            route = ''
            if v.hasAttribute('route'): # list of edges or route ID
                route = v.getAttribute('route').encode('utf-8')
                if route in R: # route ID
                    route = R[route]
            else: # child route tag
                route = v.getElementsByTagName('route')[0].getAttribute('edges').encode('utf-8')
            
            # origin and destination nodes
            origin = self.__get_edge_origin(route.split(' ')[0])
            destination = self.__get_edge_destination(route.split(' ')[-1])
            
            # create the entry in the dictionary 
            self.__vehicles[vehID] = {
                'origin': origin,
                'destination': destination,
                
                'departure_time': -1.0, # real departure time (as soon as the vehicle is no more waiting)
                'arrival_time': -1.0,
                'travel_time': -1.0
            }
            
            # update/populate __OD_matrix
            OD = self.encode_OD(origin, destination)
            num = 1
            if OD in self.__OD_matrix:
                num += self.__OD_matrix[OD][2]
            self.__OD_matrix[OD] = [origin, destination, num]
    
    # return the set of OD-pairs as an array of arrays, where
    # each position in the main array corresponds to an OD-pair
    # and each nested array has two elements, one for origin
    # and another for destination
    def get_OD_pairs(self):
        ret = []
        for origin, destination, _ in self.__OD_matrix.values():
            ret.append([origin, destination])
        return ret
    
    def get_vehicles_ID_list(self):
        # return a list with the vehicles' IDs
        return self.__vehicles.keys()
    
    def get_vehicle_dict(self, vehID):
        # return the value in __vehicles corresponding to vehID 
        return self.__vehicles[vehID]
    
    def __get_edge_origin(self, edge_id):
        # return the FROM node ID of the edge edge_id
        return self.__net.getEdge(edge_id).getFromNode().getID().encode('utf-8')
    
    def __get_edge_destination(self, edge_id):
        # return the TO node ID of the edge edge_id
        return self.__net.getEdge(edge_id).getToNode().getID().encode('utf-8')
    
    # return an Edge instance from its ID
    def __get_action(self, ID):
        return self.__net.getEdge(ID)
    
    # return a Node instance from its ID
    def __get_state(self, ID):
        return self.__net.getNode(ID)
    
    # commands to be performed upon normal termination
    def __close_connection(self):
        traci.close()               # stop TraCI
        sys.stdout.flush()          # clear standard output
        self._sumo_process.wait()   # wait for SUMO's subprocess termination
    
    def get_state_actions(self, state):
        self.__check_env()
        return self.__env[state][2]
    
    def reset_episode(self):
        
        super(SUMORouteChoice, self).reset_episode()
        
        # open a SUMO subprocess
        self._sumo_process = subprocess.Popen([self._sumo_binary, "-c", self.__cfg_file, "--remote-port", "%i"%(self.__port)], stdout=open(os.devnull, 'wb'), stderr=open(os.devnull, 'wb'))
        
        # initialise TraCI
        traci.init(self.__port)
        
        # register commands to be performed upon normal termination
        atexit.register(self.__close_connection)
        
        # reset vehicles attributes
        for vehID in self.get_vehicles_ID_list():
            self.__vehicles[vehID]['departure_time'] = -1.0
            self.__vehicles[vehID]['arrival_time'] = -1.0
            self.__vehicles[vehID]['travel_time'] = -1.0
    
    @contextmanager
    def redirected(self):
        saved_stdout = sys.stdout
        sys.stdout = open(os.devnull, 'wb')
        yield
        sys.stdout = saved_stdout
    
    # check whether the environment is ready to run
    def __check_env(self):
        # check whether the environment data structure was defined 
        if not self.__env:
            raise Exception("The routes must be set before running! Use set_routes_OD_pair(...) method for this.")
        
    def run_episode(self, max_steps=-1):
        
        self.__check_env()
        
        start = time.time()
        
        max_steps *= 1000 # traci returns steps in ms, not s
        self._has_episode_ended = False
        self._episodes += 1
        self.reset_episode()
        
        #----------------------------------------------------------------------------------
        # the initial action must be known in advance in order to create the vehicle's 
        # initial route (vehicles need a route to be created)
        learner_state_action = {}
        for vehID in self.get_vehicles_ID_list():
            self._learners[vehID].act1()
        for vehID in self.get_vehicles_ID_list():
            self._learners[vehID].act2()
        for vehID in self.get_vehicles_ID_list():
            self._learners[vehID].act3()
        for vehID in self.get_vehicles_ID_list():
            self._learners[vehID].act4()
        for vehID in self.get_vehicles_ID_list():
            # let the learner choose the first action
            state, action = self._learners[vehID].act_last()
            learner_state_action[vehID] = [state, action]
         
        #for vehID in learner_state_action.keys():
        #    self._learners[vehID].feedback1(0.0, learner_state_action[vehID][0])
        #for vehID in learner_state_action.keys():
        #    self._learners[vehID].feedback2(0.0, learner_state_action[vehID][0])
        #for vehID in learner_state_action.keys():
        #    self._learners[vehID].feedback3(0.0, learner_state_action[vehID][0])
        for vehID in learner_state_action.keys():
            # provide the initial feedback (in fact it makes no effect in the Q-table, but and in the other cases?)#TODO - provide or not?
            #self._learners[vehID].feedback_last(0.0, learner_state_action[vehID][0])
              
            # create an initial route for vehicle vehID, consisting of the first action, only
            traci.route.add('R-%s'%vehID, learner_state_action[vehID][1].split(' '))
            with (self.redirected()):
                try:
                    traci.vehicle.setRouteID(vehID, 'R-%s'%vehID)
                except:
                    pass
        #----------------------------------------------------------------------------------
        
        # main loop
        while ((max_steps > -1 and traci.simulation.getCurrentTime() < max_steps) or max_steps <= -1) and (traci.simulation.getMinExpectedNumber() > 0 or traci.simulation.getArrivedNumber() > 0):
            
            # loaded vehicles
            # the initial route must be set as soon as the vehicle is loaded, and BEFORE it enters the network
            for vehID in traci.simulation.getLoadedIDList():
                traci.vehicle.setRouteID(vehID, 'R-%s'%vehID)
            
            # run a single simulation step 
            traci.simulationStep()
            current_time = traci.simulation.getCurrentTime()/1000
            
            # departed vehicles (those that have are entering the network)
            for vehID in traci.simulation.getDepartedIDList():
                self.__vehicles[vehID]["departure_time"] = current_time
            
            # arrived vehicles (those that have reached their destinations)
            vehicles_to_process_feedback = {}
            for vehID in traci.simulation.getArrivedIDList():
                self.__vehicles[vehID]["arrival_time"] = current_time
                self.__vehicles[vehID]["travel_time"] = self.__vehicles[vehID]["arrival_time"] - self.__vehicles[vehID]["departure_time"]
                
                reward = self.__vehicles[vehID]["travel_time"] * -1
                
                vehicles_to_process_feedback[vehID] = [
                    reward, 
                    self.encode_OD(self.__vehicles[vehID]['origin'], self.__vehicles[vehID]['destination'])
                ]
            self.__process_vehicles_feedback(vehicles_to_process_feedback, current_time)
            
        # calculate and print average travel time 
        sum_tt = 0
        for vehID in self.get_vehicles_ID_list():
            sum_tt += self.__vehicles[vehID]['travel_time']
        print '%i\t%s\t%s' % (self._episodes, (sum_tt/len(self.get_vehicles_ID_list()))/60, time.time() - start)
        
        self.__close_connection()
        
        self._has_episode_ended = True
        
    def __process_vehicles_feedback(self, vehicles, current_time):
        
        # feedback1
        for vehID in vehicles.keys():
            self._learners[vehID].feedback1(vehicles[vehID][0], vehicles[vehID][1])
        
        # feedback2
        for vehID in vehicles.keys():
            self._learners[vehID].feedback2(vehicles[vehID][0], vehicles[vehID][1])
        
        # feedback3
        for vehID in vehicles.keys():
            self._learners[vehID].feedback3(vehicles[vehID][0], vehicles[vehID][1])
        
        # feedback_last
        for vehID in vehicles.keys():
            self._learners[vehID].feedback_last(vehicles[vehID][0], vehicles[vehID][1])
    
    def __is_link(self, edge_id):
        try:
            _ = self.__net.getEdge(edge_id)
            return True
        except:
            return False
    
    def run_step(self):
        raise Exception('run_step is not available in %s class' % self)
        return
    
    def has_episode_ended(self):
        return self._has_episode_ended
    
    def __calc_reward(self, state, action, new_state):
        raise Exception('__calc_reward is not available in %s class' % self)
        return

