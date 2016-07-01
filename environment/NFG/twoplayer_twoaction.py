'''
Created on 07/11/2014

@author: Gabriel de O. Ramos <goramos@inf.ufrgs.br>
'''
from environment import Environment, ENV_SINGLE_STATE

class TwoPlayerTwoAction(Environment):
    
    def __init__(self, game_string):
        
        super(TwoPlayerTwoAction, self).__init__()
        
        self.__create_env(game_string)
        
    def __create_env(self, game_string):
        
        # split the game string
        gs = game_string.split(';')
        
        # split the set of possible actions
        ac = gs[0].split(',')
        
        # create the environment data structure
        self._env = {}
        
        # define the game matrix according to the input game string
        # player 1 plays action ac[0]
        self._env[ac[0]] = {
            ac[0]: map(int, gs[1].split(',')), # player 2 plays action ac[0]
            ac[1]: map(int, gs[2].split(','))  # player 2 plays action ac[1]
        }
        # player 1 plays action ac[1]
        self._env[ac[1]] = {
            ac[0]: map(int, gs[3].split(',')), # player 2 plays action ac[0]
            ac[1]: map(int, gs[4].split(','))  # player 2 plays action ac[1]
        }
        
    def get_state_actions(self, state=None):
        return self._env.keys()
        
    def run_episode(self):
        
        self._episodes += 1
        self.reset_episode()
        
        self.run_step()
        
    def run_step(self):
        
        self._steps += 1
        self._has_episode_ended = True 
        
        # get the players
        p1,p2 = self._learners.values()
        
        # get their actions (states are present but not necessary due to the nature, of NFGs)
        #_,a1 = p1.act_last()
        #_,a2 = p2.act_last()
        a1, a2 = self.__run_acts(p1, p2)
        
        # calculate the reward associated with the actions of the players
        r1,r2 = self.__calc_reward(a1, a2)
        
        # provide the corresponding payoff
        #p1.feedback_last(r1, ENV_SINGLE_STATE)
        #p2.feedback_last(r2, ENV_SINGLE_STATE)
        self.__run_feedbacks(p1, p2, r1, r2)
        
        #print '<%s,%s> played <%s,%s> and received <%i,%i>'%(p1.get_name(),p2.get_name(),a1,a2,r1,r2)
        #print '%i\t%f\t%f'%(self._episodes, p1._QTable[ENV_SINGLE_STATE]['C'], p2._QTable[ENV_SINGLE_STATE]['C'])
        #print '%i\t%i\t%i'%(self._episodes, r1, r2)
        print "%i\t%f\t%f"%(self._episodes, p1._policy[ENV_SINGLE_STATE][self._env.keys()[0]], p2._policy[ENV_SINGLE_STATE][self._env.keys()[0]])
    
    def __run_feedbacks(self, p1, p2, r1, r2):
        
        # feedback 1
        p1.feedback1(r1, ENV_SINGLE_STATE)
        p2.feedback1(r2, ENV_SINGLE_STATE)
        
        # feedback 2
        p1.feedback2(r1, ENV_SINGLE_STATE)
        p2.feedback2(r2, ENV_SINGLE_STATE)
        
        # feedback 3
        p1.feedback3(r1, ENV_SINGLE_STATE)
        p2.feedback3(r2, ENV_SINGLE_STATE)
        
        # feedback last
        p1.feedback_last(r1, ENV_SINGLE_STATE)
        p2.feedback_last(r2, ENV_SINGLE_STATE)
        
    def __run_acts(self, p1, p2):
        
        # act 1
        _ = p1.act1()
        _ = p2.act1()
        
        # act 2
        _ = p1.act2()
        _ = p2.act2()
        
        # act 3
        _ = p1.act3()
        _ = p2.act3()
        
        # act 4
        _ = p1.act4()
        _ = p2.act4()
        
        # act last
        _,a1 = p1.act_last()
        _,a2 = p2.act_last()
        
        return [a1, a2]
        
    
    def has_episode_ended(self):
        return self._has_episode_ended
    
    def __calc_reward(self, action_p1, action_p2):
        self._env[action_p1][action_p2]
        return self._env[action_p1][action_p2]
