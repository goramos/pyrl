'''
Created on 07/11/2014

@author: Gabriel de O. Ramos <goramos@inf.ufrgs.br>
'''
from environment import *

# 2-player-2-action games
# the encoding of a game is composed by 5 cells separated by ; where 
# the first corresponds to the two actions (separated by ,) and 
# the others to the reward-pairs (also separated by ,).
# For example, consider the prisoner's dilemma game below:
#        +-------+-------+
#        |   C   |   D   |
#  +-----+-------+-------+
#  |  C  | -1/-1 | -5/0  |
#  +-----+-------+-------+
#  |  D  |  0/-5 | -2/-2 |
#  +-----+-------+-------+
# it must be encoded as "C,D;-1,-1;-5,0;0,-5;-2,-2", where:
#   - "C,D" are the available actions
#   - "-1,-1" are the payoffs of player 1 and 2 after playing actions C and C, respectively
#   - "-5,0" are the payoffs of player 1 and 2 after playing actions C and D, respectively
#   - "0,-5" are the payoffs of player 1 and 2 after playing actions D and C, respectively
#   - "-2,-2" are the payoffs of player 1 and 2 after playing actions D and D, respectively 
GAME_PRISONERS_DILEMMA = "C,D;-1,-1;-5,0;0,-5;-2,-2"
GAME_MATCHING_PENNIES = "H,T;1,-1;-1,1;-1,1;1,-1"
GAME_COORDINATION_GAME = "a1,a2;2,1;0,0;0,0;1,2"
GAME_BATTLE_OF_THE_SEXES = "B,S;5,6;1,1;2,2;6,5"
GAME_TRICKY = "a1,a2;0,3;3,2;1,0;2,1"
GAME_UD = "U,D;2,-3;1,2;1,1;4,-1"#name unknown :( mixed Nash equilibrium: U is played with probability 2/7 (p1) and 3/4 (p2)

# 2-player-3-action games
GAME_ROCK_PAPER_SCISSORS = "R,P,S;0,0;-1,1;1,-1;1,-1;0,0;-1,1;-1,1;1,-1;0,0"
GAME_SHAPLEY = "a1,a2,a3;0,0;1,0;0,1;0,1;0,0;1,0;1,0;0,1;0,0"