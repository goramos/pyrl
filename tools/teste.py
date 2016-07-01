import random
from random import shuffle

def f1(P):
	i = -1
	r = random.random()
	
	for p in P:
		i+=1
		if r < p:
			return P[i]
		else:
			r -= p
	
	return P[i]

def test_shuffle():
	P = {0.2: 0, 0.3: 0, 0.5: 0}
	
	for _ in xrange(1000000):
		P1 = list(P.keys())
		shuffle(P1)
		p = f1(P1)
		P[p] += 1
	
	print P
