from collections import deque
import pickle
from BP import MainProblem, Route

myfile = open('myfile.txt', 'rb')
a = pickle.load(myfile)
myfile.close()
