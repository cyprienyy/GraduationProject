from collections import deque
import pickle
from BP import MainProblem, Route
import heapq
import numpy as np

# myfile = open('myfile.txt', 'rb')
# a = pickle.load(myfile)
# myfile.close()
#
# class K_PriorityQueue:
#     def __init__(self, k):
#         self.__queue = []
#         self.__index = 0
#         self.k = k
#         self.cnt = 0
#
#     def push(self, item, priority):
#         if self.cnt < self.k:
#             heapq.heappush(self.__queue, (-priority, self.__index, item))
#             self.__index += 1
#             self.cnt += 1
#         else:
#             if priority < -self.__queue[0][0]:
#                 self.pop()
#                 heapq.heappush(self.__queue, (priority, self.__index, item))
#                 self.__index += 1
#
#     def pop(self):
#         # 返回按照-priority 和 _index 排序后的第一个元素(是一个元组)的最后一个元素(item)
#         return heapq.heappop(self.__queue)[-1]
#
#     def get_res(self, _label):
#         return self.__queue
#
# a = K_PriorityQueue(2)
# a.push('1',-2)
# a.push('2',-3)
# a.push('3',-1)
# a.push('4',-5)
# print(a.pop())
# print(a.pop())
# print(a.pop())
# print(a.pop())
#
# if a:
#     print('有')
# else:
#     print('无')

a = np.zeros((3,4))
print(a)
print(a.shape[0])
print(a.shape[1])

