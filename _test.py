from collections import deque
import pickle
from BP import MainProblem, Route
import heapq

# myfile = open('myfile.txt', 'rb')
# a = pickle.load(myfile)
# myfile.close()

class PriorityQueue:
    def __init__(self):
        self.__queue = []
        self.__index = 0

    def push(self, item, priority):
        heapq.heappush(self.__queue, (priority, self.__index, item))
        # 第一个参数：添加进的目标序列
        # 第二个参数：将一个元组作为整体添加进序列，目的是为了方便比较
        # 在priority相等的情况下，比较_index
        # priority为负数使得添加时按照优先级从大到小排序，因为堆排序的序列的第一个元素永远是最小的
        self.__index += 1

    def pop(self):
        # 返回按照-priority 和 _index 排序后的第一个元素(是一个元组)的最后一个元素(item)
        return heapq.heappop(self.__queue)[-1]

a = PriorityQueue()
a.push('1',-2)
a.push('2',-3)
a.push('3',-1)
a.push('4',-5)
print(a.pop())
print(a.pop())
print(a.pop())
print(a.pop())

if a:
    print('有')
else:
    print('无')

