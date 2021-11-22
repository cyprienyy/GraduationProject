from Exact_Algorithm import INFO
import gurobipy as gp
from gurobipy import GRB
import numpy as np
from collections import deque, Counter
import pickle
import heapq

BIG = 1E10
GAP = 0.001
COL = 1


class PriorityQueue:
    def __init__(self):
        self.__queue = []
        self.__index = 0

    def _push(self, item, priority):
        heapq.heappush(self.__queue, (priority, self.__index, item))
        # 第一个参数：添加进的目标序列
        # 第二个参数：将一个元组作为整体添加进序列，目的是为了方便比较
        # 在priority相等的情况下，比较_index
        # priority为负数使得添加时按照优先级从大到小排序，因为堆排序的序列的第一个元素永远是最小的
        self.__index += 1

    def _pop(self):
        # 返回按照-priority 和 _index 排序后的第一个元素(是一个元组)的最后一个元素(item)
        return heapq.heappop(self.__queue)[-1]

    def append(self, _label):
        self._push(_label, _label.cost)

    def popleft(self):
        return self._pop()

    def is_empty(self):
        if self.__queue:
            return True
        else:
            return False


class K_PriorityQueue:
    def __init__(self, k):
        self.__queue = []
        self.__index = 0
        self.k = k
        self.cnt = 0

    def push(self, item, priority):
        if self.cnt < self.k:
            heapq.heappush(self.__queue, (-priority, self.__index, item))
            self.__index += 1
            self.cnt += 1
        else:
            if priority < -self.__queue[0][0]:
                self.pop()
                heapq.heappush(self.__queue, (priority, self.__index, item))
                self.__index += 1

    def pop(self):
        # 返回按照-priority 和 _index 排序后的第一个元素(是一个元组)的最后一个元素(item)
        return heapq.heappop(self.__queue)[-1]

    def get_res(self):
        return [a[-1] for a in self.__queue]


class UserParam:
    BIG = BIG
    distBase = INFO.c_ij
    timeBase = INFO.t_ij
    gap = GAP

    # 有一个cost额外用来计算SPPRC

    def __init__(self):
        self.dist = self.distBase.copy()
        self.edges = np.zeros(self.dist.shape)
        self.duals = [20] * INFO.task_num + [30] * INFO.vehicle_num

    def reset(self):
        self.dist = self.distBase.copy()
        self.edges = np.zeros(self.dist.shape)

    def set_dist(self, i, j, dis):
        self.dist[(i, j)] = dis

    def get_dist(self, i, j):
        return self.dist[i, j]

    def adjust_obj_coeff(self, Dualsolution):
        self.duals = Dualsolution.copy()
        return

    def get_cost(self, i, j):
        if 0 < j <= INFO.task_num:
            return self.get_dist(i, j) - self.duals[j - 1]
        else:
            return self.get_dist(i, j)

    def banned(self, i, j):
        if self.get_dist(i, j) <= self.BIG - 1e-6:
            return False
        else:
            return True

    def get_vehicle_fixed_cost(self, vehicle_index):
        return - self.duals[INFO.task_num + vehicle_index]


class Route:
    def __init__(self):
        self.dis = 0
        self.time = 0
        self.vehicle_index = 0
        self.Q = 0
        self.path = list()
        self.dummy = False
        self.coef = list()

    def setcost(self, cost):
        self.cost = cost

    def setQ(self, Q):
        self.Q = Q

    def getcost(self):
        return self.cost

    def getQ(self):
        return self.Q

    def getpath(self):
        return self.path

    def getcoef(self):
        return self.coef

    def update_coef(self):
        self.coef = []
        _counter = Counter([i for i in self.path if 0 < i <= INFO.task_num])
        for i in INFO.tasks:
            self.coef.append(_counter[i])
        for i in range(INFO.vehicle_num):
            if self.vehicle_index == i:
                self.coef.append(1)
            else:
                self.coef.append(0)


class label_set:
    def __init__(self):
        self.deque = deque()

    def add_label(self, label, compare_func):
        flag = True
        _new_deque = deque()
        while self.deque:
            old_label = self.deque.pop()
            res = compare_func(old_label, label)
            if res == -1:
                flag = False
                label.dominated = True
                _new_deque.append(old_label)
            elif res == 0:
                _new_deque.append(old_label)
            elif res == 1:
                old_label.dominated = True
        if flag is True:
            _new_deque.append(label)
        self.deque = _new_deque

    def get_best(self, k):
        if len(self.deque) == 0:
            raise Exception('没有找到任何列')
        res = K_PriorityQueue(k)
        for _label in self.deque:
            if _label.cost < 0:
                res.push(_label, _label.cost)
        res = res.get_res()
        return res

    def print_paths(self):
        for _label in self.deque:
            print(_label.cost, _label.path)


class label:
    start = 0
    intermediate_stop = INFO.task_num + 1
    end = INFO.task_num + 2
    nodes_cnt = INFO.task_num + 3
    task_num = INFO.task_num

    def __init__(self, place, nodes_cnt=INFO.task_num + 3):
        self.cost = 0
        self.time = 0
        self.dis = 0
        self.f = 0
        self.visited = [0] * nodes_cnt
        self.visited[0] = 1
        self.dominated = False
        self.place = place
        self.path = []
        self.vehicle_index = 0

    def extend(self, cost_add, time_add, dis_add, f_add, des, place):
        _new_label = label(place)
        _new_label.cost = self.cost + cost_add
        _new_label.time = self.time + time_add
        _new_label.dis = self.dis + dis_add
        _new_label.visited = self.visited.copy()
        _new_label.visited[des] = 1
        _new_label.vehicle_index = self.vehicle_index

        if des == self.intermediate_stop:
            _new_label.f = INFO.vehicle_capacity
        else:
            _new_label.f = self.f + f_add
            _new_label.visited[self.intermediate_stop] = 0

        for j, stat in enumerate(_new_label.visited):
            if stat == 1 or stat == -1 or j == self.start or j == self.end or \
                    (stat == -2 and des != self.intermediate_stop):
                continue
            if j == self.intermediate_stop:
                test_time = INFO.t_ij[place][INFO.satellites[0]] + \
                            INFO.t_ij[INFO.satellites[0]][INFO.vehicle_end_locations[_new_label.vehicle_index]] + \
                            INFO.service_time[INFO.satellites[0]]
                demand = 0
            else:
                test_time = INFO.t_ij[place][j] + \
                            INFO.t_ij[j][INFO.vehicle_end_locations[_new_label.vehicle_index]] + \
                            INFO.service_time[j]
                demand = INFO.task_demand[j]

            if _new_label.time + test_time > INFO.H:
                _new_label.visited[j] = -1
            elif _new_label.f < demand:
                _new_label.visited[j] = -2
            else:
                _new_label.visited[j] = 0

        if _new_label.time <= INFO.H and _new_label.f >= 0:
            _new_label.path = self.path + [place]
            return _new_label
        else:
            # raise Exception('不应该无法产生新标签')
            return None


class MainProblem:
    def __init__(self, routes):

        self.X_r = []
        self.n_r = np.ones((INFO.task_num + INFO.vehicle_num, 1))
        self.c_r = 1E10
        self.demand_constr = []
        self.vehicle_constr = []

        self.MainProbRelax = gp.Model()  # 松弛后的列生成主问题

        # 构造主问题模型
        # 添加变量
        x = self.MainProbRelax.addVar(lb=0.0, ub=1.0, obj=self.c_r, vtype=GRB.CONTINUOUS, name='x_dummy')
        self.X_r.append(x)
        # 添加约束
        for j in INFO.tasks:
            constr = self.MainProbRelax.addConstr(
                gp.quicksum(self.n_r[j - 1, r] * self.X_r[r] for r in range(self.n_r.shape[1])) == 1,
                name='demand_constr')
            self.demand_constr.append((j, constr))
        for j in INFO.vehicle_start_locations:
            constr = self.MainProbRelax.addConstr(
                gp.quicksum(self.n_r[j - 1, r] * self.X_r[r] for r in range(self.n_r.shape[1])) <= 1,
                name='vehicle_constr')
            self.vehicle_constr.append((j, constr))
        self.MainProbRelax.setAttr(GRB.Attr.ModelSense, GRB.MINIMIZE)

        self.MainProbRelax.update()
        self.MainProbRelax.setParam('OutputFlag', 0)

        self.routes = routes
        if self.routes is not None:
            for r in self.routes:
                self.add_column(r.getcoef(), r.getcost())
        return

    def get_dual_solution(self):
        # self.MainProbRelax.update()
        Dualsolution = self.MainProbRelax.getAttr("Pi", self.MainProbRelax.getConstrs())
        return Dualsolution

    def optimize(self):
        # self.MainProbRelax.update()
        self.MainProbRelax.optimize()

    def add_column(self, columnCoeff, columnVal):
        column = gp.Column(columnCoeff, self.MainProbRelax.getConstrs())
        x = self.MainProbRelax.addVar(obj=columnVal, lb=0.0, ub=1.0, vtype=GRB.CONTINUOUS, name="CG", column=column)
        self.X_r.append(x)
        return

    def add_column_by_route(self, route: Route):
        self.routes.append(route)
        self.add_column(route.getcoef(), route.getcost())


class OneLevel:
    def __init__(self):
        self.sub_problem = None
        self.UL = PriorityQueue()
        self.TL = list()
        for _ in range(INFO.task_num + 3):
            a = label_set()
            self.TL.append(a)

    @staticmethod
    def compare_label1_label2(label1: label, label2: label):
        # 0即无关系，-1代表label1 dominates，1代表label2 dominates
        # if INFO.vehicle_start_load[label1.vehicle_index] != INFO.vehicle_start_load[label2.vehicle_index] \
        #         or INFO.vehicle_start_locations[label1.vehicle_index] \
        #         != INFO.vehicle_start_locations[label2.vehicle_index] \
        #         or INFO.vehicle_end_locations[label1.vehicle_index] \
        #         != INFO.vehicle_end_locations[label2.vehicle_index]:
        #     return 0
        label_1_flag = True
        label_2_flag = True
        count = 0
        for i, visited in enumerate(label1.visited[:label.task_num + 1]):
            # if label2.visited[i] == 1:
            #     count += 1
            if visited == 1 and label2.visited[i] <= 0:
                label_1_flag = False
            if visited <= 0 and label2.visited[i] == 1:
                label_2_flag = False
        # if count == 3:
        #     print('stop')
        if label_1_flag is True and label1.cost <= label2.cost \
                and label1.time <= label2.time and label1.dis <= label2.dis and label1.f >= label2.f:
            return -1
        if label_2_flag is True and label2.cost <= label1.cost \
                and label2.time <= label1.time and label2.dis <= label1.dis and label2.f <= label1.f:
            return 1
        return 0

    @staticmethod
    def return_task_demand(j):
        if j <= INFO.task_num:
            return - INFO.task_demand[j]
        else:
            return 0

    @staticmethod
    def return_penalty(i, real_j, start_time):
        if 0 < real_j <= INFO.task_num:
            return max(start_time + INFO.t_ij[i][real_j] - INFO.task_tw_lb[real_j], 0) * INFO.task_tw_p[real_j] / (
                    INFO.H - INFO.task_tw_lb[real_j])
        else:
            return 0

    def label_setting(self, userParam: UserParam):
        # 成本，时间，满电电池数，缺电电池数，拜访情况列表, dominates
        for i in range(INFO.vehicle_num):
            _label = label(INFO.vehicle_start_locations[i])
            _label.path.append(INFO.vehicle_start_locations[i])
            _label.f = INFO.vehicle_start_load[i]
            _label.vehicle_index = i
            _label.cost = userParam.get_vehicle_fixed_cost(i)
            self.UL.append(_label)
            self.TL[0].add_label(_label, self.compare_label1_label2)

        sol_count = 0
        cnt = 0

        while self.UL.is_empty():
            _label = self.UL.popleft()
            # print(_label.path, _label.dominated)
            cnt += 1

            if _label.place == INFO.vehicle_end_locations[
                _label.vehicle_index] and _label.cost < 0 and _label.dominated is False:
                sol_count += 1

            if sol_count >= 8 * COL:
                break

            if _label.dominated is False and _label.place != INFO.vehicle_end_locations[_label.vehicle_index]:
                # print('开始拓展')
                i = _label.place

                for j, to_visit in enumerate(_label.visited):
                    if j == label.end:
                        real_j = INFO.vehicle_end_locations[_label.vehicle_index]
                    elif j == label.intermediate_stop:
                        real_j = INFO.satellites[0]
                    else:
                        real_j = j

                    if to_visit == 0 and userParam.banned(i, real_j) is False:  # 在此处判断某条弧是否被禁止使用
                        penalty = self.return_penalty(i, real_j, _label.time)
                        # penalty = 0
                        _new_label = _label.extend(userParam.get_cost(i, real_j) + penalty,
                                                   INFO.t_ij[i][real_j] + INFO.service_time[real_j],
                                                   INFO.c_ij[i][real_j] + penalty,
                                                   self.return_task_demand(j),
                                                   j,
                                                   real_j)
                        if _new_label is not None:
                            self.TL[j].add_label(_new_label, self.compare_label1_label2)
                            if _new_label.dominated is False:
                                self.UL.append(_new_label)
        print('处理标签数', cnt)

    def return_result(self):
        _labels = self.TL[-1].get_best(10)
        _cost = 0
        res = []
        for _label in _labels:
            route = Route()
            route.vehicle_index = _label.vehicle_index
            route.time = _label.time
            route.cost = _label.dis
            route.path = _label.path
            _res = [0] * INFO.vehicle_num
            _res[route.vehicle_index] = 1
            _coef = []
            for i in _label.visited[1:INFO.task_num + 1]:
                if i == 1:
                    _coef.append(1)
                else:
                    _coef.append(0)
            route.coef = _coef + _res
            res.append(route)
            if _label.cost < _cost:
                _cost = _label.cost
        return _cost, res

    def clear(self):
        self.UL = PriorityQueue()
        for ls in self.TL:
            ls.deque = deque()


if __name__ == '__main__':
    userParam = UserParam()
    one_level = OneLevel()

    file_name_str = 'C101_15'
    routes = []
    # myfile = open('C101_10.txt', 'rb')
    # routes = pickle.load(myfile)
    # myfile.close()

    main_problem = MainProblem(routes)
    main_problem.optimize()
    _dual = main_problem.get_dual_solution()
    userParam.adjust_obj_coeff(_dual)
    one_level.clear()
    one_level.label_setting(userParam)
    _cost, _routes = one_level.return_result()

    while _cost < -1E-6:
        for _route in _routes:
            # print(_route.path)
            main_problem.add_column_by_route(_route)
        COL += 1
        myfile = open(file_name_str + '.txt', 'wb+')
        pickle.dump(main_problem.routes, myfile)
        myfile.close()
        main_problem.optimize()
        _dual = main_problem.get_dual_solution()
        print(_dual)
        userParam.adjust_obj_coeff(_dual)
        one_level.clear()
        one_level.label_setting(userParam)
        _cost, _routes = one_level.return_result()
        print(_cost)
        # print(_route.path)
    main_problem.MainProbRelax.write(file_name_str + '.sol')
    print('finished')
