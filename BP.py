from Exact_Algorithm import INFO
import gurobipy as gp
from gurobipy import GRB
import numpy as np
from collections import deque, Counter
import pickle

BIG = 1E10
GAP = 0.001

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

    def get_best(self):
        if len(self.deque) == 0:
            raise Exception('没有找到任何列')
        res = float('inf')
        _best = None
        for _label in self.deque:
            if _label.cost < res:
                _best = _label
                res = _label.cost
        return _best

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

        if _new_label.time <= INFO.H and _new_label.f >= 0:
            _new_label.path = self.path + [place]
            return _new_label
        else:
            return None


class MainProblem:
    def __init__(self, routes):

        self.X_r = []
        self.n_r = np.ones((INFO.task_num + INFO.vehicle_num, 1))
        self.c_r = 1E10

        self.MainProbRelax = gp.Model()  # 松弛后的列生成主问题

        # 构造主问题模型
        # 添加变量
        x = self.MainProbRelax.addVar(lb=0.0, ub=1.0, obj=self.c_r, vtype=GRB.CONTINUOUS, name='x_dummy')
        self.X_r.append(x)
        # 添加约束
        for j in INFO.tasks:
            self.MainProbRelax.addConstr(
                gp.quicksum(self.n_r[j - 1, r] * self.X_r[r] for r in range(self.n_r.shape[1])) == 1,
                name='demand_constr')
        for j in INFO.vehicle_start_locations:
            self.MainProbRelax.addConstr(
                gp.quicksum(self.n_r[j - 1, r] * self.X_r[r] for r in range(self.n_r.shape[1])) <= 1,
                name='vehicle_constr')
        self.MainProbRelax.setAttr(GRB.Attr.ModelSense, GRB.MINIMIZE)

        self.MainProbRelax.update()

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
        self.UL = deque()
        self.TL = list()
        for _ in range(INFO.task_num + 3):
            a = label_set()
            self.TL.append(a)

    @staticmethod
    def compare_label1_label2(label1: label, label2: label):
        # 0即无关系，-1代表label1 dominates，1代表label2 dominates
        if label1.vehicle_index != label2.vehicle_index:
            return 0
        label_1_flag = True
        label_2_flag = True
        count = 0
        for i, visited in enumerate(label1.visited[:label.task_num + 1]):
            if label2.visited[i] == 1:
                count += 1
            if visited > label2.visited[i]:
                label_1_flag = False
            if visited < label2.visited[i]:
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

        while self.UL:
            _label = self.UL.popleft()
            # print(_label.path)
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
                        _new_label = _label.extend(userParam.get_cost(i, real_j) + penalty,
                                                   INFO.t_ij[i][real_j] + INFO.service_time[real_j],
                                                   INFO.c_ij[i][real_j] + penalty,
                                                   self.return_task_demand(j),
                                                   j,
                                                   real_j)
                        if _new_label is not None:
                            self.UL.append(_new_label)
                            self.TL[j].add_label(_new_label, self.compare_label1_label2)

    def return_result(self):
        _label = self.TL[-1].get_best()
        route = Route()
        route.vehicle_index = _label.vehicle_index
        route.time = _label.time
        route.cost = _label.dis
        route.path = _label.path
        _res = [0] * INFO.vehicle_num
        _res[route.vehicle_index] = 1
        route.coef = _label.visited[1:INFO.task_num + 1] + _res
        return _label.cost, route

    def clear(self):
        self.UL = deque()
        for ls in self.TL:
            ls.deque = deque()


if __name__ == '__main__':
    userParam = UserParam()
    one_level = OneLevel()
    main_problem = MainProblem([])
    main_problem.optimize()
    _dual = main_problem.get_dual_solution()
    userParam.adjust_obj_coeff(_dual)
    one_level.clear()
    one_level.label_setting(userParam)
    _cost, _route = one_level.return_result()



    while _cost < -1E-6:
        main_problem.add_column_by_route(_route)
        main_problem.optimize()
        _dual = main_problem.get_dual_solution()
        userParam.adjust_obj_coeff(_dual)
        one_level.clear()
        one_level.label_setting(userParam)
        _cost, _route = one_level.return_result()
        print(_dual)
        print(_cost)
        print(_route.path)

    myfile = open('myfile.txt', 'wb+')
    pickle.dump(main_problem.routes, myfile)
    myfile.close()

    print('finished')
