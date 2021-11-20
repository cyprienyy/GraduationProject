import gurobipy as gp
from gurobipy import GRB
import numpy as np
import re

satellite_num = 9
satellite_ser = 90


class Info:

    @staticmethod
    def resolve_self_created_case(filename):
        with open(filename, 'r') as file_to_read:
            pos = []
            while True:
                lines = file_to_read.readline()  # 整行读取数据
                if not lines:
                    break
                    pass
                if re.match(r'\s*[0-9]', lines) is not None:
                    lines = lines.strip()
                    lines = lines.split(',')
                    lines = list(map(float, lines))
                    pos.append(lines)  # 添加新读取的数据
                pass
        pass
        return pos

    def __init__(self):
        self.c_ij = np.zeros((1, 1))
        self.t_ij = np.zeros((1, 1))
        self.satellite_ser = 0
        self.H = 0

        self.vehicle_num = 0
        self.vehicle_set = []
        self.vehicle_capacity = 0
        self.vehicle_start_locations = []
        self.vehicle_end_locations = []
        self.vehicle_start_load = []

        self.satellites = []

        self.task_num = 0
        self.tasks = []
        self.task_demand = [0]
        self.task_tw_lb = [0]
        self.task_tw_p = [0]
        self.task_ser = [0]
        self.service_time = [0]
        self.init_with_solomon()

    def init_with_solomon(self):
        """================================更改文件路径==========================="""
        pos = self.resolve_self_created_case(r'.\C101_15.csv')
        station_num, self.vehicle_num, _capacity, H = map(int, pos[0])
        self.H = H
        c_ij = np.array(pos[1:station_num + 2])
        t_ij = c_ij / 1
        self.satellite_ser = satellite_ser
        self.task_num = station_num

        self.vehicle_set = list(range(self.vehicle_num))
        self.vehicle_capacity = _capacity
        self.vehicle_start_locations = list(range(self.task_num + 1, self.task_num + self.vehicle_num + 1))
        self.vehicle_end_locations = list(
            range(self.task_num + self.vehicle_num + 1, self.task_num + 2 * self.vehicle_num + 1))
        self.vehicle_start_load = [0] * self.vehicle_num

        self.satellites = list(
            range(self.task_num + 2 * self.vehicle_num + 1, self.task_num + 2 * self.vehicle_num + satellite_num + 1))

        self.tasks = list(range(1, self.task_num + 1))
        _nodes_info = pos[station_num + 2:]
        _nodes, _t_lower_bound, _t_upper_bound, _demand, _t_ser, _w_i, _H, _loss = zip(*_nodes_info)
        _nodes = list(map(int, _nodes))
        self.task_tw_lb += list(map(int, _t_upper_bound))
        self.task_demand += list(map(int, _demand))
        self.task_ser += list(map(int, _t_ser))
        self.task_tw_p += list(map(float, _loss))

        self.service_time = self.task_ser + [0] * 2 * self.vehicle_num + [self.satellite_ser] * satellite_num

        self.c_ij = np.zeros((station_num + satellite_num + 1 + 2 * self.vehicle_num,
                              station_num + satellite_num + 1 + 2 * self.vehicle_num))
        self.c_ij[:station_num + 1, :station_num + 1] = c_ij
        self.c_ij[station_num + 1:, :station_num + 1] = c_ij[0, :station_num + 1]
        self.c_ij[:station_num + 1, station_num + 1:] = c_ij[:station_num + 1, 0][:, np.newaxis]

        self.t_ij = np.zeros((station_num + satellite_num + 1 + 2 * self.vehicle_num,
                              station_num + satellite_num + 1 + 2 * self.vehicle_num))
        self.t_ij[:station_num + 1, :station_num + 1] = t_ij
        self.t_ij[station_num:, :station_num + 1] = t_ij[0, :station_num + 1]
        self.t_ij[:station_num + 1, station_num + 1:] = t_ij[:station_num + 1, 0][:, np.newaxis]

    def build_model(self):
        lis_x_ij_v = []
        lis_i_j = []

        for i in self.vehicle_set:
            for j in self.tasks + self.satellites:
                lis_x_ij_v.append((self.vehicle_start_locations[i], j, i))
                lis_x_ij_v.append((j, self.vehicle_end_locations[i], i))
                lis_i_j.append((self.vehicle_start_locations[i], j))
                lis_i_j.append((j, self.vehicle_end_locations[i]))

        for i in self.tasks:
            for j in self.satellites:
                lis_i_j.append((i, j))
                lis_i_j.append((j, i))
                for v in self.vehicle_set:
                    lis_x_ij_v.append((i, j, v))
                    lis_x_ij_v.append((j, i, v))

        for i in self.tasks:
            for j in self.tasks:
                if i != j:
                    lis_i_j.append((i, j))
                    for v in self.vehicle_set:
                        lis_x_ij_v.append((i, j, v))

        lis_i = [i for i in
                 (self.tasks + self.satellites + self.vehicle_start_locations + self.vehicle_end_locations)]

        model = gp.Model('model')
        # 添加变量
        x_i_j_v = model.addVars(lis_x_ij_v, vtype=GRB.BINARY, name='x_i_j_v')
        t_i = model.addVars(lis_i, vtype=GRB.CONTINUOUS, lb=0.0, ub=self.H, name='t_i')
        l_i = model.addVars(lis_i, vtype=GRB.CONTINUOUS, lb=0.0, ub=self.H, name='l_i')
        p_i = model.addVars((i for i in self.tasks), vtype=GRB.CONTINUOUS, lb=0.0, name='p_i')

        # 添加路径约束
        model.addConstrs(x_i_j_v.sum(i, '*', '*') == 1 for i in self.tasks)
        model.addConstrs(x_i_j_v.sum(i, '*', '*') <= 1 for i in self.satellites)
        model.addConstrs(x_i_j_v.sum(i, '*', '*') == 1 for i in self.vehicle_start_locations)
        model.addConstrs(x_i_j_v.sum('*', i, '*') == 1 for i in self.vehicle_end_locations)
        model.addConstrs(
            x_i_j_v.sum('*', i, v) == x_i_j_v.sum(i, '*', v) for i in (self.tasks + self.satellites) for v in
            self.vehicle_set)
        # 添加容量约束
        model.addConstrs(l_i[i] <= self.vehicle_capacity for i in self.tasks)
        model.addConstrs(self.task_demand[i] <= l_i[i] for i in self.tasks)
        model.addConstrs(l_i[i] == 0 for i in self.satellites)
        model.addConstrs(
            l_i[self.vehicle_start_locations[v]] == self.vehicle_start_load[v] for v in self.vehicle_set)
        model.addConstrs(
            l_i[i] + self.task_demand[j] <= l_i[j] + self.vehicle_capacity * (1 - x_i_j_v.sum(i, j, '*')) for
            i, j in
            lis_i_j
            if j in self.tasks)
        # 添加时间约束
        model.addConstrs(
            t_i[i] + self.service_time[i] + self.t_ij[i, j] <= t_i[j] + self.H * (1 - x_i_j_v.sum(i, j, '*')) for
            i, j in
            lis_i_j)
        model.addConstrs(p_i[i] >= t_i[i] - self.task_tw_lb[i] for i in self.tasks)
        # 添加目标函数
        obj_expr = 0
        for i, j, v in lis_x_ij_v:
            obj_expr += x_i_j_v[i, j, v] * self.c_ij[i, j]
        for i in self.tasks:
            obj_expr += p_i[i] * self.task_tw_p[i] / (self.H - self.task_tw_lb[i])

        model.setObjective(obj_expr, GRB.MINIMIZE)

        model.optimize()

        model.write('1.sol')


INFO = Info()
INFO.vehicle_start_load = [INFO.vehicle_capacity - _l for _l in INFO.vehicle_start_load]

if __name__ == '__main__':
    info = Info()
    info.build_model()

'''
# 总体信息
c_ij = []
t_ij = []
H = 0

# 换电站点信息
task_num = 10
tasks = list(range(1, task_num + 1))
service_time = []
task_demand = []
task_tw_lb = []
task_tw_ub = []
task_tw_p = []

# 电池仓库信息
satellites = []

# 车辆信息
vehicle_num = 2
vehicle_set = list(range(vehicle_num))
vehicle_H = [3600, 3600]
vehicle_capacity = [30, 30]
vehicle_start_locations = list(range(task_num + 1, task_num + vehicle_num + 1))
vehicle_end_locations = list(range(task_num + vehicle_num + 1, task_num + 2 * vehicle_num + 1))
vehicle_start_load = [0, 0]



"""================================添加正确的弧变量================================"""
for i in tasks:
# 非线性目标函数线性化
model.setObjective(obj_expr, GRB.MINIMIZE)
model.optimize()
model.write('model_sol_1.sol')
'''
