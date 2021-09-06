import numpy as np
import geatpy as ea
import re


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
        self.satellite = 0
        self.satellite_ser = 0

        self.vehicle_num = 0
        self.vehicle_H = []
        self.vehicle_capacity = []
        self.vehicle_start_locations = []
        self.vehicle_end_locations = []
        self.vehicle_start_load = []

        self.task_num = 0
        self.task_demand = [0]
        self.task_tw_lb = [0]
        self.task_tw_ub = [0]
        self.task_tw_p = [0]
        self.task_ser = [0]
        self.init_with_solomon()

    def init_with_solomon(self):
        pos = self.resolve_self_created_case(r'.\RC201.csv')
        station_num, self.vehicle_num, _capacity, H = map(int, pos[0])
        self.c_ij = np.array(pos[1:station_num + 2])
        self.t_ij = self.c_ij / 1
        self.satellite = 0
        self.satellite_ser = 90

        self.vehicle_H = [H] * self.vehicle_num
        self.vehicle_capacity = [_capacity] * self.vehicle_num
        self.vehicle_start_locations = [0] * self.vehicle_num
        self.vehicle_end_locations = [0] * self.vehicle_num
        self.vehicle_start_load = [_capacity] * self.vehicle_num

        self.task_num = station_num
        _nodes_info = pos[station_num + 2:]
        _nodes, _t_lower_bound, _t_upper_bound, _demand, _t_ser, _w_i, _H, _loss = zip(*_nodes_info)
        _nodes = list(map(int, _nodes))
        self.task_tw_lb += list(map(int, _t_upper_bound))
        self.task_tw_ub += [H] * station_num
        self.task_demand += list(map(int, _demand))
        self.task_ser += list(map(int, _t_ser))
        self.task_tw_p += list(map(float, _loss))

    def calculate_tw_penalty(self, task, t):
        return self.task_tw_p[task] * (max(t, self.task_tw_lb[task]) - self.task_tw_lb[task]) / (
                self.task_tw_ub[task] - self.task_tw_lb[task])

    def decode_journey(self, journey):
        _route = []
        _v = 0
        _load = self.vehicle_start_load[_v]
        _prev_loc = self.vehicle_start_locations[_v]
        _t = 0
        _dis = 0
        _t_p = 0
        while journey:
            _task = journey[0]
            if _load < self.task_demand[_task]:
                check_time = self.t_ij[_prev_loc, self.satellite] + self.t_ij[self.satellite, _task] + self.t_ij[
                    _task, self.vehicle_end_locations[_v]] + self.task_ser[_task] + self.satellite_ser
                flag = True
            else:
                check_time = self.t_ij[_prev_loc, _task] + self.t_ij[_task, self.vehicle_end_locations[_v]] + \
                             self.task_ser[_task]
                flag = False
            if _t + check_time > self.vehicle_H[_v]:
                if _v >= self.vehicle_num - 1:
                    return 10000000  # 进行惩罚
                else:
                    _route.append('end')
                    _dis += self.c_ij[_prev_loc, self.vehicle_end_locations[_v]]
                    _v += 1
                    _load = self.vehicle_start_load[_v]
                    _prev_loc = self.vehicle_start_locations[_v]
                    _t = 0
            else:
                if flag:
                    _route.append(-1)
                    _dis = _dis + self.c_ij[_prev_loc, self.satellite] + self.c_ij[self.satellite, _task]
                    _t = _t + self.t_ij[_prev_loc, self.satellite] + self.c_ij[
                        self.satellite, _task] + self.satellite_ser
                    _load = self.vehicle_capacity[_v] - self.task_demand[_task]
                else:
                    _dis += self.c_ij[_prev_loc, _task]
                    _t += self.t_ij[_prev_loc, _task]
                    _load -= self.task_demand[_task]
                _route.append(_task)
                _t_p += self.calculate_tw_penalty(_task, _t)
                _t += self.task_ser[_task]
                _prev_loc = _task
                del journey[0]
        _dis += self.c_ij[_prev_loc, self.vehicle_end_locations[_v]]
        print(_route, _dis+_t_p)
        return _dis + _t_p  # 返回值


class MyProblem(ea.Problem):  # 继承Problem父类
    def __init__(self):
        name = 'MyProblem'  # 初始化name（函数名称，可以随意设置）

        self.info = Info()

        M = 1  # 初始化M（目标维数）
        maxormins = [1]  # 初始化maxormins（目标最小最大化标记列表，1：最小化该目标；-1：最大化该目标）
        Dim = self.info.task_num  # 初始化Dim（决策变量维数）
        varTypes = [1] * Dim  # 初始化varTypes（决策变量的类型，元素为0表示对应的变量是连续的；1表示是离散的）
        lb = [1] * Dim  # 决策变量下界
        ub = [self.info.task_num] * Dim  # 决策变量上界
        lbin = [1] * Dim  # 决策变量下边界（0表示不包含该变量的下边界，1表示包含）
        ubin = [1] * Dim  # 决策变量上边界（0表示不包含该变量的上边界，1表示包含）
        # 调用父类构造方法完成实例化
        ea.Problem.__init__(self, name, M, maxormins, Dim, varTypes, lb, ub, lbin, ubin)

    def aimFunc(self, pop):  # 目标函数
        X = pop.Phen.astype(int)  # 得到决策变量矩阵
        ObjV = []
        for i in range(X.shape[0]):
            journey = X[i].tolist()  # 按既定顺序到达的地点坐标
            distance = self.info.decode_journey(journey)
            ObjV.append(distance)
        pop.ObjV = np.array([ObjV]).T  # 把求得的目标函数值赋值给种群pop的ObjV


if __name__ == '__main__':
    '''
    """================================实例化问题对象==========================="""
    problem = MyProblem()  # 生成问题对象
    Encoding = 'P'  # 编码方式，采用排列编码
    NIND = 100  # 种群规模
    Field = ea.crtfld(Encoding, problem.varTypes, problem.ranges, problem.borders)  # 创建区域描述器
    population = ea.Population(Encoding, Field, NIND)  # 实例化种群对象（此时种群还没被初始化，仅仅是完成种群对象的实例化）
    """================================算法参数设置============================="""
    myAlgorithm = ea.soea_EGA_templet(problem, population)
    myAlgorithm.MAXGEN = 100  # 最大进化代数
    myAlgorithm.logTras = 1  # 设置每隔多少代记录日志，若设置成0则表示不记录日志
    myAlgorithm.verbose = True  # 设置是否打印输出日志信息
    myAlgorithm.drawing = 1  # 设置绘图方式（0：不绘图；1：绘制结果图；2：绘制目标空间过程动画；3：绘制决策空间过程动画）
    """===========================调用算法模板进行种群进化========================"""
    [BestIndi, population] = myAlgorithm.run()
    BestIndi.save()
    """==================================输出结果=============================="""
    print('评价次数：%s' % myAlgorithm.evalsNum)
    print('时间已过 %s 秒' % myAlgorithm.passTime)
    if BestIndi.sizes != 0:
        print('最优的目标函数值为：%s' % (BestIndi.ObjV[0][0]))
        print('最优的控制变量值为：')
        print(BestIndi.Phen[0])
    else:
        print('没找到可行解。')
    '''
    problem = MyProblem()
    problem.info.decode_journey([20, 22, 14, 17, 11, 16, 12, 25, 24, 1, 5, 7, 10, 23, 19, 18, 21, 15, 13, 9, 6, 8, 2, 4,
                                 3]
                                )
