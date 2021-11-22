from BP import UserParam, columngen, Route
from collections import defaultdict
import pickle


class treeBB:
    def __init__(self, father, son0, branchFrom, branchTo, branchValue, lowestValue, toplevel):
        self.father = father
        self.son0 = son0
        self.branchFrom = branchFrom
        self.branchTo = branchTo
        self.branchValue = branchValue
        self.lowestValue = lowestValue
        self.toplevel = toplevel


def EdgesBasedOnBranching(userParam: UserParam, branching: treeBB, recur: bool):
    if branching.father is not None:
        if branching.branchValue == 0:
            userParam.ban(branching.branchFrom, branching.branchTo)
        elif branching.branchValue == 1:
            if branching.branchFrom != userParam.satellite:
                for j in range(userParam.dist.shape[1]):
                    if j != branching.branchTo:
                        userParam.ban(branching.branchFrom, j)
            if branching.branchTo != userParam.satellite:
                for i in range(userParam.dist.shape[0]):
                    if i != branching.branchFrom:
                        userParam.ban(i, branching.branchTo)
            userParam.ban(branching.branchTo, branching.branchFrom)
        if recur is True:
            EdgesBasedOnBranching(userParam, branching.father, True)


class BranchAndBound:
    lower_bound = -1E10
    upper_bound = 1E10

    def BBNode(self, userParam, routes, branching: treeBB, bestRoutes, depth):
        if ((self.upper_bound - self.lower_bound) / self.upper_bound) < userParam.gap:
            return True

        if branching is None:
            newNode = treeBB(None, None, -1, -1, -1, -1E10, True)
            branching = newNode

        userParam.reset()
        EdgesBasedOnBranching(userParam, branching, True)

        CG = columngen()
        CGFeas, CGobj = CG.computeColGen(userParam, routes)  # 会添加新的routes

        if CGFeas is False:
            return True

        branching.lowestValue = CGobj

        # 关于什么时候可以计算lower_bound
        if branching.father is not None and branching.father.son0 is not None and branching.father.toplevel is True:
            self.lower_bound = min(branching.lowestValue, branching.father.son0.lowestValue)
            branching.toplevel = True
        elif branching.father is None:
            self.lower_bound = CGobj

        if branching.lowestValue > self.upper_bound:
            CG = None
            return True
        else:
            feasible = True
            bestEdge1 = -1
            bestEdge2 = -1
            bestObj = -1.0
            bestVal = 0

            # 正确获取各个弧的值
            edges = defaultdict(float)
            for r in routes:
                if r.getQ() > 1e-6:
                    print(r.getpath())
                    print(r.getQ())
                    path = r.getpath()
                    prev_city = path[0]
                    for city in path[1:]:
                        val = edges[(prev_city, city)]
                        edges[(prev_city, city)] = val + r.getQ()
                        prev_city = city

            # 找出取值为分数的弧
            for i, j in edges.keys():
                coef = edges[(i, j)]  # 可能需要更改
                if coef > 1e-6 and (coef < 0.9999999999 or coef > 1.0000000001):
                    feasible = False
                    change = min(coef, abs(1 - coef))
                    if change > bestObj:
                        bestEdge1 = i
                        bestEdge2 = j
                        bestObj = change
                        if abs(1 - coef) > coef:
                            bestVal = 0
                        else:
                            bestVal = 1

            """
            feasible = False
            bestEdge1 = 1
            bestEdge2 = 2
            bestVal = 0
            """
            if feasible is True:
                if branching.lowestValue < self.upper_bound:
                    self.upper_bound = branching.lowestValue
                    # 增加对于最佳结果的记录
                else:
                    print('feas')
                    # 此时是找到了可行解
                return True
            else:
                # 进行branch
                newNode1 = treeBB(branching, None, bestEdge1, bestEdge2, bestVal, -1E10, False)

                userParam.reset()
                EdgesBasedOnBranching(userParam, newNode1, True)

                nodeRoutes = []

                for r in routes:
                    path = r.getpath()
                    accept = True
                    # 需要保留一些关键的r保证解的可行性
                    for i, j in zip(path, path[1:]):
                        if userParam.banned(i, j):
                            accept = False
                    if accept is True:
                        nodeRoutes.append(r)

                ok = self.BBNode(userParam, nodeRoutes, newNode1, bestRoutes, depth + 1)
                nodeRoutes = None
                if ok is False:
                    return False  # ?

                branching.son0 = newNode1

                newNode2 = treeBB(branching, None, bestEdge1, bestEdge2, 1 - bestVal, -1E10, False)

                userParam.reset()
                EdgesBasedOnBranching(userParam, newNode2, True)

                nodeRoutes2 = []

                for r in routes:
                    path = r.getpath()
                    accept = True
                    # 需要保留一些关键的r保证解的可行性
                    for i, j in zip(path, path[1:]):
                        if userParam.banned(i, j):
                            accept = False
                    if accept is True:
                        nodeRoutes2.append(r)

                ok = self.BBNode(userParam, nodeRoutes2, newNode2, bestRoutes, depth + 1)
                nodeRoutes2 = None

                branching.lowestValue = min(newNode1.lowestValue, newNode2.lowestValue)  # ?

                return ok


if __name__ == '__main__':
    bnb = BranchAndBound()
    bestRoutes = []
    userParam = UserParam()
    branching = None

    myfile = open('C101_10.txt', 'rb')
    routes = pickle.load(myfile)
    myfile.close()

    bnb.BBNode(userParam, routes, branching, bestRoutes, 0)
    print(bnb.upper_bound, bnb.lower_bound)
