import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
from gurobipy import *

no_of_vehicles = 5
df = pd.read_csv("WRSN-coordinates.txt", ' ')
Y = list(df["Y"]);
X = list(df["X"])
coordinates = np.column_stack((X, Y))
n = len(coordinates)
m = Model("MVRP")
x = {};
y = {}
dist_matrix = np.empty([n, n])
for i in range(n):
    for j in range(n):
        x[i, j] = m.addVar(vtype=GRB.BINARY, name="x%d,%d" % (i, j))  # 决策变量，公式1
        dist_matrix[i, j] = np.sqrt((X[i] - X[j]) ** 2 + (Y[i] - Y[j]) ** 2)  # 距离矩阵
m.setObjective(quicksum(quicksum(x[(i, j)] * dist_matrix[(i, j)] for j in range(n)) for i in range(n)),
               GRB.MINIMIZE)  # 目标函数, 公式2
m.addConstr(quicksum(x[(0, j)] for j in range(n)) == no_of_vehicles)  # 约束条件1，公式3
m.addConstr(quicksum(x[(i, 0)] for i in range(n)) == no_of_vehicles)  # 约束条件2，公式4
for i in range(n - 1):
    m.addConstr(quicksum(x[(i + 1, j)] for j in range(n)) == 1)  # 约束条件3，公式5
for j in range(n - 1):
    m.addConstr(quicksum(x[(i, j + 1)] for i in range(n)) == 1)  # 约束条件4，公式6

for j in range(n):  # 约束条件5，公式7
    for i in range(n):
        m.addConstr((x[i, j] + x[j, i]) <= 1)
m.update()
m.optimize()
print('\n minimum distance: %g' % m.objVal)
m.printAttr('x')
from_node = []
to_node = np.empty([n, n])

for v in m.getVars():
    from_node.append(v.x)
for i in range(n):
    for j in range(n):
        to_node[i, j] = from_node[n * i + j]
print('\nDistance Matrix (dij):\n', pd.DataFrame(dist_matrix).astype('int64'))
print('\nDecision (Xij):\n', pd.DataFrame(to_node).astype('int64'))
# to node:决策矩阵
I = []  # 决策矩阵的行
J = []  # 决策矩阵的列
for i in range(n):
    for j in range(n):
        if to_node[i, j] > 0.5:
            I.append(i)
            J.append(j)
# XX1 = [];XX2 = [];YY1 = [];YY2 = []
# 以下为新增部分
print(np.array(I))
print(np.array(J))
all_routes = {}
cnt = 0
# for i in I:
#     XX1.append(X[i])
#     YY1.append(Y[i])
# for j in J:
#     XX2.append(X[j])
#     YY2.append(Y[j])
# print([XX1,XX2])


# 根据决策矩阵来求解出各条子路径的值，
# 循环遍历的初始条件：决策列表的前no_of_vehicles作为遍历开始，也就是说列表变量route的初始化从(0，J[0]),或(0，J[1]),.。。。(0，J[no_of_vehicles])开始的
# 循环遍历的执行内容：根据当前的决策变量I，下一个行J，
# 循环遍历的结束条件： -,0
for i in range(no_of_vehicles):
    end = J[i]  # 取出(0,J[0])的J[0]
    route = [0, end]  # 初始route=[0,J[0]]
    while end != 0:
        start_index = I.index(end)  # 找到J[0]在I[]中对应的index，也就是下一个取值的start_index
        end = J[start_index]  # 将这个index, 到J[]中去找对应的值，
        route.append(end)  # 将该值添加到子路径列表route中
    all_routes[str(i)] = route  # 多条子路径构成了总的路径表达
# 输出all_routes, 包括各条子路径
print('all sub route trace:')
print(all_routes)

sub_route_total_distance = {}

for i in range(no_of_vehicles):
    prev_index = 0
    # print('\n')
    sub_route_total_distance[i] = 0
    for j in range(len(all_routes[str(i)])):
        cur_index = all_routes[str(i)][j]
        # print('dist[%d,%d]=%d' % (prev_index, cur_index, dist_matrix[prev_index][cur_index]))
        sub_route_total_distance[i] += dist_matrix[prev_index][cur_index]
        prev_index = cur_index
print('all sub route total distance sum:')
print(sub_route_total_distance)

# 图1 原始的WRSN 传感器节点和充电器
plt.figure()
plt.scatter(X[1:], Y[1:], marker='o', color='blue')
plt.scatter(X[0], Y[0], marker='^', color='blue')
plt.xlabel('x-coordinate(m)')
plt.ylabel('y-coordinate(m)')
for i in range(no_of_vehicles):
    plt.scatter(X[0] - 2 + i, Y[0] - 1, marker='^', color='red')
plt.title('Original sensors and chargers positions')

# 图2 MTSP求解之后的轨迹
plt.figure()
plt.scatter(X[1:], Y[1:], marker='o', color='blue')
plt.scatter(X[0], Y[0], marker='^', color='blue')
plt.xlabel('x-coordinate(m)')
plt.ylabel('y-coordinate(m)')
plt.title('Total min distance %g (m)' % m.objVal)
# plt.plot([XX1, XX2], [YY1, YY2])
for index in all_routes:
    route_list = all_routes[index]
    x = []
    y = []
    for i in route_list:
        x.append(X[i])
        y.append(Y[i])
    plt.plot(x, y)
plt.show()
