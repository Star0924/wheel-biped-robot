import numpy as np
from scipy.optimize import root


L1 = 130
L2 = 130

a = -126.23
b = 69.77

h_target = 200   # <-- 改成你想要的目標高度



def equations(q):

    q1 = q[0]
    q2 = q[1]

    eq1 = L1 * np.cos(q1) + L2 * np.cos(q2) - h_target

    eq2 = a * np.sin(q1) + b * np.sin(q2) + 1.66

    return [eq1, eq2]



q_initial = [
    np.deg2rad(30),
    np.deg2rad(30)
]



solution = root(equations, q_initial)



if solution.success:

    q1 = solution.x[0]
    q2 = solution.x[1]

    print("求解成功")
    print("--------------------")

    print(f"q1 = {q1:.6f} rad")
    print(f"q2 = {q2:.6f} rad")

    print(f"q1 = {np.rad2deg(q1):.3f} deg")
    print(f"q2 = {np.rad2deg(q2):.3f} deg")

else:

    print("求解失敗")
    print(solution.message)