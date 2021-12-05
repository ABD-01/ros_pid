import numpy as np
import matplotlib.pyplot as plt

H = np.array([[2,-2,1,1],[-3,3,-2,-1],[0,0,1,0],[1,0,0,0]])

theta =[0, np.deg2rad(0)]
theta_dot = np.array([0,0])

X = np.array([5, 1])
X_dot = np.cos(theta)
Y = np.array([5, 1])
Y_dot = np.sin(theta)


def get_traj():
    p = np.array([[X[0], X[1], X_dot[0], X_dot[1]],
                    [Y[0], Y[1], Y_dot[0], Y_dot[1]]]).T
    # print(p)
    A = np.dot(H, p)
    # print(A)
    return A

def get_curve(t, A):
    Time = np.array([t**3, t**2, t, 1]).reshape(-1,4)
    return np.dot(Time, A)

print(get_traj())
print(get_curve(0,get_traj()))

def draw():
    time = 0
    datax = []
    datay =[] 
    A= get_traj()
    while time < 1.01:
        c= (get_curve(time, A)).squeeze()
        # print(c.shape)
        datax.append(c[0])
        datay.append(c[1])
        time += 0.005

    plt.plot([5,1], [5,1], 'o')
    plt.plot(datax, datay)
    plt.show()

draw()