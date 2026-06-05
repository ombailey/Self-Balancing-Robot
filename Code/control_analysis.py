import numpy as np
import control as ct
import matplotlib.pyplot as plt

body_com_length = 0.087
body_mass = 0.320
wheel_mass = 0.118
wheel_radius = 0.025
gravity = 9.81
# x1 = wheel position
# x2 = wheel velocity
# x3 = robot angle
# x4 = robot angular velocity

den = 3*wheel_mass*body_mass*body_com_length**2
A = np.array([[0, 1, 0, 0], [0,0, (-body_mass**2 *body_com_length**2 *gravity)/den, 0],
              [0, 0, 0, 1],[0, 0, ((body_mass+3*wheel_mass)*body_mass*gravity*body_com_length)/den, 0]])
B = np.array([[0],[(body_mass*body_com_length**2)/den], [0], [(-body_mass*body_com_length)/den]])
C = np.array([[0, 0, 1, 0]])
D = np.array([[0]])
sys = ct.ss(A,B,C,D)
K = ct.place(A,B,[-4,-5,-7,-8])
Acl = A-B*K
sys_cl = ct.ss(Acl,B,C,D)
t,y = ct.initial_response(sys_cl,T=np.linspace(0,5,1000), initial_state=[0, 0, 0.8726646, 0]  )
plt.plot(t,y)
plt.xlabel("Time (s)")
plt.ylabel("Angle (rad)")
plt.grid(True)
plt.show()