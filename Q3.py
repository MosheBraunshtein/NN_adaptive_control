import numpy as np
import matplotlib.pyplot as plt 

def calc_h(x, x_dot):
    C = [(0,-2.5),(2,-1),(4,0.5),(6,2),(8,2.5)]
    b = 0.5
    h = np.zeros((5,1))
    i=0 
    for c in C:
        c1,c2 = c 
        r = np.sqrt((x-c1)**2 + (x_dot-c2)**2)
        hi = np.exp(-(r**2)/(2*(b**2)))
        h[i,0] = hi
        i += 1
    return h


# system parameters
A = B = D = 1
m = 0.5
F = 1
a1 = A/m
a2 = - B/m
b2 = -F/m
c = D/m
c_min = 1
# start conditions
w_hat = np.array([[5], [10], [15], [20], [25]])
c_hat = 1.5
x = x_dot = x_dotdot = np.array([[0.5]])
x_d = np.array([[1]])
# simulation parameters
dt = 0.001 # s
t = 10 # s
num_steps =int(t/dt)
# reference signal parameters x_d_dot = -a*x_d + b*sin(w*dt)
a = 1
b = 3
w = 3
# controller parameters
gamma = 10
eta = 30
kp = 50
kd = 1
K = np.array([[kp],[kd]])
q11 , q12, q21, q22 = 1 , 0, 0, 1 
p12 = q11 / (2 * kp)
p22 = q22 / (2 * kd)
p11 = (q12+q21) / (2 * kp) + kd / (kp * p12) 
P = np.array([[p11,p12],[p12,p22]])
# for plot
x_vec = [None]*(num_steps+1)
x_vec[0] = x
x_d_vec = [None]*(num_steps+1)
x_d_vec[0] = x_d
f_hat_vec = [None]*(num_steps)
f_vec = [None]*(num_steps)
u_vec = [None]*(num_steps)
c_hat_vec = [None]*(num_steps)

for step in range(num_steps):
    t = step*dt

    x_d_dot = b*np.sin(w*t) - a*x_d
    x_d_dotdot = -a*x_d_dot + b*w*np.cos(w*t)
    x_d = x_d + x_d_dot*dt
    x_d_vec[step+1] = x_d
    
    # calc error
    e = x_d[0,0] - x[0,0]
    e_dot = x_d_dot[0,0] - x_dot[0,0]
    E = np.array([[e],[e_dot]])

    
    # calc w_estimate
    h = calc_h(x,x_dot)
    w_hat_dot = - (np.array([[gamma]]) @ E.T @ P @ np.array([[0],[1]])) * h 
    w_hat = w_hat + w_hat_dot@ np.array([[dt]])

    # calc estimate for the non-linear function  
    f_hat = w_hat.T @ h
    f_hat_vec[step] = f_hat

    # calculate control signal
    u = (f_hat + x_d_dotdot + K.T @ E)/c_hat
    u_vec[step] = u 

    #calc c estimate
    value = E.T @ P @ np.array([[0],[1]])*u
    if value > 0:
        c_hat_dot = (1/eta)*value
    if c_hat > c_min : 
        c_hat_dot = (1/eta)*value
    if c_hat <= c_min: 
        c_hat_dot = 1/eta
    c_hat = c_hat + c_hat_dot*dt
    c_hat_vec[step] = c_hat 


    # state propagation
    x_dotdot = -a1*x_dot - a2*x -b2*np.cos(x) + c*u
    x_dot = x_dot + x_dotdot*dt
    x = x + x_dot*dt
    f_vec[step] = -a1*x_dot - a2*x -b2*np.cos(x)
    x_vec[step+1] = x

   

    


x_vec = np.array(x_vec)
x_d_vec = np.array(x_d_vec)
f_hat_vec = np.array(f_hat_vec)
f_vec = np.array(f_vec)
u_vec = np.array(u_vec)
c_hat_vec = np.array(c_hat_vec)


time = np.linspace(0, t, num_steps+1)

plt.plot(time, x_vec[:, 0, 0], label='x')
plt.plot(time, x_d_vec[:, 0, 0], label='x_d')
plt.xlabel('Time (s)')
plt.ylabel('x [m]')
plt.legend()
plt.tight_layout()
plt.show()

#plot error x_d - x
time = np.linspace(0, t, num_steps+1)
plt.plot(time, x_d_vec[:, 0, 0] - x_vec[:, 0, 0], label='e')
plt.xlabel('Time (s)')
plt.ylabel('e [m]')
plt.legend()
plt.tight_layout()
plt.show()

time = np.linspace(0, t, num_steps)
plt.plot(time, u_vec[:, 0, 0], label='u')
plt.xlabel('Time (s)')
plt.ylabel('u')
plt.legend()
plt.tight_layout()
plt.show()

plt.plot(time, f_hat_vec[:, 0, 0], label='f_hat')
plt.plot(time, f_vec[:, 0, 0], label='f')
plt.xlabel('Time (s)')
plt.ylabel('f')
plt.legend()
plt.tight_layout()
plt.show()

c_vec = [c]*len(time)
plt.plot(time, c_hat_vec[:, 0, 0], label='c_hat')
plt.plot(time, c_vec, label='c_hat')
plt.xlabel('Time (s)')
plt.ylabel('c_hat')
plt.legend()

plt.tight_layout()
plt.show()