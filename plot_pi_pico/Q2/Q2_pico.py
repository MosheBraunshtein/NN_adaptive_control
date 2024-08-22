from machine import Pin, Timer
import utime
from ulab import numpy as np
import sys

def calc_h(state):
    C = [0, 2, 4, 6, 8]
    b = 0.5
    h = np.zeros((5,1))
    i=0 
    for c in C: 
        hi = np.exp(-((state-c)**2)/(2*(b**2)))
        h[i,0] = hi
        i += 1
    return h

#log
state_log = open("state_log","w")
state_d_log = open("state_d_log","w")
u_log = open("u","w")
# system parameters
A = B = D = 1
m = 5 
F = 1
a1 = A/m
a2 = - B/m
b2 = -F/m
c = D/m
# start condition
w_hat = np.array([[0.1], [0.2], [0.3], [0.4], [0.5]])
x = x_dot = x_dotdot = np.array([[0]])
x_d = np.array([[1]])
x_d_dot = x_d_dotdot = np.array([[0]])
u = 0
# reference signal parameters x_d_dot = -a*x_d + b*sin(w*dt)
a = 1
b = 3
w = 5
# controller parameters
gamma = 0.1
kp = 15
kd = 1
K = np.array([[kp],[kd]])
q11 , q12, q21, q22 = 10 , 1, 1, 5 
p12 = q11 / (2 * kp)
p22 = q22 / (2 * kd)
p11 = (q12+q21) / (2 * kp) + kd / (kp * p12) 
P = np.array([[p11,p12],[p12,p22]])

tf = 15 # end time

# process timer parameters
dt_process = 0.001 # 1000 Hz
process_num_steps =int(tf/dt_process)

# control timer parameters
dt_controller = 0.005 # 200 Hz
controller_num_steps = int(tf/dt_controller)

step_process = 0
def process_timer_handler(timer):
    global step_process, x, x_dot, x_dotdot, x_d, x_d_dot, x_d_dotdot, state_log, state_d_log
    
    if step_process >= process_num_steps:
        return
    
    x_dotdot = -a1 * x_dot - a2 * x - b2 * np.cos(x) + c * u
    x_dot = x_dot + x_dotdot * dt_process
    x = x + x_dot * dt_process
    
    t = step_process*dt_process
    x_d_dot = b*np.sin(w*t) - a*x_d
    x_d_dotdot = -a*x_d_dot + b*w*np.cos(w*t)
    x_d = x_d + x_d_dot*dt_process
    
    state_log.write(str(x[0,0])+ "\n")
    state_d_log.write(str(x_d[0,0])+ "\n")
    step_process += 1

step_control = 0   
def control_timer_handler(timer):
    global step_control, w_hat, u_log, u

    if step_control >= controller_num_steps:
        return
    
    h = calc_h(x)
    E = np.array([[x_d[0, 0] - x[0, 0]], [x_d_dot[0, 0] - x_dot[0, 0]]])
    w_hat_dot = -np.dot(np.array([[gamma]]),(np.dot(np.dot(E.T,P),np.array([[0], [1]])))) * h
    w_hat = w_hat + np.dot(w_hat_dot,np.array([[dt_controller]]))
    
    f_hat = np.dot(w_hat.T,h)
    
    u = (1 / c) * (f_hat + x_d_dotdot + a1 * x_dot + a2 * x + np.dot(K.T,E))
    
    u_log.write(str(u[0,0])+ "\n")
    step_control += 1
    
# controller
controller_timer = Timer()
controller_timer.init(freq=int(1/dt_controller), mode=Timer.PERIODIC, callback=control_timer_handler)

# process
process_timer = Timer()
process_timer.init(freq=int(1/dt_process), mode=Timer.PERIODIC, callback=process_timer_handler)

# Main loop
try:
    while True:
        if(step_control >= controller_num_steps and step_process >= process_num_steps):
            controller_timer.deinit()
            process_timer.deinit()
            u_log.close()
            state_log.close()
            state_d_log.close()
            break
        utime.sleep(1)
    print("done")
except KeyboardInterrupt:
    # Clean up and stop the timer on interrupt
    controller_timer.deinit()
    controller_timer.deinit()
    u_log.close()
    state_log.close()
    state_d_log.close()
    print("Timer stopped")

