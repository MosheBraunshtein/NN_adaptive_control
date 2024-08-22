from machine import Pin, Timer
import utime
from ulab import numpy as np
import sys

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
x = x_dot = x_dotdot = np.array([[0]])
x_d = np.array([[1]])
x_d_dot = x_d_dotdot = np.array([[0]])
u = 0
# reference signal parameters x_d_dot = -a*x_d + b*sin(w*dt)
a = 1
b = 3
w = 5
# controller parameters
kp = 50
kd = 10
ki = 10
integral = 0
prev_error = 0

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
    global step_control, u_log, u, integral, prev_error

    if step_control >= controller_num_steps:
        return
    
    error = x_d-x
    P = kp*error
    
    integral += error*dt_controller
    I = ki*integral
    
    derivative = (error - prev_error) / dt_controller
    D = kd*derivative
    
    prev_error = error
    
    u = P + D + I
    
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