import matplotlib.pyplot as plt
import numpy as np


x_vec = []
with open('x_log.txt', 'r') as file:
    for line in file:
        x_vec.append(float(line.strip()))

x_d_vec = []
with open('x_d_log.txt', 'r') as file:
    for line in file:
        x_d_vec.append(float(line.strip()))

u_vec = []
with open('u_log.txt', 'r') as file:
    for line in file:
        u = float(line.strip())
        u_vec.append(u)

print("u len ",len(u_vec))
print("x len ", len(x_vec))


t_1000Hz = np.linspace(0,15,14999)
plt.plot(t_1000Hz,x_vec,label="x")
plt.plot(t_1000Hz,x_d_vec,label="x_d") 
plt.xlabel('t_1000Hz')
plt.ylabel('x')
plt.title("x VS x_d")
plt.legend()
plt.show()

x_d_vec = np.array(x_d_vec)
x_vec = np.array(x_vec)
error = x_d_vec-x_vec
plt.plot(t_1000Hz,error,label="error")
plt.xlabel('t_1000Hz')
plt.ylabel('error')
plt.title("x_d - x")
plt.legend()
plt.show()

u_vec = u_vec[0:1500]
t_20Hz = np.linspace(0,15,1500)
plt.plot(t_20Hz,u_vec,label="u")
plt.xlabel('t_20Hz')
plt.ylabel('u')
plt.title("control signal")
plt.legend()
plt.show()
