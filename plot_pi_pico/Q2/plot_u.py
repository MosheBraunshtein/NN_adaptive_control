import matplotlib.pyplot as plt

# Step 1: Read data from the file
u_vec = []
with open('u_log.txt', 'r') as file:
    for line in file:
        # Convert each line to a float and add it to the values list
        u_vec.append(float(line.strip()))

# Step 2: Plot the data
plt.plot(u_vec)  # 'marker' adds dots at each data point
plt.xlabel('time')
plt.ylabel('u')

# Step 3: Show the plot
plt.show()
