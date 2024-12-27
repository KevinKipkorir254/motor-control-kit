import matplotlib.pyplot as plt

True_values = [ 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]
measured_values = [ 996, 994, 1021, 1000, 1002, 1010, 983, 971, 993, 1023]
predicted_values = []
kalman_readings = []
gains_a_ = []


x_n = 0.0
x_previous = 1000.0
x_next = 0.0
a = 0.0


for i in range(1, len(measured_values) + 1):

    # Measure
    z = measured_values[i - 1]

    # Update 
    a = (1/i)
    x_n = x_previous + a * (z - x_previous)
    gains_a_.append(a)
    kalman_readings.append(x_n)

    # predict
    x_next = x_n  # since this is a static model
    predicted_values.append(x_next)

    #time_step
    x_previous = x_next

# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(range(1, len(True_values) + 1), True_values, label='True Values', marker='s')
plt.plot(range(1, len(measured_values) + 1), measured_values, label='Measured Values', marker='o')
plt.plot(range(1, len(kalman_readings) + 1), kalman_readings, label='Kalman Readings', marker='x')
plt.plot(range(1, len(predicted_values) + 1), predicted_values, label='Predicted Values', marker='d')

# Add labels and legend
plt.title("Kalman Filter Analysis")
plt.xlabel("Time Step")
plt.ylabel("Values")
plt.legend()
plt.grid()

# Display the plot
plt.show()