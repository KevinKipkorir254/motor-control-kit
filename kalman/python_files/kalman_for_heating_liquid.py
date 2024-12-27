import matplotlib.pyplot as plt

#True_values = [ 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]
measured_values = [ 50.486, 50.963, 51.597, 52.001, 52.518, 53.05, 53.438, 53.858, 54.465, 55.114]
predicted_values = []
kalman_readings = []
gains_a_ = []


x_n = 10.0
x_previous = 0.0
x_predicted = 0.0

p_n = 10000.0
p_previous = 0.0
p_predicted = 0.0

q_noise = 0.15

tool_pn = 0.1 * 0.1




for i in range(1, len(measured_values) + 1):

    # predict
    x_predicted = x_n  # since this is a static model
    p_predicted = p_n + q_noise
    predicted_values.append(x_predicted)
    
    # Measure
    z = measured_values[i - 1]

    # Update 
    K_n = (p_predicted/(p_predicted + tool_pn))
    x_n = x_predicted + K_n * (z - x_predicted)
    kalman_readings.append(x_n)
    
    p_n = (1 - K_n) * p_predicted

# Plot the results
plt.figure(figsize=(10, 6))
#plt.plot(range(1, len(True_values) + 1), True_values, label='True Values', marker='s')
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