import matplotlib.pyplot as plt

#True_values = [ 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000]
measured_values = [ 30250, 30471, 30906, 30999, 31368, 31978, 32526, 33379, 34698, 36275]
predicted_values = []
kalman_readings = []
gains_a_ = []


x_n = 30000.0
x_previous = 0.0 
x_predict = 0.0


v_n = 50.0
v_previous = 0.0 
v_predict = 0.0


a_n = 0.0
a_previous = 0.0 
a_predict = 0.0

dt = 5.0

a = 0.5
B = 0.4
Y = 0.1

for i in range(1, len(measured_values) + 1):

    # predict
    x_predict = x_n + dt*v_n + 0.5*a_n*dt*dt# since this is a static model
    predicted_values.append(x_predict)
    v_predict = v_n + dt*a_n 
    a_predict = a_n
  
    # Measure
    z = measured_values[i - 1]

    # Update 
    x_n = x_predict + a * (z - x_predict)
    v_n = v_predict + B * ((z - x_predict) / dt)
    a_n = a_predict + Y * ((z - x_predict) / (0.5*dt*dt))
    kalman_readings.append(x_n)

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