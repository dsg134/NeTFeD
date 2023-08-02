import math
import matplotlib.pyplot as plt
import numpy as np

# Input Parameters
# ----------------

# IR Sensor Array Information
IR_angle = 40 # degrees off of back plate
IR_FOV = 60 # degrees (field of view)

# RGB Camera Information
RGB_angle = 35 # degrees off of back plate
RGB_FOV = 68 # degrees (field of view)

# Distance and Range
distance = 3 # inches between the IR sensor and RGB camera
detection_range = 5 # feet away from midpoint of IR sensor and camera centers

# NOTE: IR_FOV + IR_angle + RGB_angle MUST be less than 180 degrees, otherwise the sensor FOVs won't overlap.

# Choose Which Variable You Want to Sweep
# ---------------------------------------

# Variable Sweep Settings
sweep_variable = 'detection_range' # Choose: IR_angle, RGB_angle, or distance (the other measures are supposed to be known)
step_size = 0.1
min_value = 1
max_value = 10
# NOTE: If you choose IR_angle or RGB_angle, they will be in degrees, while distance will be in inches

# Conversions
IR_angle = math.radians(IR_angle) # radians
IR_FOV = math.radians(IR_FOV) # radians

RGB_angle = math.radians(RGB_angle) # radians
RGB_FOV = math.radians(RGB_FOV) # radians

distance = distance / 12 # feet
# detection_range remains the same (in feet)

# Area Calculations
def estimate_detection_area(IR_angle, IR_FOV, RGB_angle, RGB_FOV, distance, detection_range):
	# The covered area can be limited to an (approximate) hexagonal figure which can be split into 4 triangles
	# 4 regions --> regions A, B, C, and D

	# Area of region A
	opp_angle_A = math.radians(180) - (IR_FOV + IR_angle + RGB_angle)
	length_A1 = (distance / (2 * math.cos(IR_angle))) * (math.sin(IR_FOV) / math.sin(opp_angle_A))
	length_A2 = detection_range - ((distance / 2) * math.tan(RGB_angle))

	area_A = 0.5 * length_A1 * length_A2 * math.sin(math.radians(90) - RGB_angle) # square feet

	# Area of region B
	opp_angle_B = math.radians(180) - (RGB_FOV + IR_angle + RGB_angle)
	length_B1 = (distance / (2 * math.cos(RGB_angle))) * (math.sin(RGB_FOV) / math.sin(opp_angle_B))
	length_B2 = detection_range - ((distance / 2) * math.tan(IR_angle))

	area_B = 0.5 * length_A1 * length_A2 * math.sin(math.radians(90) - IR_angle) # square feet

	# Area of region C
	opp_angle_C = math.radians(180) - (RGB_FOV + IR_angle + RGB_angle)
	omega_angle_C = math.radians(180) - (RGB_FOV + RGB_angle) - math.asin(math.sin(RGB_FOV + RGB_angle) * (distance / (2 * detection_range)))
	z_C1 = (detection_range / math.sin(RGB_FOV + RGB_angle)) * math.sin(omega_angle_C);
	length_C1 = z_C1 - ((distance / (2 * math.cos(RGB_angle))) * (math.sin(IR_angle + RGB_angle) / math.sin(opp_angle_C)))

	rho_C_1 = (detection_range - ((distance / 2) * math.tan(IR_angle))) ** 2
	rho_C_2 = ((distance / (2 * math.cos(RGB_angle))) * (math.sin(RGB_FOV) / math.sin(opp_angle_C))) ** 2
	rho_C_3 = 2 * math.sqrt(rho_C_1) * math.sqrt(rho_C_2) * math.cos(math.radians(90) - IR_angle)
	rho_C = math.sqrt(rho_C_1 + rho_C_2 - rho_C_3)

	gamma_C = math.asin(((detection_range - (0.5 * distance * math.tan(IR_angle))) / rho_C) * math.sin(math.radians(90) - IR_angle))
	zeta_C = math.radians(180) - opp_angle_C - gamma_C

	area_C = 0.5 * rho_C * length_C1 * math.sin(zeta_C)

	# Area of region D
	opp_angle_D = math.radians(180) - (IR_FOV + IR_angle + RGB_angle)
	omega_angle_D = math.radians(180) - (IR_FOV + IR_angle) - math.asin(math.sin(IR_FOV + IR_angle) * (distance / (2 * detection_range)))
	z_D1 = (detection_range / math.sin(IR_FOV + IR_angle)) * math.sin(omega_angle_D);
	length_D1 = z_D1 - ((distance / (2 * math.cos(IR_angle))) * (math.sin(IR_angle + RGB_angle) / math.sin(opp_angle_D)))

	rho_D_1 = (detection_range - ((distance / 2) * math.tan(RGB_angle))) ** 2
	rho_D_2 = ((distance / (2 * math.cos(IR_angle))) * (math.sin(IR_FOV) / math.sin(opp_angle_D))) ** 2
	rho_D_3 = 2 * math.sqrt(rho_D_1) * math.sqrt(rho_D_2) * math.cos(math.radians(90) - RGB_angle)
	rho_D = math.sqrt(rho_D_1 + rho_D_2 - rho_D_3)

	gamma_D = math.asin(((detection_range - (0.5 * distance * math.tan(RGB_angle))) / rho_D) * math.sin(math.radians(90) - RGB_angle))
	zeta_D = math.radians(180) - opp_angle_D - gamma_D

	area_D = 0.5 * rho_D * length_D1 * math.sin(zeta_D)

	# Calculate total area
	total_area_covered = area_A + area_B + area_C + area_D # square feet
	return total_area_covered
	# NOTE: This area is an underestimate of the true area because triangles are used to approximate rounded edges at the end of the range

# Calculate and Display Detectable Area
data_length = math.floor((max_value - min_value) / step_size)
x_data = np.zeros(data_length)
y_data = np.zeros(data_length)

# x-axis values
x_data[0] = min_value
for i in range(1, data_length):
	x_data[i] = x_data[i - 1] + step_size

# y-axis values
if sweep_variable == 'IR_angle':
	for j in range(0, data_length):
		y_data[j] = estimate_detection_area(math.radians(x_data[j]), IR_FOV, RGB_angle, RGB_FOV, distance, detection_range)
elif sweep_variable == 'RGB_angle':
	for j in range(0, data_length):
		y_data[j] = estimate_detection_area(IR_angle, IR_FOV, math.radians(x_data[j]), RGB_FOV, distance, detection_range)
elif sweep_variable == 'distance':
	for j in range(0, data_length):
		y_data[j] = estimate_detection_area(IR_angle, IR_FOV, RGB_angle, RGB_FOV, x_data[j] / 12, detection_range)
elif sweep_variable == 'detection_range':
	for j in range(0, data_length):
		y_data[j] = estimate_detection_area(IR_angle, IR_FOV, RGB_angle, RGB_FOV, distance, x_data[j])

# Plot Results
sweeps = ['IR_angle', 'RGB_angle', 'distance', 'detection_range']
if (sweep_variable in sweeps) and (max_value > min_value) and ((step_size > 0) and (step_size < max_value)):
	plt.plot(x_data, y_data)
	plt.xlabel(sweep_variable)
	plt.ylabel('Area')
	plt.title('Estimated Detection Area')
	plt.grid(True)
	plt.show()
else:
	print('No Sweep Variable Detected')
	print('Area Given Current Input Parameters:', estimate_detection_area(IR_angle, IR_FOV, RGB_angle, RGB_FOV, distance, detection_range), 'Square Feet')
