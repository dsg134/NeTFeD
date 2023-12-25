import math
import matplotlib.pyplot as plt
import numpy as np

# Input Parameters
# ----------------

# Camera 1 Information
cam_1_angle = 39.44 # degrees off of back plate (relative to the FOV bisector)
cam_1_FOV = 68 # degrees (field of view)

# Camera 2 Information
cam_2_angle = 39.44 # degrees off of back plate (relative to the FOV bisector)
cam_2_FOV = 68 # degrees (field of view)

# Thermal Array Information
thermal_FOV = 60 # degrees (whose bisector is perpendicular to the back plate)

# Distance and Range
distance = 3.05 # inches between the IR sensor and RGB camera
detection_range = 6 # feet away from midpoint of IR sensor and camera centers

# NOTE: cam_1_FOV + cam_1_angle + cam_2_angle MUST be less than 180 degrees, otherwise the sensor FOVs won't overlap.

# Choose Which Variable You Want to Sweep
# ---------------------------------------

# Variable Sweep Settings
sweep_variable = '' # Choose: cam_1_angle, cam_2_angle, or distance (the other measures are supposed to be known)
step_size = 0.1
min_value = 0
max_value = 5
# NOTE: If you choose cam_1_angle or cam_2_angle, they will be in degrees, while distance will be in inches

# Conversions
cam_1_angle = math.radians(cam_1_angle) # radians
cam_1_FOV = math.radians(cam_1_FOV) # radians

cam_2_angle = math.radians(cam_2_angle) # radians
cam_2_FOV = math.radians(cam_2_FOV) # radians

thermal_FOV = math.radians(thermal_FOV) # radians

distance = distance / 12 # feet
# detection_range remains the same (in feet)

# Area Calculations
def estimate_detection_area(cam_1_angle, cam_1_FOV, cam_2_angle, cam_2_FOV, distance, detection_range):
	# The covered area can be limited to an (approximate) hexagonal figure which can be split into 4 triangles
	# 4 regions --> regions A, B, C, and D

	# Area of region A
	opp_angle_A = math.radians(180) - (cam_1_FOV + cam_1_angle + cam_2_angle)
	length_A1 = (distance / (2 * math.cos(cam_1_angle))) * (math.sin(cam_1_FOV) / math.sin(opp_angle_A))
	length_A2 = detection_range - ((distance / 2) * math.tan(cam_2_angle))

	area_A = 0.5 * length_A1 * length_A2 * math.sin(math.radians(90) - cam_2_angle) # square feet

	# Area of region B
	opp_angle_B = math.radians(180) - (cam_2_FOV + cam_1_angle + cam_2_angle)
	length_B1 = (distance / (2 * math.cos(cam_2_angle))) * (math.sin(cam_2_FOV) / math.sin(opp_angle_B))
	length_B2 = detection_range - ((distance / 2) * math.tan(cam_1_angle))

	area_B = 0.5 * length_A1 * length_A2 * math.sin(math.radians(90) - cam_1_angle) # square feet

	# Area of region C
	opp_angle_C = math.radians(180) - (cam_2_FOV + cam_1_angle + cam_2_angle)
	omega_angle_C = math.radians(180) - (cam_2_FOV + cam_2_angle) - math.asin(math.sin(cam_2_FOV + cam_2_angle) * (distance / (2 * detection_range)))
	z_C1 = (detection_range / math.sin(cam_2_FOV + cam_2_angle)) * math.sin(omega_angle_C);
	length_C1 = z_C1 - ((distance / (2 * math.cos(cam_2_angle))) * (math.sin(cam_1_angle + cam_2_angle) / math.sin(opp_angle_C)))

	rho_C_1 = (detection_range - ((distance / 2) * math.tan(cam_1_angle))) ** 2
	rho_C_2 = ((distance / (2 * math.cos(cam_2_angle))) * (math.sin(cam_2_FOV) / math.sin(opp_angle_C))) ** 2
	rho_C_3 = 2 * math.sqrt(rho_C_1) * math.sqrt(rho_C_2) * math.cos(math.radians(90) - cam_1_angle)
	rho_C = math.sqrt(rho_C_1 + rho_C_2 - rho_C_3)

	gamma_C = math.asin(((detection_range - (0.5 * distance * math.tan(cam_1_angle))) / rho_C) * math.sin(math.radians(90) - cam_1_angle))
	zeta_C = math.radians(180) - opp_angle_C - gamma_C

	area_C = 0.5 * rho_C * length_C1 * math.sin(zeta_C)

	# Area of region D
	opp_angle_D = math.radians(180) - (cam_1_FOV + cam_1_angle + cam_2_angle)
	omega_angle_D = math.radians(180) - (cam_1_FOV + cam_1_angle) - math.asin(math.sin(cam_1_FOV + cam_1_angle) * (distance / (2 * detection_range)))
	z_D1 = (detection_range / math.sin(cam_1_FOV + cam_1_angle)) * math.sin(omega_angle_D);
	length_D1 = z_D1 - ((distance / (2 * math.cos(cam_1_angle))) * (math.sin(cam_1_angle + cam_2_angle) / math.sin(opp_angle_D)))

	rho_D_1 = (detection_range - ((distance / 2) * math.tan(cam_2_angle))) ** 2
	rho_D_2 = ((distance / (2 * math.cos(cam_1_angle))) * (math.sin(cam_1_FOV) / math.sin(opp_angle_D))) ** 2
	rho_D_3 = 2 * math.sqrt(rho_D_1) * math.sqrt(rho_D_2) * math.cos(math.radians(90) - cam_2_angle)
	rho_D = math.sqrt(rho_D_1 + rho_D_2 - rho_D_3)

	gamma_D = math.asin(((detection_range - (0.5 * distance * math.tan(cam_2_angle))) / rho_D) * math.sin(math.radians(90) - cam_2_angle))
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
if sweep_variable == 'cam_1_angle':
	for j in range(0, data_length):
		y_data[j] = estimate_detection_area(math.radians(x_data[j]), cam_1_FOV, cam_2_angle, cam_2_FOV, distance, detection_range)
elif sweep_variable == 'cam_2_angle':
	for j in range(0, data_length):
		y_data[j] = estimate_detection_area(cam_1_angle, cam_1_FOV, math.radians(x_data[j]), cam_2_FOV, distance, detection_range)
elif sweep_variable == 'distance':
	for j in range(0, data_length):
		y_data[j] = estimate_detection_area(cam_1_angle, cam_1_FOV, cam_2_angle, cam_2_FOV, x_data[j] / 12, detection_range)
elif sweep_variable == 'detection_range':
	for j in range(0, data_length):
		y_data[j] = estimate_detection_area(cam_1_angle, cam_1_FOV, cam_2_angle, cam_2_FOV, distance, x_data[j])

# Plot Results
sweeps = ['cam_1_angle', 'cam_2_angle', 'distance', 'detection_range']
if (sweep_variable in sweeps) and (max_value > min_value) and ((step_size > 0) and (step_size < max_value)):
	plt.plot(x_data, y_data)
	plt.xlabel(sweep_variable)
	plt.ylabel('Area')
	plt.title('Estimated Detection Area')
	plt.grid(True)
	plt.show()
else:
	print('No sweep variable detected.')
	print('Area given current input parameters:', estimate_detection_area(cam_1_angle, cam_1_FOV, cam_2_angle, cam_2_FOV, distance, detection_range), 'Square Feet')

	# Estimate area used by thermal array
	sector_ratio = math.degrees(thermal_FOV) / 360;
	thermal_area = sector_ratio * math.pi * (detection_range ** 2)
	print("Thermal Array Area:", thermal_area, "Square Feet")

	# if thermal_area > detection_area --> warn user to re-optimize as usable space is being lost
	if (thermal_area > estimate_detection_area(cam_1_angle, cam_1_FOV, cam_2_angle, cam_2_FOV, distance, detection_range)):
		print("Re-optimize, the thermal sensor area is larger than the intersection of the camera areas.")

	# Find the area of low confidence --> the area that is not seen by the thermal array
	low_confidence_area = estimate_detection_area(cam_1_angle, cam_1_FOV, cam_2_angle, cam_2_FOV, distance, detection_range) - thermal_area
	print("Low Confidence Area:", low_confidence_area, "Square Feet")
