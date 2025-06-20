import numpy as np

def odometry_imu_observation_model_linearized():
	def observation_function_h_v1(mu, u):
		# mu is [x, y, theta]
		# Predict sensor measurements purely based on the state estimate and perhaps physical models
		predicted_imu_theta = mu[2]  # Predicts IMU's theta should match the state's theta
		predicted_imu_omega = u[1]  # If omega from control is assumed to reflect actual omega

		# [x, y, theta, theta, w]
		return np.array([mu[0], mu[1], mu[2], predicted_imu_theta, predicted_imu_omega])


	def observation_function_h_v2(mu, previous_theta, delta_t):
		# Compute omega from theta
		delta_theta = mu[2] - previous_theta
		predicted_imu_omega = delta_theta / delta_t
		
		# Assume imu_theta measurement aligns with theta from state
		predicted_imu_theta = mu[2]  # This would be aligned with an actual IMU measurement for theta

		return np.array([[mu[0]], [mu[1]], [mu[2]], [predicted_imu_theta], [predicted_imu_omega]])


	
	def jacobian_of_h_wrt_state_H_v1():
		# Jacobian with respect to the state; note additional rows for IMU data
		# Here we assume theta from IMU aligns directly with theta from state
		H = np.zeros((5, 3))
		H[0:3, :] = np.eye(3)  # Odometry part affects x, y, theta
		H[3, 2] = 1  # IMU theta part affects state theta
		H[4, 2] = 0  # IMU omega does not depend on state directly in this simple model
		return H

	def jacobian_of_h_wrt_state_H_v2(delta_t):
		# Jacobian with respect to the state; note additional rows for IMU data
		# Here we assume theta from IMU aligns directly with theta from state
		H = np.zeros((5, 3))
		H[0:3, :] = np.eye(3)  # Odometry part affects x, y, theta
		H[3, 2] = 1  # IMU theta part affects state theta
		H[4, 2] = 1 / delta_t  # IMU omega does not depend on state directly in this simple model
		return H

	return observation_function_h_v2, jacobian_of_h_wrt_state_H_v2

def odometry_imu_observation_model_with_acceleration_motion_model_linearized_1():
	def observation_function_h(mu):
		x, y, theta, v, w, ax, ay = mu
		# Return a 1D array instead of a column vector
		return np.array([x, y, theta, theta, w, ax, ay])
	
	def jacobian_of_h_wrt_state_H(mu):
		return np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],    
                    [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],   	     
                    [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],   	     
                    [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],  	     
                    [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],   	     
                    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],   	     
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])   	 
                    

	return observation_function_h, jacobian_of_h_wrt_state_H

def odometry_imu_observation_model_with_acceleration_motion_model_linearized_2():
	def observation_function_h(mu):
		x, y, theta, v_x, v_y, w, ax, ay = mu
		# Return a 1D array instead of a column vector
		return np.array([x, y, theta, theta, w, ax, ay])
	
	def jacobian_of_h_wrt_state_H(mu):
		return np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],       
                    [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],   	     
                    [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],   	     
                    [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],  	         
                    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],   	     
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],   	     
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])   	     
                    

	return observation_function_h, jacobian_of_h_wrt_state_H

def odometry_imu_observation_model_with_acceleration_motion_model_no_input_linearized():
	def observation_function_h(mu):
		x, y, theta, vx, vy, w, ax, ay = mu
		# Return a 1D array instead of a column vector
		return np.array([x, y, theta, theta, w, ax, ay])
	
	def jacobian_of_h_wrt_state_H(mu):
		return np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],   # x = x
                    [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],   	 # y = y
                    [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],   	 # theta = theta
                    [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],  	     # theta = theta_imu
                    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],   	 # omega = omega
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],   	 # a_x = a_x
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])   	 # a_y = a_y
                    
	return observation_function_h, jacobian_of_h_wrt_state_H


def odometry_imu_observation_model_with_acceleration_motion_model_3D():
	def observation_function_h(mu):
		x, y, z, roll, pitch, yaw, v_x, v_y, v_z, w_x, w_y, w_z, a_x, a_y, a_z = mu
		return np.array([[x], [y], [z], [roll], [pitch], [yaw], [roll], [pitch], [yaw], [w_x], [w_y], [w_z], [a_x], [a_y], [a_z]])
	
	return observation_function_h

