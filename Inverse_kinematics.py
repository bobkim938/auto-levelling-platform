import math

def inverse_kinematics(x, y, z, L, d):
    # Calculate theta1 using the atan2 function
    theta1 = math.atan2(y, x)
    
    # Calculate the position of the wrist point in the xy-plane
    r = math.sqrt(x**2 + y**2) - L*math.cos(theta1)
    z_prime = z - d
    
    # Calculate the distance between the wrist point and the origin of joint1
    D = math.sqrt(r**2 + z_prime**2)
    
    # Calculate the angle between link2 and the projection of link3 onto the xy-plane
    phi = math.atan2(z_prime, r)
    
    # Use the law of cosines to solve for theta3
    cos_theta3 = (L**2 - d**2 - D**2) / (2*d*D)
    sin_theta3 = math.sqrt(1 - cos_theta3**2)
    theta3 = math.atan2(sin_theta3, cos_theta3)
    
    # Calculate the remaining joint angles using the law of sines and cosines
    sin_theta2 = (D * math.sin(theta3)) / L
    cos_theta2 = math.sqrt(1 - sin_theta2**2)
    theta2 = math.atan2(sin_theta2, cos_theta2) + phi
    
    return theta1, theta2, theta3
