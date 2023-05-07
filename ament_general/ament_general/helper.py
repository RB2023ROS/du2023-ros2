import numpy as np

pi = np.pi

def cos(theta):
    return np.cos(theta)

def sin(theta):
    return np.sin(theta)

def rotation(theta):
    return np.array([
        [cos(theta), -sin(theta)], 
        [sin(theta), cos(theta)]
    ])

