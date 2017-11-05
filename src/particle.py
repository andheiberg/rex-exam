import numpy as np
import random_numbers as rn
import copy

class Particle(object):
    """Data structure for storing particle information (state and weight)"""
    
    def __init__(self, x = 0.0, y = 0.0, theta = 0.0, weight = 0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight
        self.error = None

    def getX(self):
        return self.x

    def setX(self, val):
        self.x = val

    def getY(self):
        return self.y

    def setY(self, val):
        self.y = val

    def getTheta(self):
        return self.theta

    def setTheta(self, val):
        self.theta = val

    def getWeight(self):
        return self.weight

    def setWeight(self, val):
        self.weight = val

    def getError(self):
        return self.error

    def setError(self, val):
        self.error = val


def estimate_pose(particles_list):
    """Estimate the pose from particles by computing the average position and orientation over all particles. 
    This is not done using the particle weights, but just the sample distribution."""
    x_sum = 0.0; y_sum = 0.0; cos_sum = 0.0; sin_sum = 0.0
     
    for particle in particles_list:
        x_sum += particle.getX()
        y_sum += particle.getY()
        cos_sum += np.cos(particle.getTheta())
        sin_sum += np.sin(particle.getTheta())
        
    flen = len(particles_list)
    if flen !=0:
        x = x_sum / flen
        y = y_sum / flen
        theta = np.arctan2(sin_sum/flen, cos_sum/flen)
    else:
        x = x_sum
        y = y_sum
        theta = 0.0
        
    return Particle(x,y,theta)
     
     
def move_particle(particle, delta_x, delta_y, delta_theta):
    """Move the particle by (delta_x, delta_y, delta_theta)"""
    particle.x += delta_x
    particle.y += delta_y
    particle.theta += delta_theta


def add_uncertainty(particles_list, sigma, sigma_theta):
    """Add some noise to each particle in the list. Sigma and sigma_theta is the noise variances for position and angle noise."""
    for particle in particles_list:
        particle.x += rn.randn(0.0, sigma)
        particle.y += rn.randn(0.0, sigma)
        particle.theta = np.mod(particle.theta + rn.randn(0.0, sigma_theta), 2.0 * np.pi)


def add_uncertainty_von_mises(particles_list, sigma, theta_kappa):
    """Add some noise to each particle in the list. Sigma and theta_kappa is the noise variances for position and angle noise."""
    for particle in particles_list:
        particle.x += rn.randn(0.0, sigma)
        particle.y += rn.randn(0.0, sigma)
        particle.theta = np.mod(rn.rand_von_mises (particle.theta, theta_kappa), 2.0 * np.pi) - np.pi

def resample_by_weight(particles_list):
    new_particle_list = []
    weights = np.array([p.getWeight() for p in particles_list])
    list_idx_choices = np.random.multinomial(len(weights), weights)
    
    for idx, count in enumerate(list_idx_choices):
        for i in range(count):
            # could change orientation
            new_particle_list.append(copy.copy(particles_list[idx]))

    return new_particle_list

