import numpy as np
import random_numbers as rn
import copy
import cv2

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

def project_particle(p, v, w):
    """Given a velocity and an angular velocity calculate the diff in x and y the projection would result in."""
    theta = p.getTheta()
    if w:
        delta_x = - v / w * np.sin(theta) + v / w * np.sin(theta + w)
        delta_y = v / w * np.cos(theta) - v / w * np.cos(theta + w)
    elif theta:
        delta_x = v * np.cos(theta)
        delta_y = v * np.sin(theta)
    else:
        delta_x = v
        delta_y = 0

    return (int(delta_x), int(delta_y))

def reweigh_particles_square_error(particles, landmark, measured_distance, measured_angle):
    # For now I will just compare the deviation squared from the landmark of the particle.
    for idx, p in enumerate(particles):
        (delta_x, delta_y) = project_particle(p, measured_distance, measured_angle)
        new_x = int(p.getX() + delta_x)
        new_y = int(p.getY() + delta_y)
        diff = np.sqrt((new_x - landmark[0]) ** 2 + (new_y - landmark[1]) ** 2)
        p.setError(diff)
        p.setWeight(1 / diff if diff else 1)

        # DEBUG
        # prev_diff = np.sqrt((p.getX() - landmark[0]) ** 2 + np.absolute(p.getY() - landmark[1]) ** 2)
        # print("%i: x:%f y:%f new_x:%f new_y:%f w:%f prior_diff:%f diff:%f" % (idx, p.getX(), p.getY(), new_x, new_y, p.getWeight(), prev_diff, diff))
        # cv2.line(world, (int(p.getX()), 500 - int(p.getY())), (new_x, 500 - new_y), CMAGENTA, 2)

def resample_by_weight(particles_list, landmark, measured_distance, measured_angle):
    new_particle_list = []
    weights = np.array([p.getWeight() for p in particles_list])
    list_idx_choices = np.random.multinomial(len(weights), weights)
    
    for idx, count in enumerate(list_idx_choices):
        for i in range(count):
            # could change orientation
            new_particle_list.append(copy.copy(particles_list[idx]))

    np.random.shuffle(new_particle_list)

    # Add random particles proportional to the mean error rate
    # 300 => roughly 5%
    # 10 => roughly 0.65%
    mean_error = sum(p.getError() for p in particles_list) / len(particles_list)
    num_random_particles = int(np.ceil(len(new_particle_list)/100.0*(0.5 + 0.01*mean_error)))
    print("Mean error: %f, random particles: %i" % (mean_error, num_random_particles))
    if mean_error and num_random_particles:
        new_particle_list = new_particle_list[:-num_random_particles]

        for i in range(num_random_particles):
            # Adding random samples wasn't working great.
            # new_particle_list.append(Particle(np.random.randint(10, 490), np.random.randint(10, 590), 2.0*np.pi*np.random.ranf() - np.pi, 1.0/num_random_particles))
            # This will add samples from possible locations on a circle around the landmark
            # Should be better but it's not a big improvement.
            (x, y, theta) = sample_radius_around_landmark(landmark[0], landmark[1], measured_distance, measured_angle)
            new_particle_list.append(Particle(x, y, theta))

    return new_particle_list

def sample_radius_around_landmark(x, y, distance, angle):
    r = np.random.randint(distance - 10, distance + 10)
    theta = 2 * np.pi * np.random.random()

    x1 = r * np.cos(theta) + x
    y1 = r * np.sin(theta) + y
    theta1 = np.arctan2(y-y1, x-x1)

    return (x1, y1, theta1)
