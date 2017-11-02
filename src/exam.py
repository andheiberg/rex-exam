import cv2
import camera
import frido
import numpy as np
import particle

# Some colors constants
CRED = (0, 0, 255)
CGREEN = (0, 255, 0)
CBLUE = (255, 0, 0)
CCYAN = (255, 255, 0)
CYELLOW = (0, 255, 255)
CMAGENTA = (255, 0, 255)
CWHITE = (255, 255, 255)
CBLACK = (0, 0, 0)

# Landmarks.
# The robot knows the position of 4 landmarks.
# Their coordinates are in cm.
# Their colour is the third value
# Their orientation is their fourth value
landmarks = [
    (0.0, 0.0, CRED, 'vertical'),
    (0.0, 300.0, CGREEN, 'vertical'),
    (400.0, 0.0, CBLUE, 'horizontal'),
    (400.0, 300.0, CYELLOW, 'horizontal')
]

def jet(x):
    """Colour map for drawing particles. This function determines the colour of 
    a particle from its weight."""
    r = (x >= 3.0/8.0 and x < 5.0/8.0) * (4.0 * x - 3.0/2.0) + (x >= 5.0/8.0 and x < 7.0/8.0) + (x >= 7.0/8.0) * (-4.0 * x + 9.0/2.0)
    g = (x >= 1.0/8.0 and x < 3.0/8.0) * (4.0 * x - 1.0/2.0) + (x >= 3.0/8.0 and x < 5.0/8.0) + (x >= 5.0/8.0 and x < 7.0/8.0) * (-4.0 * x + 7.0/2.0)
    b = (x < 1.0/8.0) * (4.0 * x + 1.0/2.0) + (x >= 1.0/8.0 and x < 3.0/8.0) + (x >= 3.0/8.0 and x < 5.0/8.0) * (-4.0 * x + 5.0/2.0)

    return (255.0*r, 255.0*g, 255.0*b)

def draw_world(est_pose, particles, world):
    """Visualization.
    This functions draws robots position in the world coordinate system."""
    
    
    # Fix the origin of the coordinate system
    offsetX = 100;
    offsetY = 100;
    
    # Constant needed for transforming from world coordinates to screen coordinates (flip the y-axis)
    ymax = world.shape[0]
    
    world[:] = CWHITE # Clear background to white
    
    # Find largest weight
    max_weight = 0
    for particle in particles:
        max_weight = max(max_weight, particle.getWeight())

    # Draw particles
    for particle in particles:
        x = int(particle.getX()) + offsetX
        y = ymax - (int(particle.getY()) + offsetY)
        colour = jet(particle.getWeight() / max_weight)
        cv2.circle(world, (x,y), 2, colour, 2)
        b = (
            x + int(15.0*np.cos(particle.getTheta())), 
            y + int(-15.0*np.sin(particle.getTheta()))
        )
        cv2.line(world, (x,y), b, colour, 2)
    
    for landmark in landmarks:
        lm_pos = (int(landmark[0]+offsetX), int(ymax-(landmark[1]+offsetY)))
        cv2.circle(world, lm_pos, 5, landmark[2], 2)
        cv2.putText(world, landmark[3], lm_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
    
    # Draw estimated robot pose
    a = (int(est_pose.getX())+offsetX, ymax-(int(est_pose.getY())+offsetY))
    b = (
        int(est_pose.getX() + 15.0*np.cos(est_pose.getTheta()))+offsetX, 
        ymax - (int(est_pose.getY()) + offsetX) + int(-15.0* np.sin(est_pose.getTheta()))
    )
    cv2.circle(world, a, 5, CMAGENTA, 2)
    cv2.line(world, a, b, CMAGENTA, 2)


### Main program ###

# Open windows
WIN_RF1 = "Robot view";
cv2.namedWindow(WIN_RF1);
cv2.moveWindow(WIN_RF1, 50, 50);

WIN_World = "World view";
cv2.namedWindow(WIN_World);
cv2.moveWindow(WIN_World, 500, 50);



# Initialize particles
num_particles = 100
particles = []
for i in range(num_particles):
    # Random starting points. (x,y) \in [-1000, 1000]^2, theta \in [-pi, pi].
    # p = particle.Particle(2000.0*np.random.ranf() - 1000, 2000.0*np.random.ranf() - 1000, 2.0*np.pi*np.random.ranf() - np.pi, 1.0/num_particles)
    # p = particle.Particle(2000.0*np.random.ranf() - 1000, 2000.0*np.random.ranf() - 1000, np.pi+3.0*np.pi/4.0, 1.0/num_particles)
    p = particle.Particle(np.random.randint(-90, 490), np.random.randint(-90, 390), 2.0*np.pi*np.random.ranf() - np.pi, 1.0/num_particles)
    particles.append(p)

est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose

# Driving parameters
velocity = 0.0; # cm/sec
angular_velocity = 0.0; # radians/sec

# Initialize the robot
# robot = frido.Robot()

# Allocate space for world map
# The landmarks are 300 vertical cm's apart and 400cm horizontal cm's apart.
# I'm gonna add a buffer on 100cm on either side.
world = np.zeros((500,600,3), dtype=np.uint8)

# Draw map
draw_world(est_pose, particles, world)

print("Opening and initializing camera")

cam = camera.Camera(0, 'macbookpro', 10)
#cam = camera.Camera(0, 'frindo')
#cam = camera.Camera(0, 'arlo')

while True:

    # Move the robot according to user input (for testing)
    # action = cv2.waitKey(10)
    action = cv2.waitKey(0)
    
    if action == ord('w'): # Forward
        velocity += 4.0;
    elif action == ord('x'): # Backwards
        velocity -= 4.0;
    elif action == ord('s'): # Stop
        velocity = 0.0;
        angular_velocity = 0.0;
    elif action == ord('a'): # Left
        angular_velocity += 0.2;
    elif action == ord('d'): # Right
        angular_velocity -= 0.2;
    elif action == ord('q'): # Quit
        break

    # Make the robot drive
    # Read odometry, see how far we have moved, and update particles.
    # Or use motor controls to update particles

    # First we will turn in the direction we wish to go.
    # Radions to degree conversion is radions * 100 / pi
    # robot.turn(angular_velocity * 100 / np.pi)

    # We drive in that direction (or away from the direction)
    # robot.go(velocity)

    # Update the particles, first move then add noise

    # 1. move the particles
    # we need to calculate delta_x and delta_y. We see to page 100 and 101
    for p in particles:
        v = velocity
        w = angular_velocity
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

        particle.move_particle(p, delta_x, delta_y, angular_velocity)

    # 2. add noise
    particle.add_uncertainty(particles, 1, 0.1)

    # We reset velocities
    velocity = 0.0
    angular_velocity = 0.0

    # Fetch next frame
    colour, distorted = cam.get_colour()    
    
    
    # Detect objects
    objectType, measured_distance, measured_angle, colourProb = cam.get_object(colour)
    if objectType != 'none':
        print("Object type = ", objectType)
        print("Measured distance = ", measured_distance)
        print("Measured angle = ", measured_angle)
        print("Colour probabilities = ", colourProb)

        if (objectType == 'horizontal'):
            print("Landmark is horizontal")
        elif (objectType == 'vertical'):
            print("Landmark is vertical")
        else:
            print("Unknown landmark type")
            continue

        # @TODO: Compute particle weights
        # I would to weight the particles based on the likelihood of the distance measurement to the feature.
        # This is described in 6.6 (page 147-153)

        # @TODP: Resampling

        # Draw detected pattern
        cam.draw_object(colour)
    else:
        # No observation - reset weights to uniform distribution
        for p in particles:
            p.setWeight(1.0/num_particles)

    
    est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose

    # Draw map
    draw_world(est_pose, particles, world)
    
    # Show frame
    cv2.imshow(WIN_RF1, colour);

    # Show world
    cv2.imshow(WIN_World, world);
    

# Close all windows
cv2.destroyAllWindows()