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
    (100.0, 100.0, CRED, 'vertical'),
    (100.0, 400.0, CGREEN, 'vertical'),
    (500.0, 100.0, CGREEN, 'horizontal'),
    (500.0, 400.0, CRED, 'horizontal')
]

def jet(x):
    """Colour map for drawing particles. This function determines the colour of 
    a particle from its weight.
    Red (127.5, 0, 0) being most likely.
    Blue (0, 0, 127.5) being least likely."""
    r = (x >= 3.0/8.0 and x < 5.0/8.0) * (4.0 * x - 3.0/2.0) + (x >= 5.0/8.0 and x < 7.0/8.0) + (x >= 7.0/8.0) * (-4.0 * x + 9.0/2.0)
    g = (x >= 1.0/8.0 and x < 3.0/8.0) * (4.0 * x - 1.0/2.0) + (x >= 3.0/8.0 and x < 5.0/8.0) + (x >= 5.0/8.0 and x < 7.0/8.0) * (-4.0 * x + 7.0/2.0)
    b = (x < 1.0/8.0) * (4.0 * x + 1.0/2.0) + (x >= 1.0/8.0 and x < 3.0/8.0) + (x >= 3.0/8.0 and x < 5.0/8.0) * (-4.0 * x + 5.0/2.0)

    return (255.0*r, 255.0*g, 255.0*b)

def draw_world(est_pose, particles, world):
    """Visualization.
    This functions draws robots position in the world coordinate system."""
        
    # Constant needed for transforming from world coordinates to screen coordinates (flip the y-axis)
    ymax = world.shape[0]
    
    world[:] = CWHITE # Clear background to white
    
    # Find largest weight
    max_weight = 0
    for p in particles:
        max_weight = max(max_weight, p.getWeight())

    # Draw particles
    for idx, p in enumerate(particles):
        x = int(p.getX())
        y = ymax - int(p.getY())
        colour = jet(p.getWeight() / max_weight)
        cv2.circle(world, (x,y), 2, colour, 2)
        b = (
            x + int(15.0*np.cos(p.getTheta())), 
            y + int(-15.0*np.sin(p.getTheta()))
        )
        cv2.line(world, (x,y), b, colour, 2)

        # DEBUG add weight next to particle
        # cv2.putText(world, "%i: %f" % (idx, p.getWeight()), (x + 10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
        # cv2.putText(world, "%i" % (idx), (x + 10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
    
    for landmark in landmarks:
        lm_pos = (int(landmark[0]), int(ymax-landmark[1]))
        cv2.circle(world, lm_pos, 5, landmark[2], 2)
        cv2.putText(world, landmark[3], lm_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255)
    
    # Draw estimated robot pose
    a = (int(est_pose.getX()), ymax-int(est_pose.getY()))
    b = (
        int(est_pose.getX() + 15.0*np.cos(est_pose.getTheta())), 
        ymax - int(est_pose.getY()) + int(-15.0* np.sin(est_pose.getTheta()))
    )
    cv2.circle(world, a, 5, CMAGENTA, 2)
    cv2.line(world, a, b, CMAGENTA, 2)

def move_towards_target_given_found_landmark(target, found, position):
    """Takes (x, y) for target and found location as well as the current pose of the robot as a Particle.
    Should return velocity and angular_velocity the robot should move in to find target.
    """

    # The vector between two points A and B is B-A = (B.x-A.x, B.y-A.y).
    # The angle between two vectors can be calculated with the dot product or atan2.
    robotToFound = (
        found[0] - position.getX(),
        found[1] - position.getY()
    )
    robotToTarget = (
        target[0] - position.getX(),
        target[1] - position.getY()
    )

    return (
        4.0,
        np.arctan2(robotToTarget[1], robotToTarget[0]) - np.arctan2(robotToFound[1], robotToFound[0])
    )

def move_towards_target(target, position):
    """Takes (x, y) for target location as well as the current pose of the robot as a Particle.
    Should return velocity and angular_velocity the robot should move in to find target.
    """

    # The vector between two points A and B is B-A = (B.x-A.x, B.y-A.y).
    # The angle between two vectors can be calculated with the dot product or atan2.
    robotToTarget = (
        target[0] - position.getX(),
        target[1] - position.getY()
    )
    currentHeading = (
        np.cos(est_pose.getTheta()),
        np.sin(est_pose.getTheta())
    )

    return (
        4.0,
        np.arctan2(robotToTarget[1], robotToTarget[0]) - np.arctan2(currentHeading[1], currentHeading[0])
    )

### Main program ###

# Open windows
WIN_RF1 = "Robot view";
cv2.namedWindow(WIN_RF1);
cv2.moveWindow(WIN_RF1, 50, 50);

# WIN_World = "World view";
# cv2.namedWindow(WIN_World);
# cv2.moveWindow(WIN_World, 500, 50);


# Initialize particles
num_particles = 1000
particles = []
for i in range(num_particles):
    # Random starting points. (x,y) \in [-1000, 1000]^2, theta \in [-pi, pi].
    # p = particle.Particle(2000.0*np.random.ranf() - 1000, 2000.0*np.random.ranf() - 1000, 2.0*np.pi*np.random.ranf() - np.pi, 1.0/num_particles)
    # p = particle.Particle(2000.0*np.random.ranf() - 1000, 2000.0*np.random.ranf() - 1000, np.pi+3.0*np.pi/4.0, 1.0/num_particles)
    p = particle.Particle(np.random.randint(10, 490), np.random.randint(10, 590), 2.0*np.pi*np.random.ranf() - np.pi, 1.0/num_particles)
    particles.append(p)

est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose

# Driving parameters
velocity = 0.0; # cm/sec
angular_velocity = 0.0; # radians/sec
targetLandmark = 0

# Initialize the robot
robot = frido.Robot()

# Allocate space for world map
# The landmarks are 300 vertical cm's apart and 400cm horizontal cm's apart.
# I'm gonna add a buffer on 100cm on either side.
world = np.zeros((500,600,3), dtype=np.uint8)

# Draw map
# draw_world(est_pose, particles, world)

print("Opening and initializing camera")

#cam = camera.Camera(0, 'macbookpro', 10)
cam = camera.Camera(0, 'frindo')
#cam = camera.Camera(0, 'arlo')

while True:
    # Make the robot drive
    # Read odometry, see how far we have moved, and update particles.
    # Or use motor controls to update particles

    # First we will turn in the direction we wish to go.
    # Radions to degree conversion is radions * 100 / pi
    robot.turn(angular_velocity * 100 / np.pi)

    # We drive in that direction (but around obstacles)
    while not robot.canGo(velocity):
        robot.turn(5)
    
    robot.go(velocity)

    # Update the particles, first move then add noise

    # 1. move the particles
    # we need to calculate delta_x and delta_y. We see to page 100 and 101
    for p in particles:
        (delta_x, delta_y) = particle.project_particle(p, velocity, angular_velocity)
        # print("v: %f, w:%f, dx:%f, dy:%f" % (velocity, angular_velocity, delta_x, delta_y))
        particle.move_particle(p, delta_x, delta_y, angular_velocity)

    # We reset velocities
    velocity = 0.0
    angular_velocity = 0.0

    # 2. add noise
    particle.add_uncertainty(particles, 0.2, 0.15)

    # Fetch next frame
    colour, distorted = cam.get_colour()


    # Detect objects
    objectType, measured_distance, measured_angle, colourProb = cam.get_object(colour)
    
    foundLandmark = None
    if objectType != 'none':
        green = colourProb[1] > colourProb[0]
        if (objectType == 'horizontal'):
            if green:
                foundLandmark = 2
            else:
                foundLandmark = 3
        elif (objectType == 'vertical'):
            if green:
                foundLandmark = 1
            else:
                foundLandmark = 0
        else:
            print("Unknown landmark type")
            continue

        print("Measured distance = ", measured_distance)
        print("Measured angle = ", measured_angle)

        # Compute particle weights
        # I would to weight the particles based on the likelihood of the distance measurement to the feature.
        # This is described in 6.6 (page 147-153)

        particle.reweigh_particles_square_error(
            particles,
            landmarks[foundLandmark],
            measured_distance,
            measured_angle
        )

        # Normalise the particle weights
        weight_normaliser = sum(p.getWeight() for p in particles) ** -1
        for p in particles:
            p.setWeight(p.getWeight() * weight_normaliser)

        # # DEBUG
        # cv2.imshow(WIN_World, world);
        # action = cv2.waitKey(0)

        # Resampling
        particles = particle.resample_by_weight(
            particles,
            landmarks[foundLandmark],
            measured_distance,
            measured_angle
        )

        # Draw detected pattern
        cam.draw_object(colour)
    else:
        # No observation - reset weights to uniform distribution
        for p in particles:
            p.setWeight(1.0/num_particles)

    est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose

    if targetLandmark in [0, 1, 2, 3]:
        if foundLandmark == None:
            if targetLandmark == 2:
                (velocity, angular_velocity) = move_towards_target(
                    (landmarks[targetLandmark][0], landmarks[targetLandmark][1]),
                    (landmarks[foundLandmark][0], landmarks[foundLandmark][1]),
                    est_pose
                )
            else:
                # For every target except for landmark 3 we have line of sight and hence it makes more sense to turn
                # until we see the landmark. 
                angular_velocity -= 0.2
        elif foundLandmark != targetLandmark:
            (velocity, angular_velocity) = move_towards_target_given_found_landmark(
                (landmarks[targetLandmark][0], landmarks[targetLandmark][1]),
                (landmarks[foundLandmark][0], landmarks[foundLandmark][1]),
                est_pose
            )
        elif 30.0 < measured_distance:
            velocity += 4.0
            print("Can see landmark %i. Go straight." % (targetLandmark + 1))
        else:
            targetLandmark = targetLandmark + 1
            print("Landmark %i found. Turn towards landmark %i." % (targetLandmark, targetLandmark + 1))

            if targetLandmark == 1:
                angular_velocity -= 1.4 # calculate angle
            elif targetLandmark == 2:
                angular_velocity -= 0.8 # calculate angle
            elif targetLandmark == 3:
                angular_velocity -= 0.8 # calculate angle
    elif targetLandmark == 4:
        print("Finished the race by being within 40 cm of landmark 4.")
        break
    else:
        print("Unknown landmark detected. Exiting.")
        break

    # Draw map
    # draw_world(est_pose, particles, world)
    
    # Show frame
    cv2.imshow(WIN_RF1, colour);

    # Show world
    # cv2.imshow(WIN_World, world);
    

# Close all windows
cv2.destroyAllWindows()