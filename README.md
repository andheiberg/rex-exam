# REX Exam

The description of the exam assignment can be found in assingment.pdf.

I currently have a anaconda environment locally that I should try and add to this repo sometime.

To get up and running:

```
source activate playground
python src/exam.py
```

## Description
We have modified the code handed out as part of assignment 5, included parts of our previous assignments and made the necessary modifications.

The program consists of:

Camera.py unmodified
Robot.py unmodified

Frido.py is our extension of Robot.py specific to our needs and our Frido robot. It supports go(cm), canGo(cm) and turn(degrees). It hides away the complexity of calibrating the robot to go straight and turn predictably. It also encompasses calibration of the IR sensors used in canGo(cm).

Particles.py defines the Particle class which stores information about each particle (x, y, theta, weight, error). It also has helped functions for performing operations on particles as well as sampling particles from distributions.

Exam.py is the main program that is run with python exam.py. It implements a pretty simple Monte Carlo Localisation algorithm.

Initially 1000 particles are uniformly distributed across the map. We’ve defined the map to be 600 cm x 500 cm with each landmark positioned 100 cm from the corners.

We then enter a while loop that continues until the course have been completed or an unrecoverable situation has been identified. On each iteration of the while loop the action (velocity and angular velocity, for the first iteration they’re both 0) and a measurement to any visible landmark is incorporated to create a new particle distribution.

First we project each particle according to the action. Then we apply noise (The motion update.).

Then we calculate a weight for each particle based on the distance between the projected particle and the landmark versus the measured distance to the landmark. We also normalise these weights so they sum to 1. (Sensor update)

Now we resample the particles with replacement appropriate to the probability of each particle. This will result in a new particle distribution around the most likely particles. In case the error rate is high we have also implemented replacing a percentage of particle proportional to the error rate with random particles based on the sensor measurement. The particles are taken from a Gaussian distribution around the xxxx of a circle centred in the location of the landmark identified with a radius of the distance to said landmark.

Once the new particles have been calculated we use the measurement and the estimated pose to navigate the race track.

The navigational code keeps track of the targetLandmark and will depending on data availability take the appropriate course of action. Once the robot is within 30 cm’s of the target it will increment the targetLandmark and move on to the next target.

If no landmark has been detected we calculate the turn that is likely to take us in the right direction based on the estimated pose.

If a landmark has been detected that is not the target we calculate the right direction to drive based on our current estimated location and the measured angle to the landmark.

If the target landmark is detected we simply move forward as described above.

For all these actions it holds that the robot will overwrite the action if it’s path is blocked. If that is the case it will try to navigate around the obstacle.

## Camera calibration
The program uses the camera class found in camera.py to access the camera and do the image analysis. The Python version includes some roughly right calibrations in the constructor of the Camera class. You can calibrate your own camera using the camera_calibrator found in Absalon under OpenCV, but we also provide some roughly right calibration files.
