# REX Exam

The description of the exam assignment can be found in assingment.pdf.

I currently have a anaconda environment locally that I should try and add to this repo sometime.

To get up and running:

```
source activate playground
python src/exam.py
```

As I'm currently still developing it you will need to press w,a,d or x while the map window is active to start map rendering. You can move the robot with the same keys.

I've marked missing bit's with @TODO

Currently I need to reweigh the particles and resample. Then I will need to devise some kinda navigational strategy.

## Camera calibration
The program uses the camera class found in camera.py to access the camera and do the image analysis. The Python version includes some roughly right calibrations in the constructor of the Camera class. You can calibrate your own camera using the camera_calibrator found in Absalon under OpenCV, but we also provide some roughly right calibration files.
