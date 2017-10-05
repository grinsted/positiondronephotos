


# Assign camera positions to images from flight logs

In digital photogrammetry it is important to have the camera position and orientation of each image. Often when you collect images using a drone, then positions are stored in a log file, and a camera is triggered by the flight computer. You have to somehow extract the positions from the flight log, and match it to the image files. You can do that in mission planner in the [geo ref images](http://ardupilot.org/copter/docs/common-geotagging-images-with-mission-planner.html) menu. Unfortunately this is not perfect, and this code is an attempt at doing a better job - atleast for my specific setup. 

* It should work even if some jpgs are missing: Mission planner assumes that there is one photo for each "CAM" message in the log files. But sometimes the camera cannot keep up, and some images are not captured even if the flight computer attempted to trigger one. 
* Can use kalman filtered values instead of raw values (EKF1 messages instead of ATT/CAM/GPS messages). The CAM messages in the log appear to use the raw GPS and orientation readings (the are the same as the last GPS/ATT message). But flight logs also contain Kalman Filtered positions and orientations. The kalman filtered values are probably better. 
* Allows for any shutter delay by interpolating in high frequency observations in the log file. (Mission planner also allows for a shutter delay, but it does not seem to allow as fine grained interpolation). 

Project not done... Still in test phase. 


## the project is so far only focused on my own needs. 

My setup is:
* Drone: A cumulus one from sky-watch  
* Sony QX1 ( manual focus, shutterspeed 1/4000, sony qx-1)  
* log files saved with mission planner
* Exporting csv files for importing be used in AGISOFT photoscan 



