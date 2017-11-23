


# Assign camera positions to images from flight logs

In digital photogrammetry it is important to have the camera position and orientation of each image. Often when you collect images using a drone, then positions are stored in a log file, and a camera is triggered by the flight computer. You have to somehow extract the positions from the flight log, and match it to the image files. You can do that in mission planner in the [geo ref images](http://ardupilot.org/copter/docs/common-geotagging-images-with-mission-planner.html) menu. Unfortunately this is not perfect, and this code is an attempt at doing a better job - atleast for my specific setup. 

* It should work even if some jpgs are missing: Mission planner assumes that there is one photo for each "CAM" message in the log files. But sometimes the camera cannot keep up, and some images are not captured even if the flight computer attempted to trigger one. 
* Can use kalman filtered values instead of raw values (EKF1 messages instead of ATT/CAM/GPS messages). The CAM messages in the log appear to use the raw GPS and orientation readings (the are the same as the last GPS/ATT message). But flight logs also contain Kalman Filtered positions and orientations. The kalman filtered values are probably better. 
* Allows for any shutter delay by interpolating in high frequency observations in the log file. (Mission planner also allows for a shutter delay, but it does not seem to allow as fine grained interpolation). 
* There is no way to get high precision time codes from the image files themselves. The filedates, and the exif info only has 1 second precision. So, we figure out the timing by finding the best match between exif-DateTimeOriginal to the gps timings in the CAM message from the log. This works because the triggering rate is not constant. 
* Project not done... Still in test phase. 




## the project is so far only focused on my own needs. 

My setup is:
* Drone: A cumulus one from sky-watch  
* Sony QX1 ( manual focus, shutterspeed 1/4000)  
* log files saved with mission planner
* Exporting csv files for importing be used in AGISOFT photoscan 

One particular problem I have with this setup is that the QX1 only records acquisition time at 1 second resolution. WHY!? -Anyway this project is working around this issue. 

For future reference: The sony qx-1 may be light, have nice lenses, have a good sensor, but it is really falling short in other respects for drone mapping.
Here are some of the drawbacks:
* 1 second resolution of the meta data (what are they thinking!)
* long shutterlag (even in completely manual mode)
* Annoying to setup via wifi. 
* Looses date/time in the cold when battery is changed. 


### Files....

* The starting point for the code is in matchcamfiles.py  - This loads all exif data from a folder of jpgs, and loads the log, and tries to match them accounting for a lag 
* pixelpicker.py is a code to zoom an pick points in an imagefile. It returns a numpy array of coordinates when done
* findgcps.py takes a list of stakes (GCPs), and finds images that might cover one of the gcps. It then opens the pixelpicker to allow manual identification of the GP in the photo.


 