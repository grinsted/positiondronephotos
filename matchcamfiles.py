# -*- coding: utf-8 -*-
"""
Created on Tue Oct  3 09:43:16 2017

@author: aslak
"""



import glob
import numpy as np
import pandas 
from scipy.optimize import minimize
from PIL import Image
from datetime import datetime 
#import matplotlib.pyplot as plt
import os
import parselog

def getImageList(globpattern):
    """Return a dataframe containing a list of images and dates of when the photos were taken."""
    jpgfiles = glob.glob(globpattern)
    filenames = list(map(lambda f: os.path.split(f)[-1],jpgfiles)) 
    df = pandas.DataFrame({'DateTimeOriginal': np.datetime64(unit='ms'), 'Filename': jpgfiles },index=filenames)
    tagDateTimeOriginal = 36867 #DateTimeOriginal  <- unfortunately only 1sec resolution!
    for idx,row in df.iterrows():
        with Image.open(row.Filename) as img:
            t=datetime.strptime(img._getexif()[tagDateTimeOriginal],'%Y:%m:%d %H:%M:%S');
            df.set_value(idx,'DateTimeOriginal',np.datetime64(t,unit='ms'))
            #exif = { ExifTags.TAGS[k]: v for k, v in img._getexif().items() if k in ExifTags.TAGS }
            #jpgtimes[idx] = np.datetime64(parse(exif["DateTimeOriginal"]))
    t = df.DateTimeOriginal.values.astype('datetime64[ms]')
    t = (t-t[0]).astype('float')
    df.insert(len(df.columns),'RelTimeMS',t)
    df.sort_values(['DateTimeOriginal', 'Filename'],inplace=True)
    df.index.name = 'File'
    return df

def matchtocam(jpgtimes,camtimes):
    """figure out which rows match which camera times and return index of the matches."""
    
    constoffset = camtimes[0] #the minimizer works better if we are closer to zero. 
    camtimes = camtimes - constoffset
    # construct a costtimes-vs-cost list we can interpolate in to figure out distance to closest cam message
    costtimes = np.sort(camtimes)
    dt = np.diff(costtimes)
    costtimes = np.interp(np.arange(-0.5,len(costtimes),.5),np.arange(len(costtimes)),costtimes,left=costtimes[0]-np.max(dt),right=costtimes[-1]+np.max(dt))
    cost = costtimes * 0
    cost[2:-1:2] = dt/2
    cost[0] = np.max(dt) #special treatment of edges.
    cost[-1] = np.max(dt)
    
    # this function calculates the mean(square(temporaldistance)) to nearest photo.
    costfun = lambda offset: np.mean((np.interp(jpgtimes+offset,costtimes,cost,left=cost[0],right=cost[-1]))**2.0)
    
    # Search for which cam corresponds to jpgtime ... 
    bestcost = np.Inf
    bestoffset = 0
    for camtime in camtimes:
        curoffset = camtime - jpgtimes[0] #todo: pick a less random one
        curcost = costfun(curoffset)
        if curcost<bestcost:
            bestoffset = curoffset
            bestcost = curcost
    # then optimize
    offset = minimize(costfun, bestoffset) #the cost fun has many local minima, so we need to be close to the optimal time
    
    t = jpgtimes + offset.x[0]
    camix = np.interp(t,camtimes,np.arange(len(camtimes)),left = 0, right=len(camtimes)-1)
    camix = np.round(camix).astype(np.int64)
    
    if len(camix) > len(set(camix)):
        # todo: add logic for what to do if two images are the same.
        raise('Photos have not all been assigned to different camera trigger events.')
    return camix




if __name__ == "__main__":

    folder = r"D:/drone/EGRIP 2017/020817 D2C1/C1/" 
    
    print("Folder: {}".format(folder))
    
    globpattern = folder + r"images/*.JPG"
    logfile = glob.glob(folder + r"logs/*.log")[0]
    
    outputfolder = folder + 'georef'

    print("Parsing log...")
    
    log = parselog.parselogfile(logfile)    
    
    print("Extracting EXIF...")
    images = getImageList(globpattern)

    print("Number of images: {}".format(len(images)))
    print("Number of CAM messages in log: {}".format(len(log["CAM"])))
    
    print("Matching photo time to camera log")
    jpgtimes = images.RelTimeMS.values
    camtimes = log['CAM']['GPSTime'].values
    camix = matchtocam(jpgtimes,camtimes)
    print("Matched! index of first photo: {}".format(camix[0]))
    
    jpgcams = log['CAM'].iloc[camix].copy()
    jpgcams.set_index(images.index.values,inplace =True)
    jpgcams.index.name = 'Filename'

    if not os.path.exists(outputfolder):
        os.makedirs(outputfolder)

    jpgcams[['Lng','Lat','Alt','Yaw','Pitch','Roll']].to_csv(outputfolder + 'CamLocations_raw_CAM.txt')
    
    for shutterdelayMS in [400,500,600,700,800]:
#    shutterdelayMS = 900 #ms (should be greater than zero... this is the shutter delay)
        print("Accounting for shutterlag of {} ms".format(shutterdelayMS))
        #TODO: get lat,long,alt etc from EKF1
        useEKF1 = True
        if useEKF1:
            datasource = 'EKF1' #ATT -OR- EKF1
            mappings = {'Lat': 'PN', 'Lng': 'PE', 'Alt': 'PD'}
            jt = jpgcams['GPSTime']+shutterdelayMS-log['gpstimeoffset']
    
            for jpgkey, ekfkey in mappings.items():
                y=log['GPS'][jpgkey]
                x=log['GPS'][ekfkey]
                p=np.polyfit(x,y,1) #infer linear mapping...
                x=np.interp(jt,log['EKF1']['TimeMS'],log['EKF1'][ekfkey])
                jpgcams[jpgkey] = np.polyval(p,x)
        else:
            jt=jpgcams['GPSTime']+shutterdelayMS
            jpgcams['Lat'] = np.interp(jt,log['GPS']['TimeMS'],log['GPS']['Lat'])
            jpgcams['Lng'] = np.interp(jt,log['GPS']['TimeMS'],log['GPS']['Lng'])
            jpgcams['Alt'] = np.interp(jt,log['GPS']['TimeMS'],log['GPS']['Alt'])
            datasource = 'ATT' #ATT -OR- EKF1
    
        jt = jpgcams['GPSTime']+shutterdelayMS-log['gpstimeoffset']
        #todo: protect against circular overflows - quaternion interpolation...
        jpgcams['Roll'] = np.interp(jt,log[datasource]['TimeMS'],log[datasource]['Roll'])
        jpgcams['Pitch'] = np.interp(jt,log[datasource]['TimeMS'],log[datasource]['Pitch'])
        jpgcams['Yaw'] = np.interp(jt,log[datasource]['TimeMS'],log[datasource]['Yaw'])
    
        outputfilename = outputfolder + 'CamLocations_{}lag.txt'.format(shutterdelayMS)
        jpgcams[['Lng','Lat','Alt','Yaw','Pitch','Roll']].to_csv(outputfilename)
    

    print("Done!")
    
    
    
    