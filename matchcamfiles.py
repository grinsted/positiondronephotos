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
from scipy.signal import savgol_filter
from scipy.stats import trim_mean

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


def costfun2(jpgtimes,costtimes):
    totalcost = 0
    curix = 0
    camix = np.zeros([len(jpgtimes),1],dtype=np.int)
    for jix, time in enumerate(jpgtimes):
        cost = np.inf
        for ix in np.arange(curix,len(costtimes)):
            thiscost = abs(time-costtimes[ix])
            if thiscost < cost:
                cost = thiscost
                curix = ix
            else:
                break
        camix[jix] = curix
        curix = curix + 1 
        totalcost = totalcost + cost
    return [totalcost,camix]

def matchtocam(jpgtimes,camtimes):
    """figure out which rows match which camera times and return index of the matches."""
    
#    # deal with limited resol
#    dt = trim_mean(np.diff(jpgtimes),proportiontocut=.1) 
#    window = 2000./dt #apply a ~2second smoothing filter
#    window = np.round(window/2)*2+1 #ensure an odd filter
#    if window>=3:
#        tt = savgol_filter(jpgtimes,window_length=int(window),polyorder=1,mode='nearest')
#        ix = abs(tt-jpgtimes)<1000 
#        jpgtimes[ix]=tt[ix]
        
        
#    constoffset = camtimes[0] #the minimizer works better if we are closer to zero. 
#    camtimes = camtimes - constoffset
#    # construct a costtimes-vs-cost list we can interpolate in to figure out distance to closest cam message
#    costtimes = np.sort(camtimes)
#    dt = np.diff(costtimes)
#    costtimes = np.interp(np.arange(-0.5,len(costtimes),.5),np.arange(len(costtimes)),costtimes,left=costtimes[0]-np.max(dt),right=costtimes[-1]+np.max(dt))
#    cost = costtimes * 0
#    cost[2:-1:2] = dt/2
#    cost[0] = np.max(dt) #special treatment of edges.
#    cost[-1] = np.max(dt)
#    
#    # this function calculates the mean(square(temporaldistance)) to nearest photo.
#    costfun = lambda offset: np.mean((np.interp(jpgtimes+offset,costtimes,cost,left=cost[0],right=cost[-1]))**2.0)

    constoffset = camtimes[0] #the minimizer works better if we are closer to zero. 
    camtimes = camtimes - constoffset

    
    # this function calculates the mean(square(temporaldistance)) to nearest photo.
    costfun = lambda offset: costfun2(jpgtimes+offset,camtimes)[0]
    
    # Search for which cam corresponds to jpgtime ... 
    bestcost = np.Inf
    bestoffset = 0
    for camtime in camtimes:
        for dt in np.arange(-1000,1001,250):
            curoffset = camtime - jpgtimes[0] +dt #todo: pick a less random one
            curcost = costfun(curoffset)
            if curcost < bestcost:
                bestoffset = curoffset
                bestcost = curcost
    # then optimize
    offset = minimize(costfun, bestoffset) #the cost fun has many local minima, so we need to be close to the optimal time
    
    t = jpgtimes + offset.x[0]
    
#    plt.plot(t[1:],np.diff(t))
#    plt.plot(camtimes[1:],np.diff(camtimes))
    
    camix = costfun2(t,costtimes)[1]
#    camix = np.interp(t,camtimes,np.arange(len(camtimes)),left = 0, right=len(camtimes)-1)
#    for ii in range(1,len(camix)-1):
#        dcamback = camix[ii]-camix[ii-1]
#        dcamforward = camix[ii+1]-camix[ii]
#        if (dcamforward == 0) & (dcamback>1):
#            camix[ii]=camix[ii]-1
#        if dcamback<=0:
#            camix[ii+1]=camix[ii]+1
    
#    
#    if len(camix) > len(set(camix)):
#        # todo: add logic for what to do if two images are the same.
#        raise('Photos have not all been assigned to different camera trigger events.')
    camix = np.concatenate(camix).tolist()    
    return camix




if __name__ == "__main__":

    folder = r"D:/drone/EGRIP 2017/2017-07-25 HC forest/" 
    
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
        #todo: protect against circular overflows - quaternion interpolation... (Gimbal lock extremely unlikely though)
        jpgcams['Roll'] = np.interp(jt,log[datasource]['TimeMS'],log[datasource]['Roll'])
        jpgcams['Pitch'] = np.interp(jt,log[datasource]['TimeMS'],log[datasource]['Pitch'])
        log[datasource]['Yaw'] = np.unwrap(log[datasource]['Yaw']*np.pi/180)*180/np.pi
        jpgcams['Yaw'] = np.interp(jt,log[datasource]['TimeMS'],log[datasource]['Yaw']) % 360
    
        outputfilename = outputfolder + 'CamLocations_{}lag.txt'.format(shutterdelayMS)
        jpgcams[['Lng','Lat','Alt','Yaw','Pitch','Roll']].to_csv(outputfilename)
    
    print("Done!")
    
    
    
    