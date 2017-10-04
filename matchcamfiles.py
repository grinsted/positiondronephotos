# -*- coding: utf-8 -*-
"""
Created on Tue Oct  3 09:43:16 2017

@author: aslak
"""



import glob
import numpy as np
import pandas 
from scipy.optimize import minimize
from PIL import Image, ExifTags
from dateutil.parser import parse

import matplotlib.pyplot as plt



def getExifTimes(jpgfiles):
    
    jpgtimes = np.zeros([len(jpgfiles), 1], dtype='datetime64[ms]')
    tagDateTimeOriginal = 36867 #DateTimeOriginal  <- unfortunately only 1sec resolution!
    for idx,filename in enumerate(jpgfiles):
        with Image.open(filename) as img:
            jpgtimes[idx] = np.datetime64(parse(img._getexif()[tagDateTimeOriginal]))
            #exif = { ExifTags.TAGS[k]: v for k, v in img._getexif().items() if k in ExifTags.TAGS }
            #jpgtimes[idx] = np.datetime64(parse(exif["DateTimeOriginal"]))
    
    return jpgtimes

def matchtocam(jpgtimes,camtimes):
    constoffset = camtimes[0]
    camtimes = camtimes - constoffset
    costtimes = np.sort(camtimes)
    dt = np.diff(costtimes)
    costtimes = np.interp(np.arange(-0.5,len(costtimes),.5),np.arange(len(costtimes)),costtimes,left=costtimes[0]-np.max(dt),right=costtimes[-1]+np.max(dt))
    cost = costtimes * 0
    cost[2:-1:2] = dt/2
    cost[0] = np.max(dt)
    cost[-1] = np.max(dt)
    
    costfun = lambda offset: np.mean((np.interp(jpgtimes+offset,costtimes,cost,left=cost[0],right=cost[-1]))**2.0)
    #bounds = [np.concatenate((camtimes[0]-jpgtimes[-1],camtimes[-1]-jpgtimes[0]))]
    
    # Search for which cam corresponds to jpgtime0 ... 
    bestcost = np.Inf
    bestoffset = 0
    for camtime in camtimes:
        curoffset = camtime - jpgtimes[-5]
        curcost = costfun(curoffset)
        if curcost<bestcost:
            bestoffset = curoffset
            bestcost = curcost
    # then optimize
    offset = minimize(costfun, bestoffset)
    
    t = jpgtimes + offset.x[0]
    camix = np.interp(t,camtimes,np.arange(len(camtimes)),left = 0, right=len(camtimes)-1)
    camix = np.round(camix).astype(np.int64)
    # todo: add logic for what to do if two images are the same.
    return camix




if __name__ == "__main__":

    folder = r"D:/drone/EGRIP 2017/020817 D2C1/" 

    jpgfiles = glob.glob(folder + r"D2/*.JPG")
    
    import parselog
    
    log = parselog.parselogfile(folder + '2017-08-02 10-51-12.log')
    camtimes = log['CAM']['GPSTime'].values
    jpgtimes = getExifTimes(jpgfiles)
    jpgtimes = (jpgtimes-jpgtimes[0]).astype(np.int64) * 1.0
    
    camix = matchtocam(jpgtimes,camtimes)
    
    jpgcams = log['CAM'].iloc(camix)[:].copy()
    
    lagtime = 150 #ms (should be greater than zero... this is the shutter delay)
    

    #TODO: get lat,long,alt etc from EKF1
    useEKF1 = True
    if useEKF1:
        datasource = 'EKF1' #ATT -OR- EKF1
        mappings = {'Lat': 'PN', 'Lng': 'PE', 'Alt': 'PD'}
        jt = jpgcams['GPSTime']+lagtime-log['gpstimeoffset']

        for jpgkey, ekfkey in mappings.items():
            y=log['GPS'][jpgkey]
            x=log['GPS'][ekfkey]
            p=np.polyfit(x,y,1) #infer linear mapping...
            x=np.interp(jt,log['EKF1']['TimeMS'],log['EKF1'][ekfkey])
            jpgcams[jpgkey] = np.polyval(p,x)
    else:
        jt=jpgcams['GPSTime']+lagtime
        jpgcams['Lat'] = np.interp(jt,log['GPS']['TimeMS'],log['GPS']['Lat'])
        jpgcams['Lng'] = np.interp(jt,log['GPS']['TimeMS'],log['GPS']['Lng'])
        jpgcams['Alt'] = np.interp(jt,log['GPS']['TimeMS'],log['GPS']['Alt'])
        datasource = 'ATT' #ATT -OR- EKF1

    jt = jpgcams['GPSTime']+lagtime-log['gpstimeoffset']
    #todo: protect against circular overflows - np.unwrap...
    jpgcams['Roll'] = np.interp(jt,log[datasource]['TimeMS'],log[datasource]['Roll'])
    jpgcams['Pitch'] = np.interp(jt,log[datasource]['TimeMS'],log[datasource]['Pitch'])
    jpgcams['Yaw'] = np.interp(jt,log[datasource]['TimeMS'],log[datasource]['Yaw'])

    
    
    
    
    