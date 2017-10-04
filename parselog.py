# -*- coding: utf-8 -*-
"""
Created on Tue Oct  3 12:12:00 2017

Parse Mission Planner log files, and extract EKF1,CAM and GPS messages



@author: aslak
"""

import numpy as np
import pandas


def parselogfile(fname):
    ekf1 = np.zeros([700000,13])
    att = np.zeros([700000,6])
    gps = np.zeros([50000,16])
    cam = np.zeros([20000,8])
    
    nekf1 = 0
    ngps = 0
    ncam = 0
    natt = 0 
    timems = 0
    timeoffset = 0
    
    with open(fname) as f:
        for line in f:
            if line.startswith("EKF1"):
                aline = np.fromstring(line[5:], sep=',')
                ekf1[nekf1,:] = aline
                timems = aline[0]
                nekf1 = nekf1 + 1
            elif line.startswith("GPS"):
                aline = np.fromstring(line[4:], sep=',')
                PEND = ekf1[nekf1-1,7:10]
                gps[ngps,:] = np.concatenate((aline,PEND))
                timeoffset = aline[1] - timems #gpstime(ms) - timems
                ngps = ngps + 1
            elif line.startswith("CAM"):
                aline = np.fromstring(line[4:], sep=',')
                cam[ncam,:] = aline
                ncam = ncam + 1
            elif line.startswith("ATT"):
                aline = np.fromstring(line[4:], sep=',')
                att[natt,:] = aline
                natt = natt + 1
    
    
    ekf1=pandas.DataFrame(ekf1[0:nekf1,:], columns="TimeMS,Roll,Pitch,Yaw,VN,VE,VD,PN,PE,PD,GX,GY,GZ".split(","))
    gps=pandas.DataFrame(gps[0:ngps,:], columns="Status,TimeMS,Week,NSats,HDop,Lat,Lng,RelAlt,Alt,Spd,GCrs,VZ,T,PN,PE,PD".split(","))
    cam=pandas.DataFrame(cam[0:ncam,:], columns="GPSTime,GPSWeek,Lat,Lng,Alt,Roll,Pitch,Yaw".split(","))
    att=pandas.DataFrame(att[0:natt,:], columns="TimeMS,Roll,Pitch,Yaw,ErrorRP,ErrorYaw".split(","))
    return {'EKF1': ekf1, 'GPS': gps, 'CAM': cam, 'ATT': att, 'gpstimeoffset': timeoffset}

if __name__ == "__main__":
    fname = r"D:/drone/EGRIP 2017/020817 D2C1/2017-08-02 10-51-12.log"
    v = parselogfile(fname)