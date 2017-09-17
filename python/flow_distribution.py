# system
import os
import sys
import time
import argparse
import sched
import numpy as np
import math

# lcm
import lcm

# beautifulsoup
import urllib2
from bs4 import BeautifulSoup

# setup
sample_start = 11 # [hours]
sample_duration = 4 # [hours]

station = "KMACAMBR9"
y = 2014
m = 12
d = 12
fname = station + "_" + str(y) + "_" + str(m).zfill(2) + "_" + str(d).zfill(2) + ".txt"
f = open(fname, 'w')

# unscented transform
N = 2
samples = 2*N

# Open wunderground.com url
url = "http://www.wunderground.com/weatherstation/WXDailyHistory.asp?ID=" + station + "&month=" + str(m) + "&day=" + str(d) + "&year=" + str(y) + "&format=1"
page = urllib2.urlopen(url)
soup = BeautifulSoup(page)

text = ""
# loop through list
count = 0
times_all_list = []
speeds_all_list = []
dirs_all_list = []
for i in soup.p.contents:
    if i.string:
        #print i.string.strip()
        line = i.string.strip()
        text += line + "\n"

        if count > 0 and len(i) > 2:
            elements = line.split(',')
            times_all_list.append(elements[0])
            dirs_all_list.append(float(elements[5]))
            speeds_all_list.append(float(elements[6]))

        count += 1
print "Total Measurements: " + str(count)

# write to file
print "Writing: " + fname
f.write(text)
f.close()

# get subset of data
count = 0
speeds_list = []
dirs_list = []
for i in range(len(speeds_all_list)):
    if i > sample_start*60/5 and i < sample_start*60/5 + sample_duration*60/5:
        #print times_all_list[i]
        speeds_list.append(speeds_all_list[i])
        dirs_list.append(dirs_all_list[i])
        count += 1

print "Start Sample: " + times_all_list[0]
print "End Sample: " + times_all_list[-1]
print "Samples: " + str(count)
#print speeds_list
#print dirs_list

# convert speed to metric
speeds_met_list = []
for val in speeds_list:
    speeds_met_list.append(0.447*val)

# transform angle from 0 deg North, CW to 0 deg East, CCW
dirs_rot_list = []
for val in dirs_list:
    dirs_rot_list.append(-(val - 90))

# get unit sine and cosine vectors for angle mean
c_list = []
s_list = []
for val in dirs_rot_list:
    c_list.append(np.cos(math.radians(val)))
    s_list.append(np.sin(math.radians(val)))

c_bar = np.mean(np.matrix(c_list))
s_bar = np.mean(np.matrix(s_list))
dir_mean = -math.degrees(math.atan2(s_bar,c_bar)) + 90

s_matrix = np.matrix(speeds_list)
s_mean = np.mean(s_matrix)

print "Prior Mean Speed [m/s]: " + str(0.447*s_mean)
print "Prior Mean Heading [deg]: " + str(dir_mean)

count = 0
u_list = []
v_list = []
# get x and y velocity components
for i in range(len(speeds_list)):
    u_list.append(speeds_met_list[i]*math.cos(math.radians(dirs_rot_list[i])))
    v_list.append(speeds_met_list[i]*math.sin(math.radians(dirs_rot_list[i])))
    count += 1
#print u_list

# get means and cov
#print "Converting data..."
u_matrix = np.matrix(u_list)
v_matrix = np.matrix(v_list)
u_mean = np.mean(u_matrix)
v_mean = np.mean(v_matrix)
uv_mean = np.matrix([u_mean,v_mean])
uv_cov = np.cov(u_matrix,v_matrix)

print "X and Y Velocity Mean:"
print uv_mean
print "X and Y Velocity Covariance:"
print uv_cov

# sigma points
#print "Getting sigma points..."
# cholesky decomposition
P = np.linalg.cholesky(N*uv_cov)

# sigma points
X = np.empty([samples, 2])
for i in range(N):
    X[i*N,:] = uv_mean + np.transpose(P[:,i])
    X[i*N + 1,:] = uv_mean - np.transpose(P[:,i])
#print X

# transform back to polar coordinates
Y = np.empty([samples,2])
for i in range(samples):
    Y[i,0] = np.linalg.norm(X[i,:])
    Y[i,1] = math.atan2(X[i,1],X[i,0])
#print Y

# get Y with angles in degrees
Ydeg = np.empty([samples,2])
for i in range(samples):
    Ydeg[i,0] = np.linalg.norm(X[i,:])
    Ydeg[i,1] = -np.degrees(math.atan2(X[i,1],X[i,0])) + 90
print "Sigma Points:"
print Ydeg

# find mean
Ybar = []
mean_sum = 0
for i in range(samples):
    mean_sum += Y[i,0]

Ybar.append(1.0/2.0/float(N)*mean_sum)
Ybar.append(-np.degrees( np.arctan2( np.sum(np.sin(Y[:,1])), np.sum(np.cos(Y[:,1]))) ) + 90)

print "Posterior Unscented Transform Mean [m/s, deg N]:"
print str(Ybar)
