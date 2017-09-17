# system
import os
import sys
import time
import argparse
import sched
import pytz
import datetime
import commands

# lcm
import urllib2
from bs4 import BeautifulSoup
import pprint

# pretty print
pp = pprint.PrettyPrinter(indent=4)

# station name
station_str = 'KMACAMBR9'
path_str = '/home/jakeware/Dropbox/MIT/rrg/flow_field/weather_station/'

# globals
scheduler = sched.scheduler(time.time, time.sleep)
url = "http://api.wunderground.com/weatherstation/WXCurrentObXML.asp?ID=KMACAMBR9"

year_str_last = ''
mon_str_last = ''
day_str_last = ''
hour_str_last = ''
min_str_last = ''
sec_str_last = ''

def crawl():
    global url
    
    # Open wunderground.com url
    page = urllib2.urlopen(url)
    soup = BeautifulSoup(page)
    
    #print soup.prettify()
    
    # observation time
    #print soup.body.observation_time_rfc822.name
    time_str = soup.body.observation_time_rfc822.string
    #print time_str
    
    gmt = pytz.timezone('GMT')
    est = pytz.timezone('US/Eastern')
    date = datetime.datetime.strptime(time_str, '%a, %d %b %Y %H:%M:%S GMT')
    date_gmt = gmt.localize(date)
    date_est = date_gmt.astimezone(est)
    date_est = date_est.replace(tzinfo=None)
    
    # parse time
    day_str = str(date_est.day)
    mon_str = str(date_est.month)
    year_str = str(date_est.year)
    hour_str = str(date_est.hour)
    min_str = str(date_est.minute)
    sec_str = str(date_est.second)
#     print day_str
#     print mon_str
#     print year_str
#     print hour_str
#     print min_str
#     print sec_str
    
    # wind speed
    #print soup.body.observation_time_rfc822.name
    spd_str = soup.body.wind_mph.string
    #print spd_str
    
    # wind heading
    #print soup.body.observation_time_rfc822.name
    dir_str = soup.body.wind_degrees.string
    #print dir_str
    
    return {'year':year_str,'month':mon_str,'day':day_str,'hour':hour_str,'minute':min_str,'second':sec_str,'speed':spd_str,'direction':dir_str}
    
def publisher():
    global year_str_last
    global mon_str_last
    global day_str_last
    global hour_str_last
    global min_str_last
    global sec_str_last
    
    dict = crawl()
    
    # check if we have gotten a new observation
    if dict['hour'] != hour_str_last or dict['minute'] != min_str_last or dict['second'] != sec_str_last:
        # create file name
        fname = dict['year'] + '_' + dict['month'] + '_' + dict['day'] + '_' + station_str + '.txt'
        #print fname
        fullpath = path_str + fname
        
        # check if file is open
        try:
            file = open(fullpath, 'a')
        except IOError:
            print "Could not open file!"    
                
        # otherwise, close last file and open new one
        
        # write observation to file
        out_str = dict['year'] + ',' + dict['month'] + ',' + dict['day'] + ',' + dict['hour'] + ',' + dict['minute'] + ',' + dict['second'] + ',' + dict['speed'] + ',' + dict['direction'] + '\n'
        #print out_str
        file.write(out_str)
        
        file.close()
        
        # save off last write values
        year_str_last = dict['year']
        mon_str_last = dict['month']
        day_str_last = dict['day']
        hour_str_last = dict['hour']
        min_str_last = dict['minute']
        sec_str_last = dict['second']

def new_timed_call(calls_per_second, callback, *args, **kw):
    period = 1.0 / calls_per_second
    def reload():
        callback(*args, **kw)
        scheduler.enter(period, 0, reload, ())
    scheduler.enter(period, 0, reload, ())

def main():
    parser = argparse.ArgumentParser(description='Get rapid fire weather underground data from green building and write to file.')
    args = parser.parse_args()
    #print args

    # start scrapy to grab data from wunderground xml file
    new_timed_call(1, publisher)
    scheduler.run()

if __name__ == '__main__':
        main()
