import urllib2
from bs4 import BeautifulSoup

# station = "KMACAMBR9"
station = "KCAMOUNT27"
start_year = 2014
start_month = 1
start_day = 1
end_year = 2014
end_month = 12
end_day = 31

# Iterate through year, month, and day
for y in range(start_year,end_year + 1):
    for m in range(start_month,end_month + 1):
        for d in range(start_day,end_day + 1):
            fname = station + "_" + str(y) + "_" + str(m).zfill(2) + "_" + str(d).zfill(2) + ".txt"
            f = open(fname, 'w')

            # Check if leap year
            if y%400 == 0:
                leap = True
            elif y%100 == 0:
                leap = False
            elif y%4 == 0:
                leap = True
            else:
                leap = False

            # Check if already gone through month
            if (m == 2 and leap and d > 29):
                continue
            elif (m == 2 and d > 28):
                continue
            elif (m in [4, 6, 9, 10] and d > 30):
                continue

            # Open wunderground.com url
            url = "http://www.wunderground.com/weatherstation/WXDailyHistory.asp?ID=" + station + "&month=" + str(m) + "&day=" + str(d) + "&year=" + str(y) + "&format=1"
            page = urllib2.urlopen(url)
            soup = BeautifulSoup(page)

            text = ""
            # loop through list
            for i in soup.p.contents:
                if i.string:
                    #print i.string.strip()
                    text += i.string.strip() + "\n"
            
            # Write timestamp and temperature to file
            print "writing file " + fname
            f.write(text)
      
# Done getting data! Close file.
f.close()
