#!/usr/bin/python
import serial
import math
import time
import datetime

def getweeknum(weekseconds):
    return math.floor(weekseconds/(7*24*3600))

def getweeksec(weekseconds):
    return weekseconds - getweeknum(weekseconds)*(7*24*3600)

def yearfour(year):
    if year<=80:
        year += 2000
    elif year<1990 and year>80:
        year += 1900
    return year

def isleapyear(year):
    return (yearfour(year)%4==0 and yearfour(year)%100!=0) or yearfour(year)%400==0
                 
def timefromGPS(weeknum,weeksec):
    year = 0
    month = 0
    day = 0
    hour = 0
    minute = 0
    second = 0
    doy = 0
    daypermon = [31,28,31,30,31,30,31,31,30,31,30,31]

    weeknum += getweeknum(weeksec)
    weeksec  = getweeksec(weeksec)
    
    weekmin  = math.floor(weeksec/60.0)
    second   = weeksec - weekmin*60.0
    weekhour = math.floor(weekmin/60)
    minute   = weekmin - weekhour*60
    weekday  = math.floor(weekhour/24)
    hour     = weekhour - weekday*24

    totalday = weekday+weeknum*7
    if totalday<360:
        year = 1980
    else:
        year = 1981
        totalday -= 360
        while True:
            if totalday<365:
                break
            if isleapyear(year): totalday -= 1
            totalday -= 365
            year += 1
    doy = totalday

    if totalday <= daypermon[0]:
        month = 1
    else:
        totalday -= daypermon[0];
        if isleapyear(year): totalday -= 1
        month = 2
        while True:
            if totalday<=daypermon[month-1]:
                break
            else:
                totalday -= daypermon[month-1]
                month += 1
    if month==2 and isleapyear(year): totalday += 1
    day = totalday
    return [year,month,day,hour,minute,second,doy]


def configNovatel(ser):

    # need to change the following lever arm values when mounting in the car
    #'setimutoantoffset -0.2077 1.8782 1.0 0.10 0.10 0.10\r',\
    # 'setinstranslation ant2 x, y, z, std_x, std_y, std_z\r',\
    setupcommands7  = ['unlogall\r',\
                'serialconfig com1 230400 N 8 1 N OFF\r',\
                'ETHCONFIG ETHA AUTO AUTO AUTO AUTO\r',\
                'NTRIPCONFIG ncom1 client v1 106.12.40.121:2201 RTK rtkeasy XSTHF3GLPGXGF5AS\r',\
                'interfacemode ncom1 rtcmv3 novatel off\r',\
                'interfacemode com1 novatel novatel on\r',\
                'alignmentmode automatic\r',\
                'setinstranslation ant1 0.0 0.0 0.0 0.10 0.10 0.10\r',\
                'setinstranslation ant2 0.0 0.0 0.0 0.10 0.10 0.10\r',\
                'setinsrotation rbv -180 0 90\r',\
                #'setinsrotation rbv 90 0 180\r',\
                'log inspvasb ontime 0.1\r',\
                'saveconfig\r']
        
    for cmd in setupcommands7:
        ser.write(cmd.encode())    
 

ser = serial.Serial('com37',230400,parity='N',bytesize=8,stopbits=1,timeout=None) #novatel
fname = './log_data/novatel_CPT7-'
ser.flushInput()

fmode = 'wb'

while True:
    if ser.isOpen(): break

print ('\Port is open now\n')
configNovatel(ser)
ser.flushInput()

# get the time information from #INSPVAXA
##while True:
##    line = ser.readline().decode('utf-8')
##    if line.find('#INSPVAXA', 0, len(line)) >= 0:
##        info = line.split(',')
##        #print(info)
##        gpsweek = int(info[5]);
##        sow     = float(info[6]);
##        #print(gpsweek)
##        #print(sow)
##        startime = timefromGPS(gpsweek,sow)
##        fname += '_%4d%02d%02d_%02d%02d%02d.txt' % (startime[0],startime[1],startime[2],startime[3],startime[4],startime[5])
##        print(fname)
##        break

fname += time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime()) + '.bin'

with open(fname,fmode) as outf:
    while True:
        try:
            line = ser.readline()
            #print (line, end='\r\n')
            #outf.write(line.decode('utf-8'))
            outf.write(bytes(line))  #line.decode('utf-8')

        except:
            #break
            pass

    outf.close()
