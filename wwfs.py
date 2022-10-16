#!/usr/bin/env python
#
# wwfs.py
#
"""WWFS - "World's Worst Flight Simulator" or "Wonderful Way to Fake Stratux"

This program implements a sender of the GDL-90 data format used by Stratux, with
the parameters for the ownship/pilot craft being received through user input (currently
just CLI with keyboard listener, but rudimentary Tkinter UI might be added in a future version)

Copyright (c) 2013 by Eric Dey; All rights reserved
Copyright (c) 2021 by Shane Lenagh
"""

import time
import socket
import gdl90.encoder
import math
import os
import argparse

import keyboard
import threading
import sys

lat_input = 0
lon_input = 0
vinput = 0
lat_lon_diff = .005
vert_diff = 100
last_input_tmst = 0
heading_input = 360
const_own_velocity = 0
const_traf_velocity = 0
const_accel = 10
own_ground = 0
traffic_ground = 0

def process_input():    
    global lat_input
    global lon_input
    global vinput
    global vert_diff
    global lat_lon_diff
    global last_input_tmst
    global heading_input
    global const_own_velocity
    global const_accel
    global const_traf_velocity
    global own_ground
    global traffic_ground
    
    while True:
        key = keyboard.read_key()
        if key in ("j", "l", "i", "k", "n", ".", "u", "o", "a", "d", "=", "-", "q", "f", "s", "w", "r", "g", "t"):
            print(f"got key {key}")
            if key == "j":
                lon_input -= lat_lon_diff
                heading_input = 270
            elif key == "l":
                lon_input += lat_lon_diff
                heading_input = 90
            elif key == "i":
                lat_input += lat_lon_diff
                heading_input = 360
            elif key == "k":
                lat_input -= lat_lon_diff    
                heading_input = 180               
            elif key == "n":
                lon_input -= lat_lon_diff
                lat_input -= lat_lon_diff
                heading_input = 225
            elif key == ".":
                lon_input += lat_lon_diff
                lat_input -= lat_lon_diff   
                heading_input = 135                
            elif key == "u":
                lat_input += lat_lon_diff
                lon_input -= lat_lon_diff
                heading_input = 315                
            elif key == "o":
                lon_input += lat_lon_diff
                lat_input += lat_lon_diff
                heading_input = 45   
            elif key == "f":
                const_own_velocity += const_accel
            elif key == "g":
                own_ground = 1 if own_ground == 0 else 0
            elif key == "t":
                traffic_ground = 1 if traffic_ground == 0 else 0
            elif key == "s":
                const_own_velocity -= const_accel
                if const_own_velocity < 0:
                    const_own_velocity = 0
            elif key == "r":
                const_traf_velocity += const_accel
            elif key == "w":
                const_traf_velocity -= const_accel
                if const_traf_velocity < 0:
                    const_traf_velocity = 0                    
            elif key == "a":
                vinput += vert_diff
                print(f"ascending by setting vinput to {vinput}")
            elif key == "d":
                vinput -= vert_diff
                print(f"descending by setting vinput to {vinput}")
            elif key == "=":
                vert_diff *= 10.00
                lat_lon_diff *= 10.00
                const_accel *= 10.00
                print(f"increased lat/lon diff to {lat_lon_diff} and vert diff to {vert_diff}")
            elif key == "-":
                vert_diff /= 10.00
                lat_lon_diff /= 10.00
                const_accel /= 10.00
                print(f"decreased lat/lon diff to {lat_lon_diff} and vert diff to {vert_diff}")
            elif key == "q":
                print("Bye!")
                os._exit(os.X_OK)
            time.sleep(0.15)    # don't hyper-respond to single keypresses

# Default values for options
#DEF_SEND_ADDR="255.255.255.255"
DEF_SEND_ADDR="localhost"
DEF_SEND_PORT=5000

LATLONG_TO_RADIANS = math.pi / 180.0
RADIANS_TO_NM = 180.0 * 60.0 / math.pi

def distance(lat0, lon0, lat1, lon1):
    """compute distance between two points"""
    lat0 *= LATLONG_TO_RADIANS
    lat1 *= LATLONG_TO_RADIANS
    lon0 *= -LATLONG_TO_RADIANS
    lon1 *= -LATLONG_TO_RADIANS
    radians = math.acos(math.sin(lat0)*math.sin(lat1)+math.cos(lat0)*math.cos(lat1)*math.cos(lon0-lon1))
    return(radians*RADIANS_TO_NM)


def distance_short(lat0, lon0, lat1, lon1):
    """compute distance between two points that are close to each other"""
    lat0 *= LATLONG_TO_RADIANS
    lat1 *= LATLONG_TO_RADIANS
    lon0 *= -LATLONG_TO_RADIANS
    lon1 *= -LATLONG_TO_RADIANS
    radians = 2.0*math.asin(math.sqrt((math.sin((lat0-lat1)/2.0))**2 + math.cos(lat0)*math.cos(lat1)*(math.sin((lon0-lon1)/2.0))**2))
    return(radians*RADIANS_TO_NM)


def horizontal_speed(distance, seconds):
    """compute integer speed for a distance traveled in some number of seconds"""
    return(int(3600.0 * distance / seconds))

def parse_traffic_list(traffic):
    parsed_traffic = []
    i = 0x000001
    for t in traffic:
        try:
            tdict = {}
            for item in t.split(","):
                kvp = item.split("=")
                tdict[kvp[0].strip()] = kvp[1].strip()
            parsed_traffic.append((float(tdict["lat"]), float(tdict["long"]), int(tdict["alt"]), int(tdict["hspeed"]), int(tdict["vspeed"]), int(tdict["hdg"]), tdict["callsign"], i))
            i = i+1
        except:
            print(">>>>>>>>>>>>>>>>>>>> Traffic items in args should be CSV of the following (any order): lat={float},long={float},alt={int},hdg={int},hspeed={int},vspeed={int},callsign={sting}")
            print(f">>>>>>>>>>>>>>>>>>>> and you provided: {t}")      
            print("No traffic loaded from CLI as a result, and program is exiting")
            os._exit(1)
    
    return parsed_traffic
    
def main(args):

    global lat_input
    global lon_input
    global vinput
    global heading_input
    global const_own_velocity
    global const_traf_velocity  
    global own_ground
    global traffic_ground
    
    if args.host:
        destAddr = args.host
    elif 'SEND_ADDR' in os.environ.keys():
        destAddr = os.environ['SEND_ADDR']
    else:
        destAddr = DEF_SEND_ADDR

    if args.port:
        destPort = args.port
    else:
        destPort = int(DEF_SEND_PORT)

    print ("Simulating Stratux unit.")
    print ("Transmitting to %s:%s" % (destAddr, destPort))
    
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    packetTotal = 0
    encoder = gdl90.encoder.Encoder()
    
    callSign = args.callsign if args.callsign else 'N12345'
    latCenter = args.lat if args.lat else 41.184510
    longCenter = args.long if args.long else -95.962490
    altitude = args.alt if args.alt else 0
    heading_input =  args.hdg if args.hdg else 0
    groundspeed = args.hspeed if args.hspeed else 0
    verticalspeed = args.vspeed if args.vspeed else 0
    
    # ADS-B towers:
    towers = [
        (29.888890, -97.865556, 'HYI01'),
        (30.463333, -99.736390, 'TX009'),
        (31.203056, -97.051111, 'TX021'),
        (30.586667, -97.682222, 'TX024'),
        (31.598056, -100.160000, 'TX028'),
    ]

    # traffic tuples: lat, long, alt, hspeed, vspeed, hdg, callSign, address
    if args.traffic:
        traffic = parse_traffic_list(args.traffic)
    else:
        traffic = [
            [41.60, -96.00, 3000, const_traf_velocity, 500, 45, 'NBNDT1', 0x000001],
            [41.60, -95.80, 2500, const_traf_velocity, 0, 295, 'NBNDT2', 0x000002],
            [41.18, -95.93, 3200, const_traf_velocity, -100, 285, 'NBNDT3', 0x000003],
            [41.13, -95.30, 2000, const_traf_velocity, 250, 10, 'NBNDT4', 0x000004],
        ]
    
    uptime = 0
    latitudePrev = latCenter
    longitudePrev = longCenter
    altitudePrev = 2500
    headingPrev = 0
    airspeedPrev = 0
    
    while True:
        
        timeStart = time.time()  # mark start time of message burst
        
        # Move ourself, if input supplied
        latitude = latitudePrev + lat_input + (const_own_velocity / (60.0*3600.0)) * math.cos((heading_input / 180 * math.pi))
        longitude = longitudePrev + lon_input + (const_own_velocity / (60.0*3600.0)) * math.sin((heading_input / 180 * math.pi))
        altitude = altitudePrev + vinput 
        heading = heading_input 
        
        lat_input = 0
        lon_input = 0
        vinput = 0
        
        distanceMoved = distance_short(latitudePrev, longitudePrev, latitude, longitude)
        groundspeed = horizontal_speed(distanceMoved, 1.0)
        latitudePrev = latitude
        longitudePrev = longitude
        altitudePrev = altitude
        headingPrev = heading
        
        # Heartbeat Message
        buf = encoder.msgHeartbeat()
        s.sendto(buf, (destAddr, destPort))
        packetTotal += 1

        # Stratux Heartbeat Message
        buf = encoder.msgStratuxHeartbeat()
        s.sendto(buf, (destAddr, destPort))
        packetTotal += 1
        
        # Hiltonsoftware SX Heartbeat Message
        buf = encoder.msgSXHeartbeat(towers=towers)
        s.sendto(buf, (destAddr, destPort))
        packetTotal += 1
        
        # Ownership Report
        buf = encoder.msgOwnershipReport(latitude=latitude, longitude=longitude, altitude=altitude, hVelocity=groundspeed, vVelocity=verticalspeed, trackHeading=heading, callSign=callSign, misc=(9 if own_ground == 0 else 1))
        s.sendto(buf, (destAddr, destPort))
        packetTotal += 1
        
        # Ownership Geometric Altitude
        buf = encoder.msgOwnershipGeometricAltitude(altitude=altitude)
        s.sendto(buf, (destAddr, destPort))
        packetTotal += 1
        
        # Traffic Reports
        if const_traf_velocity > 0: 
            for i in range(len(traffic)):
                traffic[i][3] = const_traf_velocity
                traffic[i][0] = traffic[i][0]+(const_traf_velocity / (60.0*3600.0)) * math.cos((traffic[i][5] / 180 * math.pi))
                traffic[i][1] = traffic[i][1]+(const_traf_velocity / (60.0*3600.0)) * math.sin((traffic[i][5] / 180 * math.pi))
        for t in traffic:
            (tlat, tlong, talt, tspeed, tvspeed, thdg, tcall, taddr) = t
            buf = encoder.msgTrafficReport(latitude=tlat, longitude=tlong, altitude=talt, hVelocity=tspeed, vVelocity=tvspeed, trackHeading=thdg, callSign=tcall, address=taddr, misc=(9 if traffic_ground == 0 else 1))
            s.sendto(buf, (destAddr, destPort))
            if args.verbose:
                print(f"Traffic {tcall} at ({tlat},{tlong}) at alt {talt}, groundspeed {tspeed}, vspeed {tvspeed}, heading {thdg}, ground {traffic_ground}")
            packetTotal += 1
        
        # GPS Time, Custom 101 Message
        buf = encoder.msgGpsTime(count=packetTotal)
        s.sendto(buf, (destAddr, destPort))
        packetTotal += 1
        
        # On-screen status output, every 5 seconds
        uptime += 1
        if uptime % 5 == 0:
            print ("Uptime %d, lat=%3.6f, long=%3.6f, altitude=%d, heading=%d, speed=%d, ground=%d" % (uptime, latitude, longitude, altitude, heading, groundspeed, own_ground))
        
        # Delay for the rest of this second
        if 1.0 - (time.time() - timeStart) > 0:
            time.sleep(1.0 - (time.time() - timeStart))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Simulate user-defined ownship and traffic for Stratux/GDL90 network receivers")
    parser.add_argument('--host', dest='host', type=str, help='Host/subnet to broadcast GDL90 packets to')
    parser.add_argument('--port', dest='port', type=int, help='Port to broadcast GDL90 packets to') 
    parser.add_argument('--lat', dest='lat', type=float, help='Ownship latitude') 
    parser.add_argument('--long', dest='long', type=float, help='Ownship longitude') 
    parser.add_argument('--alt', dest='alt', type=int, help='Ownnship altitude') 
    parser.add_argument('--hspeed', dest='hspeed', type=int, help='Ownship horizontal speed') 
    parser.add_argument('--vspeed', dest='vspeed', type=int, help='Ownship vertical speed') 
    parser.add_argument('--hdg', dest='hdg', type=float, help='Ownship heading (degrees)') 
    parser.add_argument('--callsign', dest='callsign', type=str, help='Ownship callsign') 
    parser.add_argument('traffic', nargs='*', help='Traffic items in CSV arg list, each of the form lat=x,long=y,alt=z,hdg=d,hspeed=vx,vspeed=vz,callsign=abc1234')
    parser.add_argument('--v', dest='verbose', help='Verbose output') 
    pargs = parser.parse_args()
    threading1 = threading.Thread(target=main, args=(pargs,))
    threading1.daemon = True
    threading1.start()
    process_input() 