from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import math

def arm_and_takeoff(vehicle, aTargetAltitude):

    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    
    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto otherwise the command after Vehicle.simple_takeoff will execute immediately
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95: #buffer to prevent undershooting and overshooting
            print("Reached target altitude")
            break
        time.sleep(1)

def return_to_launch(vehicle):
    print("Returning to Launch")
    # vehicle.mode = VehicleMode("RTL")
    vehicle.mode = VehicleMode("LAND")
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt <= 0:
            print("Landed")
            break
        time.sleep(1)

def get_location_metres(original_location, dNorth, dEast, dAlt):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle posi
    on.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    newalt = original_location.alt + dAlt

    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon, newalt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon, newalt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def goto(vehicle, dNorth, dEast, dAlt, speed, error_dist = 2):
    print ("go to relative position: {}".format(dEast,dNorth))
    
    currentLocation=vehicle.location.global_relative_frame #get the current location of the drone
    targetLocation=get_location_metres(currentLocation, dNorth, dEast, dAlt) #get the coord of next waypoint relative to the current position
    vehicle.simple_goto(targetLocation, groundspeed = speed) #fly to the target location (next waypoint in relative position) at a given speed

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        remainingDistance=get_distance_metres(vehicle.location.global_frame, targetLocation)
        print ("Distance to target: {}".format (remainingDistance))
        if remainingDistance<=error_dist: #Just below target, in case of undershoot.
            print ("Reached target")
            break
        time.sleep(2) #take a break and wait for the drone to take action for all goto command
        

def goto_all_octahedron_points(vehicle, home_location, coordinateList, changeSpeed_times, errorDist, speed):

    """
    loops through each waypoint dict in coordinate List, pass coordinates and target time into new_absolute_goto function.
    """

    for n in range(0,len(coordinateList)):
    
        #get coordinates
        abs_x = coordinateList[n]['coord'][0]
        abs_y = coordinateList[n]['coord'][1]
        abs_z = coordinateList[n]['coord'][2]
        
        #target arrival time
        targetTime = coordinateList[n]['targetTime']
        print("Target Time: {}".format(targetTime))

        #waypoint
        waypoint = n + 1
        
        #go to target coordinate at speed
        new_absolute_goto(vehicle, home_location, abs_x, abs_y, abs_z, targetTime, speed, changeSpeed_times, waypoint, errorDist)
    
    return None
        
def new_absolute_goto(vehicle, home_location, abs_x, abs_y, abs_z, targetTime, speed, changeSpeed_times, waypoint, errorDist):

    """
    Moves drone to target location (abs_x, abs_y, abs_z) at a certain speed and recalibrate the speed n times
    """
    
    print("Go to absolute position: {},{},{} ".format(abs_x, abs_y, abs_z))
    
    #convert xyz cartesian coordinates into lat-long
    targetLoc = drone_challenge_base_command.get_location_metres(home_location, abs_x, abs_y, abs_z)
    
    for n in range(changeSpeed_times):
        '''update speed n times along each edge'''
        
        #go to target location with updated speed
        vehicle.simple_goto(targetLoc, groundspeed = speed)
        
        #get remaining distance
        remainingDist = drone_challenge_base_command.get_distance_metres(vehicle.location.global_frame, targetLoc)
        
        #break loop when the drone falls within buffer
        if remainingDist <= errorDist:
            break
        
        #update speed required to reach target location by target time
        speed = get_required_speed(remainingDist,targetTime)
        
        time.sleep(1)
        
    print("Reached waypoint {}".format(waypoint))
    time.sleep(1)
            
    return None

def get_required_speed(remainingDist, targetTime):

    """
    returns the speed required for the drone to arrive at its target location on time
    """
    
    print("remaining distance: {} ".format(remainingDist))
    
    #get remaining time
    remainingTime = targetTime - time.time()
    print("time remaining to reach target loc: {} ".format(remainingTime))

    #speed = distance/time
    newSpeed = remainingDist/ remainingTime
    print("Change speed to {} m/s. ".format(newSpeed))
    
    return newSpeed
    