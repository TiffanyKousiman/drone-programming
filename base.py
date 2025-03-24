import logging
logging.basicConfig(level=logging.DEBUG)

from dronekit import connect as dkconnect, VehicleMode, LocationGlobalRelative
import util
from math import sqrt
import time

ipAddress  = "127.0.0.1"
print(ipAddress)

udp_port = "14551"

connection_string = "udp:"+ipAddress+":"+udp_port

if connection_string:
    print('Connecting to vehicle on: %s' % connection_string)
    vehicle = dkconnect(connection_string, wait_ready=False)

#command to take off and climb to 10m alt
util.arm_and_takeoff(vehicle, 10)

#record the starting time of flight
start_time = time.time()

#set home location
home_location = vehicle.location.global_relative_frame

#list of dictionary of octahedron points and target arrival time
coordinateList = [
    {'coord': [5*sqrt(2), 0, 5*sqrt(2)], 'targetTime': start_time+10}, #B
    {'coord': [0, 5*sqrt(2), 5*sqrt(2)], 'targetTime': start_time+20}, #C
    {'coord': [-5*sqrt(2), 0, 5*sqrt(2)], 'targetTime': start_time+30}, #D
    {'coord': [0, -5*sqrt(2), 5*sqrt(2)], 'targetTime': start_time+40}, #E
    {'coord': [5*sqrt(2), 0, 5*sqrt(2)], 'targetTime': start_time+50}, #B
    {'coord': [0, 0, 10*sqrt(2)], 'targetTime': start_time+60}, #F
    {'coord': [0, 5*sqrt(2), 5*sqrt(2)], 'targetTime': start_time+70}, #C
    {'coord': [0, 0, 0], 'targetTime': start_time+80}, #A
    {'coord': [-5*sqrt(2), 0, 5*sqrt(2)], 'targetTime': start_time+90}, #D
    {'coord': [0, 0, 10*sqrt(2)], 'targetTime': start_time+100}, #F
    {'coord': [0, -5*sqrt(2), 5*sqrt(2)], 'targetTime':start_time+110}, #E
    {'coord': [0, 0, 0], 'targetTime': start_time+120} #A
    ]
    
util.goto_all_octahedron_points(vehicle,home_location, coordinateList, changeSpeed_times = 10, errorDist = 1.5, speed = 1)

end_time = time.time()  #record the end time
total_time_taken = end_time - start_time  #calculate the time used
print ("time taken = {}".format(total_time_taken))

print("Returning to Launch")
util.return_to_launch(vehicle) #3 command to return to the drone to the launching location

print("Close vehicle object")
vehicle.close()
