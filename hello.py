# Import DroneKit-Python
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import math
import time

pause_script = False

# Connect to the Vehicle.
#connection_string = '/dev/ttyUSB0'
connection_string = 'tcp:127.0.0.1:5763'

print "Connecting to vehicle on: %s " % connection_string
#vehicle = connect('/dev/ttyUSB0', wait_ready=True, heartbeat_timeout=15, status_printer=status_printer)

vehicle = connect(
    connection_string, baud=57600, wait_ready=True, heartbeat_timeout=15)

#vehicle = connect('/dev/ttyUSB0', baud=57600, wait_ready=True, heartbeat_timeout=15)


def print_status():
    print "Autopilot Firmware version: %s" % vehicle.version
    print "Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp
    print "Global Location: %s" % vehicle.location.global_frame
    print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
    print "Local Location: %s" % vehicle.location.local_frame  #NED
    print "Attitude: %s" % vehicle.attitude
    print "Velocity: %s" % vehicle.velocity
    print "GPS: %s" % vehicle.gps_0
    print "Groundspeed: %s" % vehicle.groundspeed
    print "Airspeed: %s" % vehicle.airspeed
    print "Gimbal status: %s" % vehicle.gimbal
    print "Battery: %s" % vehicle.battery
    print "EKF OK?: %s" % vehicle.ekf_ok
    print "Last Heartbeat: %s" % vehicle.last_heartbeat
    print "Rangefinder: %s" % vehicle.rangefinder
    print "Rangefinder distance: %s" % vehicle.rangefinder.distance
    print "Rangefinder voltage: %s" % vehicle.rangefinder.voltage
    print "Heading: %s" % vehicle.heading
    print "Is Armable?: %s" % vehicle.is_armable
    print "System status: %s" % vehicle.system_status.state
    print "Mode: %s" % vehicle.mode.name  # settable
    print "Armed: %s" % vehicle.armed  # settable


def arm_and_takeoff():
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)


def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint.
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint == 0:
        return None
    missionitem = vehicle.commands[nextwaypoint -
                                   1]  #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat, lon, alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame,
                                          targetWaypointLocation)
    return distancetopoint


def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0  #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat /
                                            180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobal(newlat, newlon, original_location.alt)


@vehicle.on_attribute('last_heartbeat')
def listener(self, attr_name, value):
    global pause_script
    if value > 1 and not pause_script:
        print "Pausing script due to bad link"
        pause_script = True
    if value < 1 and pause_script:
        pause_script = False
        print "Un-pausing script"


# Get some vehicle attributes (system_statuste)
print "Preparing to start up"
arm_and_takeoff()

print "Status"
print_status()

############################################################
### Loop
while True:
    if vehicle.mode.name != "GUIDED":
        print "User has changed flight modes - aborting follow-me"
        break

    #if pause_script:
    #    while (pause_script == True):
    #        time.sleep(2)

    dest = get_location_metres(vehicle.location.global_frame, 10, 1)

    vehicle.simple_goto(dest, 10.00, 10.00)
    time.sleep(30)
#We need to take a relative position

############################################################

# Close vehicle object before exiting script
vehicle.close()
print("Completed")
