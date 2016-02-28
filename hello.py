# Import DroneKit-Python
from dronekit import connect, VehicleMode
import socket
import exceptions
import time
import sys

# Connect to the Vehicle.
connection_string = 'udp:127.0.0.1:14550'
print "Connecting to vehicle on: %s " % connection_string
#vehicle = connect('/dev/ttyUSB0', wait_ready=True, heartbeat_timeout=15, status_printer=status_printer)

vehicle = connect(connection_string, wait_ready=True, heartbeat_timeout=30)
#vehicle = connect('/dev/ttyUSB0', baud=57600, wait_ready=True, heartbeat_timeout=15)
#if (vehicle.wait_ready
# Get some vehicle attributes (state)
print "Get some vehicle attribute values:"
print " GPS: %s" % vehicle.gps_0
print " Battery: %s" % vehicle.battery
print " Last Heartbeat: %s" % vehicle.last_heartbeat
print " Is Armable?: %s" % vehicle.is_armable
print " System status: %s" % vehicle.system_status.state
print " Mode: %s" % vehicle.mode.name    # settable

# Close vehicle object before exiting script
vehicle.close()

# Shut down simulator
sitl.stop()
print("Completed")

def status_printer(txt):
    print "error %s" % txt
