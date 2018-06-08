#!/usr/bin/env python

import pyModeS as pms


###### Core functions for ADS-B decoding #####
msg = "8D4840D6202CC371C32CE0576098" 
print "icao",pms.adsb.icao(msg)
print "typecode",pms.adsb.typecode(msg)

# # typecode 1-4
print "callsign",pms.adsb.callsign(msg)

# typecode 5-8 (surface) and 9-18 (airborne)
msg_even = "8D40621D58C382D690C8AC2863A7"
msg_odd = "8D40621D58C386435CC412692AD6"
t_even = 0.0
t_odd = 1.0
lat_ref = 52.258
lon_ref = 3.918
print "position",pms.adsb.position(msg_even, msg_odd, t_even, t_odd, lat_ref=None, lon_ref=None)
print "airborne_position",pms.adsb.airborne_position(msg_even, msg_odd, t_even, t_odd)
print "surface_position",pms.adsb.surface_position(msg_even, msg_odd, t_even, t_odd, lat_ref, lon_ref)

msg = "8D40621D58C382D690C8AC2863A7"
print "position_with_ref",pms.adsb.position_with_ref(msg, lat_ref, lon_ref)
print "airborne_position_with_ref",pms.adsb.airborne_position_with_ref(msg, lat_ref, lon_ref)
print "surface_position_with_ref",pms.adsb.surface_position_with_ref(msg, lat_ref, lon_ref)

print "altitude",pms.adsb.altitude(msg)

# # typecode: 19
msg = "8D485020994409940838175B284F"
print "velocity",pms.adsb.velocity(msg)          # handles both surface & airborne messages. return: v(modulo en kt), h (heading en deg), Vr (verticalrate en ft/min)
print "speed_heading",pms.adsb.speed_heading(msg)     # handles both surface & airborne messages
# print "surface_velocity",pms.adsb.surface_velocity(msg)
# print "airborne_velocity",pms.adsb.airborne_velocity(msg)