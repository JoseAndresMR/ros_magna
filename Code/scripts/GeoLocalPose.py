import numpy as np
import geodesy.utm
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point32

class GeoLocalPose(object):

    def __init__(self, origin_geo):
        self.origin_geo = origin_geo
        self.origin_UTM = geodesy.utm.fromMsg(origin_geo) 

    def geoToUtmToCartesian(self,new_coordinate_geo):
        # Conversion from geographic coordinates to UTM.
        new_coordinate_UTM = geodesy.utm.fromMsg(new_coordinate_geo)

        # new_coordinate_geo_second = new_coordinate_UTM.toMsg()
        # print(new_coordinate_geo_second)
        # Conversion from geographic coordinates to UTM.
        
                    
        new_coordinate_cartesian = Point32()                            # Cartesian coordinate that this function will return.

        # The problem with UTM is that if there are coordinates in more than one zone, it's difficult to merge the coordinates of the different zones.
        # This is because each zone has 6 degrees of longitude, and the width in meters of the zone is variable from equator to the poles.
        # Something similar when the coordinates are in different hemispheres, separated by the equator. Each hemisphere has a different origin for the y axis.
        # These are the reasons why the x and y assignation for the Cartesian conversion isn't a simple UTM substraction.

        # Assignating "new_coordinate_cartesian.x" is tricky when the actual coordinate and the origin of the Cartesian coordinates are on different zones.

        if int(new_coordinate_UTM.zone)==60 and self.origin_UTM.zone==1:          # Coordinate and origin separated by the +- 180 degrees longitude
            geo_180_w = GeoPoint()
            geo_180_e = GeoPoint()
            geo_180_w.longitude = 179.9999999
            geo_180_w.latitude = new_coordinate_geo.latitude
            geo_180_w.altitude = new_coordinate_geo.altitude
            geo_180_e.longitude = -179.9999999
            geo_180_e.latitude = new_coordinate_geo.latitude
            geo_180_e.altitude = new_coordinate_geo.altitude
            utm_180_w = UTMPoint((geo_180_w))
            utm_180_e = UTMPoint((geo_180_e))
            new_coordinate_cartesian.x = new_coordinate_UTM.easting - utm_180_w.easting + utm_180_e.easting - self.origin_UTM.easting    # Transformation of the x coordinate taking into account the different zones. " - utm_180_w.easting + utm_180_e.easting " computes the x step in both sides of the border longitude.
        
        elif self.origin_UTM.zone==60 and int(new_coordinate_UTM.zone)==1:    # Coordinate and origin separated by the +-180 degrees longitude
            geo_180_w = GeoPoint()
            geo_180_e = GeoPoint()
            geo_180_w.longitude = 179.9999999
            geo_180_w.latitude = new_coordinate_geo.latitude
            geo_180_w.altitude = new_coordinate_geo.altitude
            geo_180_e.longitude = -179.9999999
            geo_180_e.latitude = new_coordinate_geo.latitude
            geo_180_e.altitude = new_coordinate_geo.altitude
            utm_180_w = UTMPoint((geo_180_w))
            utm_180_e = UTMPoint((geo_180_e))
            new_coordinate_cartesian.x = new_coordinate_UTM.easting - utm_180_e.easting + utm_180_w.easting - self.origin_UTM.easting    # Transformation of the x coordinate taking into account the different zones. " - utm_180_w.easting + utm_180_e.easting " computes the x step in both sides of the border longitude.
        
        elif int(new_coordinate_UTM.zone) < self.origin_UTM.zone:
            quotient_from_int_division = int(np.max(np.abs(new_coordinate_geo.longitude),self.origin_geo.longitude))/6
            border_longitude = quotient_from_int_division * 6.0 * (-1)**np.sigbit(np.max(new_coordinate_geo.longitude,origin_geo.longitude))
            geo_w = GeoPoint()
            geo_e = GeoPoint()
            geo_w.longitude = border_longitude - 0.0000001
            geo_w.latitude = new_coordinate_geo.latitude
            geo_w.altitude = new_coordinate_geo.altitude
            geo_e.longitude = border_longitude + 0.0000001
            geo_e.latitude = new_coordinate_geo.latitude
            geo_e.altitude = new_coordinate_geo.altitude
            utm_w = UTMPoint((geo_w))
            utm_e = UTMPoint((geo_e))
            new_coordinate_cartesian.x = new_coordinate_UTM.easting - utm_w.easting + utm_e.easting - self.origin_UTM.easting       # Transformation of the x coordinate taking into account the different zones. " - utm_w.easting + utm_e.easting " computes the x step in both sides of the border longitude.
        
        elif self.origin_UTM.zone < int(new_coordinate_UTM.zone):
            quotient_from_int_division = int(np.max(np.abs(new_coordinate_geo.longitude),self.origin_geo.longitude))/6
            border_longitude = quotient_from_int_division * 6.0 * (-1)**np.sigbit(np.max(new_coordinate_geo.longitude,self.origin_geo.longitude))
            geo_w = GeoPoint()
            geo_e = GeoPoint()
            geo_w.longitude = border_longitude - 0.0000001
            geo_w.latitude = new_coordinate_geo.latitude
            geo_w.altitude = new_coordinate_geo.altitude
            geo_e.longitude = border_longitude + 0.0000001
            geo_e.latitude = new_coordinate_geo.latitude
            geo_e.altitude = new_coordinate_geo.altitude
            utm_w = UTMPoint((geo_w))
            utm_e = UTMPoint((geo_e))
            new_coordinate_cartesian.x = new_coordinate_UTM.easting - utm_e.easting + utm_w.easting - self.origin_UTM.easting      # Transformation of the x coordinate taking into account the different zones. " - utm_w.easting + utm_e.easting " computes the x step in both sides of the border longitude.

        else:
            # The actual coordinate is in the same zone that the origin (the first station). This is the normal situation, the assigntation of the x axis value is trivial (simple subtraction).
            new_coordinate_cartesian.x = new_coordinate_UTM.easting -self.origin_UTM.easting

        # Assignating "new_coordinate_cartesian.y" is also tricky when the actual coordinate and the origin of the Cartesian coordinates are on different hemispheres:
        if self.origin_UTM.band=='N' and new_coordinate_UTM.band=='M':   # Coordinate and origin separated by the 0 degrees latitude
            geo_0_n = GeoPoint()
            geo_0_s = GeoPoint()
            geo_0_n.longitude = new_coordinate_geo.longitude
            geo_0_n.latitude = 0.0000001
            geo_0_n.altitude = new_coordinate_geo.altitude
            geo_0_s.longitude = new_coordinate_geo.longitude
            geo_0_s.latitude = -0.0000001
            geo_0_s.altitude = new_coordinate_geo.altitude
            utm_0_n = UTMPoint((geo_0_n))
            utm_0_s = UTMPoint((geo_0_s))
            new_coordinate_cartesian.y = new_coordinate_UTM.northing - utm_0_s.northing + utm_0_n.northing - self.origin_UTM.northing    # Transformation of the y coordinate taking into account the different hemispheres. " - utm_0_s.northing + utm_0_n.northing " computes the y step in both sides of the equator.
        
        elif new_coordinate_UTM.band=='N' and self.origin_UTM.band=='M':  # Coordinate and origin separated by the 0 degrees latitude
            geo_0_n = GeoPoint()
            geo_0_s = GeoPoint()
            geo_0_n.longitude = new_coordinate_geo.longitude
            geo_0_n.latitude = 0.0000001
            geo_0_n.altitude = new_coordinate_geo.altitude
            geo_0_s.longitude = new_coordinate_geo.longitude
            geo_0_s.latitude = -0.0000001
            geo_0_s.altitude = new_coordinate_geo.altitude
            utm_0_n = UTMPoint((geo_0_n))
            utm_0_s = UTMPoint((geo_0_s))
            new_coordinate_cartesian.y = new_coordinate_UTM.northing - utm_0_n.northing + utm_0_s.northing - self.origin_UTM.northing    # Transformation of the y coordinate taking into account the different hemispheres. " - utm_0_n.northing + utm_0_s.northing " computes the y step in both sides of the equator.
        
        else:
            # The actual coordinate is in the same hemisphere that the origin (the first station). This is the normal situation, the assigntation of the y axis value is trivial (simple subtraction).
            new_coordinate_cartesian.y = new_coordinate_UTM.northing-self.origin_UTM.northing
        
        # Assignating "new_coordinate_cartesian.z" is trivial always, simple subtraction.
        new_coordinate_cartesian.z = new_coordinate_UTM.altitude-self.origin_UTM.altitude

        return new_coordinate_cartesian#, new_coordinate_UTM

    def CartesionToUTMToGeo(self,new_coordinate_cartesian):

        new_coordinate_UTM = geodesy.utm.UTMPoint(zone = self.origin_UTM.zone, band = self.origin_UTM.band)

        new_coordinate_UTM.easting = new_coordinate_cartesian.x + self.origin_UTM.easting
        new_coordinate_UTM.northing = new_coordinate_cartesian.y + self.origin_UTM.northing
        new_coordinate_UTM.altitude = new_coordinate_cartesian.z# + self.origin_UTM.altitude

        new_coordinate_geo = new_coordinate_UTM.toMsg()

        return new_coordinate_geo#, new_coordinate_UTM

    def setGeo(self,wp):

        new_coordinate_cartesian = Point32(wp.x_lat,wp.y_long,wp.z_alt)
        self.actual_coordinate_geo = self.CartesionToUTMToGeo(new_coordinate_cartesian)

        return self.actual_coordinate_geo


# new_coordinate_geo = GeoPoint(47.3974914551, 8.55076217651, 40.0)
# origin_geo = GeoPoint(47.3974914552, 8.550762763961, 40.0)

# geo_local_pose = GeoLocalPose(origin_geo)
# local_pose = geo_local_pose.geoToUtmToCartesian(new_coordinate_geo)
# print(local_pose)

# new_coordinate_cartesian = Point32(300,4,5)
# new_coordinate_geo = geo_local_pose.CartesionToUTMToGeo(new_coordinate_cartesian)
# print(new_coordinate_geo)

