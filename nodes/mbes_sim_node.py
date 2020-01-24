#!/usr/bin/env python

import gdal
import sys
import rospy
from geographic_msgs.msg import GeoPointStamped
from std_msgs.msg import Float32
from marine_msgs.msg import NavEulerStamped
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import math
from project11 import geodesic

last_depth_time = None

heading = None

# beam angle of 120 degrees
tan_half_swath_angle = math.tan(math.radians(120/2.0))
beam_count = 20

class BathyGrid:
    def __init__(self, fname):
        
        self.dataset = gdal.Open(fname, gdal.GA_ReadOnly)
        #print 'opened',self.dataset.GetDescription()
        self.band = self.dataset.GetRasterBand(1)
        self.geoTransform = self.dataset.GetGeoTransform()
        self.inverseGeoTransorm = gdal.InvGeoTransform(self.geoTransform)
        #print self.geoTransform
        self.data = self.band.ReadAsArray()
        #print self.data.shape
        
        sourceSR = gdal.osr.SpatialReference()
        sourceSR.SetWellKnownGeogCS("WGS84")
        
        targetSR = gdal.osr.SpatialReference()
        targetSR.ImportFromWkt(self.dataset.GetProjection())
        
        self.coordinateTransformation = gdal.osr.CoordinateTransformation(sourceSR, targetSR)

    def getXY(self,lat,lon):
        return self.coordinateTransformation.TransformPoint(lon,lat)[:2]
        
    def getDepthAtLatLon(self,lat,lon):
        x,y,z = self.coordinateTransformation.TransformPoint(lon,lat)
        #print x,y
        return self.getDepth(x,y)
    
    def getDepth(self,x,y):
        xi = self.inverseGeoTransorm[0]+x*self.inverseGeoTransorm[1]+y*self.inverseGeoTransorm[2]
        yi = self.inverseGeoTransorm[3]+x*self.inverseGeoTransorm[4]+y*self.inverseGeoTransorm[5]
        #print xi,yi
        try:
            return self.data[int(yi),int(xi)]
        except IndexError:
            return None

def heading_callback(data):
    global heading
    heading = data.orientation.heading
        
def position_callback(data):
    #print data
    depth = grid.getDepthAtLatLon(data.position.latitude, data.position.longitude)
    #print depth
    global last_depth_time
    if last_depth_time is None or data.header.stamp - last_depth_time > rospy.Duration(0.25):
        if depth is not None and depth >= 0:
            depth_publisher.publish(depth)
            if heading is not None:
                swath_half_width = depth*tan_half_swath_angle;
                #print 'swath half width:',swath_half_width
                lon_rad = math.radians(data.position.longitude)
                lat_rad = math.radians(data.position.latitude)
                port_outer_beam_location = geodesic.direct(lon_rad, lat_rad, math.radians(heading-90),swath_half_width)
                starboard_outer_beam_location = geodesic.direct(lon_rad, lat_rad, math.radians(heading+90),swath_half_width)
                #print 'outer beam locations:',port_outer_beam_location,starboard_outer_beam_location
                port_outer_beam_location_xy = grid.getXY(math.degrees(port_outer_beam_location[1]), math.degrees(port_outer_beam_location[0]))
                starboard_outer_beam_location_xy = grid.getXY(math.degrees(starboard_outer_beam_location[1]), math.degrees(starboard_outer_beam_location[0]))
                dx = (starboard_outer_beam_location_xy[0] - port_outer_beam_location_xy[0])/float(beam_count)
                dy = (starboard_outer_beam_location_xy[1] - port_outer_beam_location_xy[1])/float(beam_count)
                
                dx_re_mbes = 2.0*swath_half_width/float(beam_count)
                
                pointCloud = PointCloud()
                pointCloud.header.frame_id = 'mbes'
                pointCloud.header.stamp = data.header.stamp
                
                for i in range(beam_count):
                    x = port_outer_beam_location_xy[0] + dx*i
                    y = port_outer_beam_location_xy[1] + dy*i
                    z = grid.getDepth(x,y)
                    #print 'depth:', z
                    if z is not None:
                        p = Point32()
                        p.y = -(-swath_half_width+i*dx_re_mbes)
                        p.z = -z
                        pointCloud.points.append(p)
                ping_publisher.publish(pointCloud)
                    
        last_depth_time = data.header.stamp


if __name__ == '__main__':
    rospy.init_node('mbes_sim')

    grid = BathyGrid(sys.argv[1])

    position_subscriber = rospy.Subscriber('/position', GeoPointStamped, position_callback)
    heading_subscriber = rospy.Subscriber('/heading', NavEulerStamped, heading_callback)
    depth_publisher = rospy.Publisher('/depth', Float32, queue_size = 5)
    ping_publisher = rospy.Publisher('/mbes_ping',PointCloud,queue_size=10)

    
    rospy.spin()
