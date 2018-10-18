#!/usr/bin/env python

import gdal
import sys
import rospy
from geographic_msgs.msg import GeoPointStamped
from std_msgs.msg import Float32

last_depth_time = None

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
        

    def getDepth(self,lat,lon):
        x,y,z = self.coordinateTransformation.TransformPoint(lon,lat)
        #print x,y
        xi = self.inverseGeoTransorm[0]+x*self.inverseGeoTransorm[1]+y*self.inverseGeoTransorm[2]
        yi = self.inverseGeoTransorm[3]+x*self.inverseGeoTransorm[4]+y*self.inverseGeoTransorm[5]
        #print xi,yi
        try:
            return self.data[int(yi),int(xi)]
        except IndexError:
            return None
        
def position_callback(data):
    #print data
    depth = grid.getDepth(data.position.latitude, data.position.longitude)
    #print depth
    global last_depth_time
    if last_depth_time is None or data.header.stamp - last_depth_time > rospy.Duration(0.25):
        if depth is not None and depth >= 0:
            depth_publisher.publish(depth)
        last_depth_time = data.header.stamp


if __name__ == '__main__':
    rospy.init_node('mbes_sim')

    grid = BathyGrid(sys.argv[1])

    position_subscriber = rospy.Subscriber('/position', GeoPointStamped, position_callback)
    depth_publisher = rospy.Publisher('/depth', Float32, queue_size = 5)
    
    rospy.spin()
