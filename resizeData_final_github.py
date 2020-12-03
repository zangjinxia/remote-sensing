# -*- coding: utf-8 -*-
"""
@author zangjinx
@date 2020-12-3
@brief 利用shp裁剪影像,影像需具有坐标信息
"""

import gdal
from osgeo import gdal_array,ogr
import gdalnumeric
from PIL import Image, ImageDraw
import os
import sys
import numpy as np

gdal.UseExceptions()

class Dataset:
    def read_img(self, filename):
        dataset = gdal.Open(filename)

        width = dataset.RasterXSize
        height = dataset.RasterYSize
        band = dataset.RasterCount
        im_data = dataset.ReadAsArray(0, 0, width, height)

        geotrans = dataset.GetGeoTransform()
        proj = dataset.GetProjection()
        # data = np.zeros([width, height, band])

        return im_data, proj, geotrans,band,width,height

    def write_tiff(self, filename, proj, geotrans, data,minx,maxy):
        # gdal数据类型包括
        # gdal.GDT_Byte,
        # gdal .GDT_UInt16, gdal.GDT_Int16, gdal.GDT_UInt32, gdal.GDT_Int32,
        # gdal.GDT_Float32, gdal.GDT_Float64
        # 判断栅格数据的数据类型
        if 'int8' in data.dtype.name:
            datatype = gdal.GDT_Byte
        elif 'int16' in data.dtype.name:
            datatype = gdal.GDT_UInt16
        else:
            datatype = gdal.GDT_Float32

        # 判读数组维数
        if len(data.shape) == 3:
            bands, height, width = data.shape
        else:
            bands = 1
            height, width = data.shape
        # 创建文件
        driver = gdal.GetDriverByName("GTiff")
        dataset = driver.Create(filename, width, height, bands, datatype)
        geotrans_update = (minx, geotrans[1],geotrans[2],maxy,geotrans[4],geotrans[5])
        dataset.SetGeoTransform(geotrans_update)
        dataset.SetProjection(proj)

        if bands == 1:
            dataset.GetRasterBand(1).WriteArray(data)
        else:
            for i in range(bands):
                dataset.GetRasterBand(i + 1).WriteArray(data[i])
        del dataset

# This function will convert the rasterized clipper shapefile
# to a mask for use within GDAL.

def world2Pixel(geoMatrix, x, y):
    """
    Uses a gdal geomatrix (gdal.GetGeoTransform()) to calculate
    the pixel location of a geospatial coordinate
    """
    ulX = geoMatrix[0]
    ulY = geoMatrix[3]
    xDist = geoMatrix[1]
    pixel = int((x - ulX) / xDist)
    line = int((ulY - y) / xDist)
    return (pixel, line)


def main( shapefile_path, raster_path, outRaster_path):
    # 读取栅格数据
    dataset = Dataset()
    srcArray,proj,geo,band,width,height = dataset.read_img(raster_path)
    print(geo)
    if band == 1:
        clip = np.array(srcArray,dtype = float)
    else:
        clip = np.empty((band,height,width))
        for i in range(band):
            clip[i] = np.array(srcArray[i],dtype = float)


    # 打开shp文件
    shapef = ogr.Open(shapefile_path)
    lyr = shapef.GetLayer( os.path.split( os.path.splitext( shapefile_path )[0] )[1] )

    poly = lyr.GetNextFeature()

    # 将范围转为图像像素坐标
    minX, maxX, minY, maxY = lyr.GetExtent()
    ulX, ulY = world2Pixel(geo, minX, maxY)
    lrX, lrY = world2Pixel(geo, maxX, minY)

    # 计算新影像的尺寸大小
    pxWidth = int(lrX - ulX)
    pxHeight = int(lrY - ulY)
    if band == 1:
        clip = srcArray[ulY:lrY, ulX:lrX]
    else:

        clip = clip[:,ulY:lrY, ulX:lrX]



    # 创建一个新矩阵
    geoTrans = list(geo)
    geoTrans[0] = minX
    geoTrans[3] = maxY


    dataset.write_tiff(outRaster_path, proj, geo, clip,minX,maxY)

if __name__ == '__main__':

    # if len(sys.argv) != 4:
    #     # distutils.log.error("not enougth input parameters")
    #     sys.exit(-1)
    # shapefile_path = sys.argv[1]
    # raster_path = sys.argv[2]
    # outRaster_path = sys.argv[3]
    shapefile_path = 'I:/data/GF_radiance/range.shp'
    raster_path = 'I:/data/GF_radiance/Vege_test/GF1caijian.tif'
    outRaster_path = 'D:/AAdata/resize8.tif'
    main( shapefile_path, raster_path, outRaster_path )