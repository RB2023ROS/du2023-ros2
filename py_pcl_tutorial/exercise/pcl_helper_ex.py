import pcl
import ctypes
import struct
from random import randint

def random_color_gen():
    """ Generates a random color

        Args: None

        Returns:
            list: 3 elements, R, G, and B
    """
    r = randint(0, 255)
    g = randint(0, 255)
    b = randint(0, 255)
    return [r, g, b]

def XYZRGB_to_XYZ(XYZRGB_cloud):
    """ Converts a PCL XYZRGB point cloud to an XYZ point cloud (removes color info)

        Args:
            XYZRGB_cloud (PointCloud_PointXYZRGB): A PCL XYZRGB point cloud

        Returns:
            PointCloud_PointXYZ: A PCL XYZ point cloud
    """
    XYZ_cloud = pcl.PointCloud()
    points_list = []

    for data in XYZRGB_cloud:
        points_list.append([data[0], data[1], data[2]])

    XYZ_cloud.from_list(points_list)
    return XYZ_cloud

def rgb_to_float(color):
    """ Converts an RGB list to the packed float format used by PCL

        From the PCL docs:
        "Due to historical reasons (PCL was first developed as a ROS package),
         the RGB information is packed into an integer and casted to a float"

        Args:
            color (list): 3-element list of integers [0-255,0-255,0-255]

        Returns:
            float_rgb: RGB value packed as a float
    """
    hex_r = (0xff & color[0]) << 16
    hex_g = (0xff & color[1]) << 8
    hex_b = (0xff & color[2])

    hex_rgb = hex_r | hex_g | hex_b

    float_rgb = struct.unpack('f', struct.pack('i', hex_rgb))[0]

    return float_rgb


def get_color_list(cluster_count):
    """ Returns a list of randomized colors

        Args:
            cluster_count (int): Number of random colors to generate

        Returns:
            (list): List containing 3-element color lists
    """
    if (cluster_count > len(get_color_list.color_list)):
        for i in range(len(get_color_list.color_list), cluster_count):
            get_color_list.color_list.append(random_color_gen())
    return get_color_list.color_list