import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import yaml
import os
from shapely.geometry import Polygon, Point
import xml.etree.ElementTree as ET

def readScenario():
    # Load and parse the XML file
    tree = ET.parse(SCENARIO + SCENARIO_NAME + '.xml')
    root = tree.getroot()
    
    wps = {}
    for waypoint in root.findall('waypoint'):
        waypoint_id = waypoint.get('id')
        x = float(waypoint.get('x'))
        y = float(waypoint.get('y'))
        r = float(waypoint.get('r'))
        wps[waypoint_id] = {'x': x, 'y': y, 'r': r}
    
    return wps
        

if __name__ == '__main__':
    
    MAP_DIR = os.path.expanduser('~/git/PeopleFlow/utilities_ws/src/RA-L/peopledensity/maps/')
    MAP_NAME = 'warehouse'
    SCENARIO = os.path.expanduser('~/git/PeopleFlow/utilities_ws/src/RA-L/peopledensity/scenarios/')
    SCENARIO_NAME = 'warehouse'

    WPS = readScenario()
    
    # Define areas (group of waypoints) and their corresponding polygon boundaries
    AREAS = {
        'shelves_12': Polygon([(-18.839, 9), (-10.5, 9), (-10.5, 3), (-18.839, 3)]),
        'shelves_34': Polygon([(-18.839, 2.5), (-10.5, 2.5), (-10.5, -3), (-18.839, -3)]),
        'shelves_56': Polygon([(-18.839, -3.5), (-10.5, -3.5), (-10.5, -10.5), (-18.839, -10.5)]),
        'shelf_top_corr': Polygon([(-10.5, 10.5), (-7.2, 10.5), (-7.1, 2), (-10.5, 2)]),
        'shelf_centre_corr': Polygon([(-10.5, 2), (-7.1, 2), (-7.1, -4), (-10.5, -4)]),
        'shelf_bottom_corr': Polygon([(-10.5, -4), (-7.1, -4), (-7.1, -10.5), (-10.5, -10.5)]),
        'entrance': Polygon([(10.6, 10), (10.6, 6.87), (2, 6.87), (2, 10)]),
        'corridor_0': Polygon([(2, 10), (2, 6.87), (-1.55, 6.87), (-1.55, 10)]),
        'corridor_1': Polygon([(-1.54, 10), (-1.54, 5), (-7.14, 5), (-7.19, 10)]),
        'corridor_2': Polygon([(-7.14, 5), (-7.1, -1.93), (-1.54, -1.93), (-1.54, 5)]),
        'corridor_3': Polygon([(-7.1, -1.93), (-1.54, -1.93), (-1.54, -5.05), (1.95, -5.05), (1.95, -10.5), (-7.1, -10.5)]),
        'office_1': Polygon([(-1.54, 6.9), (1.95, 6.9), (1.95, 4), (-1.54, 4)]),   
        'office_2': Polygon([(-1.54, 4), (1.95, 4), (1.95, 1), (-1.54, 1)]),   
        'toilet_1': Polygon([(-1.54, 1), (1.95, 1), (1.95, -1.92), (-1.54, -1.92)]),   
        'toilet_2': Polygon([(-1.54, -1.92), (1.95, -1.92), (1.95, -5), (-1.54, -5)]),   
        'kitchen_1': Polygon([(10.6, 6.9), (1.95, 6.9), (1.95, 2), (5.8, 2), (5.8, 4.8), (10.6, 4.8)]),         
        'kitchen_2': Polygon([(1.95, 2), (5.8, 2), (5.8, -2.4), (1.95, -2.4)]),         
        'kitchen_3': Polygon([(1.95, -2.4), (5.8, -2.4), (5.8, -10.5), (1.95, -10.5)]),         
        'tables_23': Polygon([(5.8, 4), (10.6, 4), (10.6, 0), (5.8, 0)]),         
        'tables_45': Polygon([(5.8, -0.35), (10.6, -0.35), (10.6, -4.35), (5.8, -4.35)]),         
        'tables_6': Polygon([(5.8, -4.9), (10.6, -4.9), (10.6, -10.5), (5.8, -10.5)])         
    }
    
    
    # Map waypoints to clusters
    WP_AREA = {}
    for wp, coords in WPS.items():
        point = Point(coords['x'], coords['y'])
        for cluster_name, cluster_polygon in AREAS.items():
            if cluster_polygon.contains(point):
                WP_AREA[wp] = cluster_name
                break
    
    
    with open(MAP_DIR + MAP_NAME + '/map.yaml', 'r') as yaml_file:
        map_info = yaml.safe_load(yaml_file)

    # Step 1: Load the PNG Image
    map_image = mpimg.imread(MAP_DIR + MAP_NAME + '/map.pgm')

    # Step 2: Plot the Map
    # Get resolution and origin from YAML
    resolution = map_info['resolution']
    origin_x, origin_y = map_info['origin'][:2]

    # Plot the map image
    plt.imshow(map_image, extent=(origin_x, origin_x + len(map_image[0]) * resolution, 
                                  origin_y, origin_y + len(map_image) * resolution),
               cmap='gray')
    
    for area_name, poly in AREAS.items():
        x, y = poly.exterior.xy
        plt.fill(x, y, alpha=0.5, label=area_name)
    plt.show()

    
