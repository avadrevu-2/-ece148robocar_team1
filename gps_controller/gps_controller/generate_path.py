import osmnx as ox
import networkx as nx
from osmnx import osm_xml
import os

# Params
graphs_folder = '/home/projects/ros2_ws/src/ucsd_robocar_hub2/gps_controller/graphs'
graph_file = 'current_path.osm'


def generate_path(origin_point, target_point, perimeter, logger, mode):
    """
    Reference: https://github.com/ThomasAFink/optimal_path_dijkstra_for_data_science/blob/main/dijkstra_map.py
    """
    try:
        # Split points into lat lon
        origin_lat = origin_point[0]
        origin_lon = origin_point[1]
        target_lat = target_point[0]
        target_lon = target_point[1]

        # If the origin is further from the equator than the target
        if  origin_lat > target_lat:
            north = origin_lat 
            south = target_lat
        else:
            north = target_lat
            south = origin_lat

        # If the origin is further from the prime meridian than the target
        if  origin_lon > target_lon:
            east = origin_lon
            west = target_lon
        else:
            east = target_lon
            west = origin_lon
        logger.info("Setup Points")
        
        # If graph file already exists, load that for environment
        graph_path = os.path.join(graphs_folder, graph_file)
        if os.path.exists(graph_path):
            roadgraph = ox.graph_from_xml(graph_path)
            logger.info("Loaded graph file from XML")

        # Otherwise, download a new one and save it
        else:
            logger.info("Couldn't find graph file, downloading")

            # Acceptable modes for map
            modes = ['drive', 'bike', 'walk']
            if mode not in modes:
                mode = modes[0]  # default to drive if not one of these three
            
            # download graph file
            roadgraph = ox.graph_from_bbox(north+perimeter, south-perimeter, east+perimeter, west-perimeter, network_type=modes[0], simplify=False)
            logger.info("Downloaded graph file, saving")

            # save graph file
            osm_xml.save_graph_xml(roadgraph, filepath=graph_path)
            logger.info("Saved graph file")

        # Get origin and target nodes from graph file
        logger.info("Calculating nodes")
        origin_node = ox.nearest_nodes(roadgraph, origin_lon, origin_lat)
        target_node = ox.nearest_nodes(roadgraph, target_lon, target_lat)

        # Plan shortest path from origin to target node
        route = nx.shortest_path(roadgraph, origin_node, target_node, weight='length', method='dijkstra')
        logger.info("Calculated shortest path")

        # Store points from path as coordinates
        coordinates = []
        for i in route:
            point = roadgraph.nodes[i]
            # format is (y, x) = (lat, lon)
            coordinates.append((point['y'], point['x']))
        logger.info("Converted path to points")
        return coordinates
    
    except Exception as e:
        logger.info("Couldn't calculate path")
        return None