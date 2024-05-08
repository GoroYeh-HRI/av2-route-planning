#!/usr/bin/env python
# license removed for brevity
import json
from dataclasses import dataclass
from enum import Enum

import numpy as np

# Coordinate Transformation
import requests
import utm
from scipy.spatial.transform import Rotation as R

from av2.utils.utils import connect_paths, interpolate


class Instr(Enum):
    TURN_LEFT = 1
    TURN_RIGHT = 2
    GO_STRAIGHT = 3

@dataclass
class Node:
    lat: float
    lng: float
    instruction: str

@dataclass
class Step:
    start: Node
    goal:  Node
    instruction: str

def get_latlngs_from_steps(steps):
    """Convert a list of steps to two lists of [lats], [lngs]

    Args:
        [Step]: a list of "step"
                Each step is a list of two "Node": start and goal.

    Returns:
        lats: a list of latitudes
        lngs: a list of longitudes

    """

    lats = []
    lngs = []
    for step in steps:
        lats.extend([step.start.lat, step.goal.lat])
        lngs.extend([step.start.lng, step.goal.lng])
    return lats, lngs


def get_rough_route_steps(start, goal, api_key='AIzaSyBmYtO7rXCbqG02eEzLWb2FgexIve6FmvU'):
    """Get rough route using Google Route API

    Args:
        start: list[float, float] representing starting (latitude, longitude)
        goal: list[float, float] representing goal (latitude, longitude)
        api_key: string 

    Returns:
        [Step]: a list of "step"
                Each step is a list of two "Node": start and goal.

    """

    # ----------- Set up HTTP request --------------- #
    response_fields = 'routes'
    headers = {
        'Content-Type': 'application/json',
        'X-Goog-Api-Key': api_key,
        'X-Goog-FieldMask': response_fields,
    }


    json_data ={
        "origin": {
            "location":{
                "latLng":{
                    "latitude":  start[0],
                    "longitude": start[1]
                }
            }
        },
        "destination": {
            "location":{
                "latLng":{
                    "latitude":  goal[0], 
                    "longitude": goal[1]
                }
            }   
        },  
        "travelMode": "DRIVE",
        "polylineQuality" : "HIGH_QUALITY",
        "polylineEncoding": "GEO_JSON_LINESTRING",
        "routingPreference": "TRAFFIC_AWARE_OPTIMAL",
        "computeAlternativeRoutes": False,
        "routeModifiers": {
            "avoidTolls": False,
            "avoidHighways": False,
            "avoidFerries": False
        },
        "languageCode": "en-US",
        "units": "IMPERIAL"
    }

    print(" Sending POST request...")
    response = requests.post("https://routes.googleapis.com/directions/v2:computeRoutes", headers=headers, json=json_data)     
    response_dict = response.json()
    print(f"Getting POST response")
    # Serializing json
    json_object = json.dumps(response_dict, indent=4)

    # req_json_file = self.pkg_path_ + "/config/route_request_addr.json"
    output_file = "./route_response.json"
    print(f"write response.json to {output_file}")
    with open(output_file, "w") as outfile:
        outfile.write(json_object)  

    # ----------- Iterate HTTP response and extract lat lng --------------- #
    latitude_list  = []
    longitude_list = []

    output_steps = []

    if len(response_dict)==0:
        print(f"ERROR: Can't get rough route grom Google API")
        return

    # Note: inside 'routes' it's a lsit of ONE dictionary
    legs = response_dict["routes"][0]["legs"]
    for leg in legs:
        steps = leg["steps"]
        for step in steps:
            # For each navigation instruction,
            instruction = step['navigationInstruction']['instructions']
            print(f"    Instruction: {instruction}")
            # Classify instruction into 3 categories:
            if "Turn left" in instruction:
                instruction = Instr.TURN_LEFT
            elif "Turn right" in instruction:
                instruction = Instr.TURN_RIGHT
            else:
                instruction = Instr.GO_STRAIGHT

            start_lat = step['startLocation']['latLng']['latitude']
            start_lng = step['startLocation']['latLng']['longitude']
            start_node = Node(start_lat, start_lng, instruction)
            goal_lat = step['endLocation']['latLng']['latitude']
            goal_lng = step['endLocation']['latLng']['longitude']            
            goal_node = Node(goal_lat, goal_lng, instruction)
            output_steps.append(Step(start_node, goal_node, instruction))

    print(f"Number of steps: {len(output_steps)}")
    for step in output_steps:
        print(f"step: instruction {step.instruction}")
        print(f"    start node: {step.start.lat}, {step.start.lng}")
        print(f"    goal node: {step.goal.lat}, {step.goal.lng}")
        

    distance = response_dict["routes"][0]["localizedValues"]["distance"]["text"]
    print(f"route distance: {distance}")
    return output_steps

def get_rough_route(start, goal, api_key='AIzaSyBmYtO7rXCbqG02eEzLWb2FgexIve6FmvU'):
    """Get rough route using Google Route API

    Args:
        start: list[float, float] representing starting (latitude, longitude)
        goal: list[float, float] representing goal (latitude, longitude)
        api_key: string 

    Returns:
        lats: list[float]: list of latitudes of rough route
        lngs: list[float]: list of longitudes of rough route
    """
    print(f"start: {start}")
    print(f"goal : {goal}")

    # ----------- Set up HTTP request --------------- #
    response_fields = 'routes'
    headers = {
        'Content-Type': 'application/json',
        'X-Goog-Api-Key': api_key,
        'X-Goog-FieldMask': response_fields,
    }


    json_data ={
        "origin": {
            "location":{
                "latLng":{
                    "latitude":  start[0],
                    "longitude": start[1]
                }
            }
        },
        "destination": {
            "location":{
                "latLng":{
                    "latitude":  goal[0], 
                    "longitude": goal[1]
                }
            }   
        },  
        "travelMode": "DRIVE",
        "polylineQuality" : "HIGH_QUALITY",
        "polylineEncoding": "GEO_JSON_LINESTRING",
        "routingPreference": "TRAFFIC_AWARE_OPTIMAL",
        "computeAlternativeRoutes": False,
        "routeModifiers": {
            "avoidTolls": False,
            "avoidHighways": False,
            "avoidFerries": False
        },
        "languageCode": "en-US",
        "units": "IMPERIAL"
    }

    print(" Sending POST request...")
    response = requests.post("https://routes.googleapis.com/directions/v2:computeRoutes", headers=headers, json=json_data)     
    response_dict = response.json()
    print(f"Getting POST response")
    # print(f"response_dict: {response_dict}")
    # Serializing json
    json_object = json.dumps(response_dict, indent=4)

    # req_json_file = self.pkg_path_ + "/config/route_request_addr.json"
    output_file = "./route_response.json"
    print(f"write response.json to {output_file}")
    with open(output_file, "w") as outfile:
        outfile.write(json_object)  

    # ----------- Iterate HTTP response and extract lat lng --------------- #
    latitude_list  = []
    longitude_list = []

    if len(response_dict)==0:
        print(f"ERROR: Can't get rough route grom Google API")
        return

    init = False
    # Note: inside 'routes' it's a lsit of ONE dictionary
    legs = response_dict["routes"][0]["legs"]

    # A list of path. Each path is a list of waypoints
    # A step -> a path (road segments)
    paths = []
    for leg in legs:
        if not init:
            origin_coord = (leg['startLocation']['latLng']['latitude'], leg['startLocation']['latLng']['longitude'])
            init = True

        # Add interpolation within each leg
        steps = leg["steps"]
        for step in steps:
            # For each navigation instruction,
            poly_coords = step["polyline"]["geoJsonLinestring"]["coordinates"]
            num_of_coords = len(poly_coords)
            instruction = step['navigationInstruction']['instructions']
            print(f"    Instruction: {instruction}")
            # Classify instruction into 3 categories:
            if "Turn left" in instruction:
                instruction = "TURN_LEFT"
            elif "Turn right" in instruction:
                instruction = "TURN_RIGHT"
            else:
                instruction = "GO STRAIGHT"

   
            path = [] # a list of (lat, lng)
            # for coord in poly_coords:
            for idx in range(num_of_coords):
                coord = poly_coords[idx]
                lat_ = coord[1]
                lng_ = coord[0]
                latitude_list.append(lat_)
                longitude_list.append(lng_)             

                if idx>0:
                    path.append((lat_, lng_))

                # Add interpolation
                if idx < num_of_coords-1:
                    next_coord = poly_coords[idx+1]
                    s1 = (lat_, lng_)
                    s2 = (next_coord[1], next_coord[0])
                    interpolate_points = interpolate(s1, s2)   
                    path.extend(interpolate_points)

            paths.append(path)

    distance = response_dict["routes"][0]["localizedValues"]["distance"]["text"]
    print(f"Distance: {distance}")

    # ----------- Add splines between every adjacent path ---------- #
    rough_route = [] # Return: a list of (lat, lng)
    for i in range(len(paths)):
        # Append current path points
        rough_route.extend(paths[i])

        # if i < len(paths)-1:
        #     cur_path = paths[i]
        #     next_path = paths[i+1]

        #     # Determine if two paths space a lot
        #     # if len(cur_path) >=6 and len(next_path) >=5:
        #     if len(cur_path) >=2 and len(next_path) >=2:            
                
        #         # Covert lat long to x y
        #         cur_xys = []
        #         next_xys = []
        #         for point in cur_path:
        #             x, y, zone_number, zone_letter = utm.from_latlon(point[0], point[1])
        #             cur_xys.append((x,y))
        #         for point in next_path:
        #             x, y, _, _ = utm.from_latlon(point[0], point[1])
        #             next_xys.append((x,y))
        #         print(f"len(cur_xys) {len(cur_xys)}")
        #         print(f"len(next_xys) {len(next_xys)}")
        #         print(f"np.asarray(cur_xys).shape {np.asarray(cur_xys).shape}")
        #         splined_path = connect_paths(np.asarray(cur_xys), np.asarray(next_path))

        #         # Convert (x,y) to lat,lng and insert between 
        #         for wpt_xy in splined_path:
        #             lat, lng = utm.to_latlon(wpt_xy)
        #             rough_route.append((lat, lng))

    print(f"Number of spline points: {len(rough_route)}")
    print(f"Average distance per point: {response_dict['routes'][0]['legs'][0]['distanceMeters']/len(rough_route)} meter/point")

    # return rough_route
    lats, lngs = zip(*rough_route)
    return lats, lngs