"""
It calculates the direction to travel in to get to a destination on the
road map, given the coordinates of the start and end points.
"""


import json
import math
from typing import NamedTuple, Tuple
import requests


class MapPosition(NamedTuple):
    """ It contains the global coordinates of a point on the map. """
    latitude: float
    longitude: float


def main(start: MapPosition, end: MapPosition) -> Tuple[str, float]:
    """
    It calculates the shortest route on the road map from the start
    position to the end position, and returns the compass bearing
    required to start the journey.
    """
    err, get_request_string_for_route = make_get_request_string_for_route(
        start, end)
    if err is not None:
        return err, None
    raw_request = requests.get(get_request_string_for_route)
    if raw_request.status_code != 200:
        return (
            "Router server gave status code {}".format(
                raw_request.status_code),
            None)
    return calculate_direction(raw_request.content.decode('utf-8'))


def nearest_point(v: MapPosition) -> Tuple[str, MapPosition]:
    """
    It finds the nearest point on the road network to the given map
    position.
    """
    request_string: str = make_get_request_string_for_snap(v)
    raw_response = requests.get(request_string)
    if raw_response.status_code != 200:
        return (
            'Router server gave status code {}'.format(
                raw_response.status_code),
            None)
    return None, parse_snap_response(raw_response.content.decode('utf-8'))


def make_get_request_string_for_snap(v: MapPosition) -> str:
    """
    It makes the string for finding out the nearest accessible node to
    the map coordinate.
    """
    return 'http://localhost:5000/nearest/v1/driving/{},{}'.format(
        v.longitude, v.latitude)


def vector_angle_to_north(v: MapPosition) -> Tuple[str, float]:
    """
    It calculates the angle of the vector relative to (0, 1).  The angle
    is between 0 and 2π radians.
    """
    magnitude: float = (v.longitude**2 + v.latitude**2)**0.5
    if math.isclose(magnitude, 0):
        return "Vector has zero magnitude.", None
    angle = math.acos(v.latitude / magnitude)
    if v.longitude < 0:
        angle = 2*math.pi - angle
    return None, angle


def parse_route(
        raw_route: str) -> Tuple[str, MapPosition, MapPosition]:
    """
    It decodes the json string containing the route plan, and extracts
    the coordinates of the ends of the first segment of it.
    """
    route_as_dict = json.loads(raw_route)
    if 'code' not in route_as_dict:
        return "No 'code' key in route string.", None, None
    if route_as_dict['code'] != 'Ok':
        return ('Route code is {}'.format(route_as_dict['code']),
                None,
                None)
    points = route_as_dict['routes'][0]['geometry']['coordinates']
    return (None,
            MapPosition(longitude=points[0][0], latitude=points[0][1]),
            MapPosition(longitude=points[1][0], latitude=points[1][1]))


def parse_snap_response(raw_response: str) -> MapPosition:
    """ It extracts the first location from the snap response. """
    as_json = json.loads(raw_response)
    if len(as_json['waypoints']) > 1:
        raw_position = as_json['waypoints'][1]['location']
    else:
        raw_position = as_json['waypoints'][0]['location']
    return MapPosition(
        longitude=float(raw_position[0]),
        latitude=float(raw_position[1]))


def calculate_direction(raw_route: str) -> Tuple[str, float]:
    """
    It calculates the bearing along the first segment of the route.
    """
    err, start, end = parse_route(raw_route)
    if err is not None:
        return err, None
    diff: MapPosition = MapPosition(
        longitude=end.longitude - start.longitude,  # type: ignore
        latitude=end.latitude - start.latitude)  # type: ignore
    err, angle = vector_angle_to_north(diff)
    if err is not None:
        return "The first two nodes in the route are identical.", None
    return None, angle


def valid_map_position(m: MapPosition) -> bool:
    """ It checks that a MapPosition is a two-tuple of floats. """
    return (len(m) == 2 and
            isinstance(m.latitude, float) and
            isinstance(m.longitude, float))


def make_get_request_string_for_route(
        start: MapPosition,
        destination: MapPosition) -> Tuple[str, str]:
    """
    It creates an SQL query string that gets the shortest route between
    the start and end nodes.
    """
    if not valid_map_position(start):
        return "Start position is not valid.", None
    if not valid_map_position(destination):
        return "Destination position is not valid.", None
    return (
        None,
        "http://localhost:5000/route/v1/driving/{},{};{},{}?geometries="
        "geojson".format(start.longitude, start.latitude,
                         destination.longitude, destination.latitude))
