"""
It calculates the direction to travel in to get to a destination on the
road map, given the coordinates of the start and end points.
"""


import json
import math
from typing import NamedTuple, Tuple, List
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
    angle = math.acos(v.longitude / magnitude)
    return None, angle


EARTH_RADIUS: float = 6371e3


def distance_between(a: MapPosition, b: MapPosition) -> float:
    """
    It calculates the distance in metres between two map points, each
    specified by a latitude and longitude.

    This formula is taken from

    https://www.movable-type.co.uk/scripts/latlong.html

    A copy of the html of the page is in the file
    DistanceBetweenPoints.html.
    """
    phi1: float = math.radians(a.latitude)
    phi2: float = math.radians(b.latitude)
    delta_phi: float = phi2 - phi1
    delta_lambda: float = math.radians(b.longitude - a.longitude)
    aa: float = (
        math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2)
        * math.sin(delta_lambda/2)**2)
    c: float = 2 * math.atan2(math.sqrt(aa), math.sqrt(1 - aa))
    return EARTH_RADIUS * c


def segment_30_metres_away(
        points: List[List[float]]) -> Tuple[str, MapPosition, MapPosition]:
    """
    Given a list of the coordinates of the route, it finds the ones that
    are the start and end points of the segment that is 30 metres away.
    """
    distance: float = 0
    len_points = len(points)
    if len_points < 2:
        return 'Less than two points in the list.', None, None
    for i, point in enumerate(points[1:]):
        stop = MapPosition(latitude=point[0], longitude=point[1])
        start = MapPosition(latitude=points[i][0], longitude=points[i][1])
        length = distance_between(start, stop)
        distance += length
        if distance > 30:
            break
    return None, start, stop


def parse_route(
        raw_route: str) -> Tuple[str, MapPosition, MapPosition]:
    """
    It decodes the json string containing the route plan, and extracts
    the coordinates of the ends of the segment of it that is 30 metres
    away from the start.
    """
    route_as_dict = json.loads(raw_route)
    if 'code' not in route_as_dict:
        return "No 'code' key in route string.", None, None
    if route_as_dict['code'] != 'Ok':
        return ('Route code is {}'.format(route_as_dict['code']),
                None,
                None)
    points = route_as_dict['routes'][0]['geometry']['coordinates']
    result = segment_30_metres_away(points)
    return result


def parse_snap_response(raw_response: str) -> MapPosition:
    """ It extracts the first location from the snap response. """
    as_json = json.loads(raw_response)
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
    print('diff is {}'.format(diff))
    err, angle = vector_angle_to_north(diff)
    print('angle is {}'.format(angle))
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
