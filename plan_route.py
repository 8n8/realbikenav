"""
It calculates the direction to travel in to get to a destination on the
road map, given the coordinates of the start and end points.
"""


import json
import math
from typing import List, Dict, NamedTuple, Tuple, Union
import numpy
import psycopg2  # type: ignore
import psycopg2.extras  # type: ignore
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


Num = Union[int, float]


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


def calculate_direction(raw_route: str) -> Tuple[str, float]:
    """
    It calculates the bearing along the first segment of the route.
    """
    print(raw_route)
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


def get_database_cursor():
    """ It connects to the database and returns the cursor. """
    connection = psycopg2.connect(
        "host='localhost' dbname='routing' user='t'")
    return connection.cursor(cursor_factory=psycopg2.extras.DictCursor)


def table_to_numpy_array(
        sql_table: List[Dict[str, Num]]) -> Dict[str, 'numpy.ndarray[Num]']:
    """
    It converts an SQL table in the form of a list of dictionaries, with
    a key for each column, into a dictionary of numpy arrays, one for
    each column.
    """
    return {key: numpy.array([float(d[key]) for d in sql_table])
            for key in sql_table[0].keys()}