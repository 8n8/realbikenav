"""
It calculates the direction to travel in to get to a destination on the
road map, given the coordinates of the start and end points.
"""

import math
from typing import List, Dict, NamedTuple, Tuple, Union
import numpy
import psycopg2  # type: ignore
import psycopg2.extras  # type: ignore


# It gets the list of node id numbers and their latitudes and longitudes.
SQL_STRING_FOR_NODES: str = 'SELECT id, lat, lon FROM ways_vertices_pgr;'


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
    database_cursor = get_database_cursor()
    database_cursor.execute(SQL_STRING_FOR_NODES)
    id_table = table_to_numpy_array(database_cursor.fetchall())
    route = []
    while route == []:
        start_node = nearest_node_to(start, id_table)
    print(start_node)
    end_node = nearest_node_to(end, id_table)
    print(end_node)
    _, sql_string_for_route = make_sql_string_for_route(
        start_node, end_node)
    database_cursor.execute(sql_string_for_route)
    route = database_cursor.fetchall()
    return calculate_direction(route[:2], id_table)


def vector_angle_to_north(v: MapPosition) -> Tuple[str, float]:
    """
    It calculates the angle of the vector relative to (0, 1).  The angle
    is between 0 and 2π radians.
    """
    magnitude = (v.longitude**2 + v.latitude**2)**0.5
    if math.isclose(magnitude, 0):
        return "Vector has zero magnitude.", None
    angle = math.acos(v.latitude / magnitude)
    if v.longitude < 0:
        angle = 2*math.pi - angle
    return None, angle


Num = Union[int, float]


def calculate_direction(
        route: List[Dict[str, Num]],
        id_table: Dict[str, 'numpy.ndarray[Num]']) -> Tuple[str, float]:
    """
    It calculates the bearing from the first node in the route to the
    second node.
    """
    node1: int = int(route[0]['node'])
    node2: int = int(route[1]['node'])
    node1index: int = numpy.where(  # type: ignore
        id_table['id'].astype(int) == node1)[0]
    node2index: int = numpy.where(  # type: ignore
        id_table['id'].astype(int) == node2)[0]
    node1longitude: int = id_table['lon'][node1index][0]  # type: ignore
    node1latitude: int = id_table['lat'][node1index][0]  # type: ignore
    node2longitude: int = id_table['lon'][node2index][0]  # type: ignore
    node2latitude: int = id_table['lat'][node2index][0]  # type: ignore
    diff: MapPosition = MapPosition(
        longitude=node2longitude - node1longitude,  # type: ignore
        latitude=node2latitude - node1latitude)  # type: ignore
    err, angle = vector_angle_to_north(diff)
    if err is not None:
        return "The first two nodes in the route are identical.", None
    return None, angle


def nth_nearest_node_to(
        n: int,
        position: MapPosition,
        id_table: Dict[str, 'numpy.ndarray[Num]']) -> int:
    """
    It finds the id number of the node on the map that is closest but 'n'
    to the given position.  So if 'n' is zero then it will find the
    closest, and if it is 1 it will find the next closests.  The third
    argument is the table from the database containing in each row the ID
    number of a node with its corresponding latitude and longitude.  It
    has columns 'id', 'lat' and 'lon'.
    """
    nth_smallest_index: int = int(numpy.argpartition(  # type: ignore
        (id_table['lat'] - position.latitude)**2 +
        (id_table['lon'] - position.longitude)**2, n)[n])
    print('longitude is {} and latitude is {}'.format(
        id_table['lon'][smallest_index],
        id_table['lat'][smallest_index]))
    return int(id_table['id'][smallest_index])  # type: ignore


def make_sql_string_for_route(
        start_node: int,
        destination_node: int) -> Tuple[str, str]:
    """
    It creates an SQL query string that gets the shortest route between
    the start and end nodes.
    """
    if not isinstance(start_node, int):
        return "Start node is not an int.", None
    if not isinstance(destination_node, int):
        return "Destination node is not an int.", None
    return (None, ("SELECT * FROM pgr_dijkstra('SELECT gid AS id, source, "
                   "target, length AS cost From ways', {}, {}, directed := "
                   "true);".format(start_node, destination_node)))


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
