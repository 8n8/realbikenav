"""
It loads the bike navigation data and trains the neural network with it.
"""

import json
import math
import os
import sys
from typing import List, Set, Tuple

import cv2  # type: ignore
import keras  # type: ignore
from mypy_extensions import TypedDict
import numpy as np  # type: ignore


class State(TypedDict):
    """ The state that is carried around between iterations. """
    unused_directories: Set[str]
    current_directory: str


def carry_on_running(state: State) -> bool:
    """ The condition for the trainer to keep running. """
    return state['unused_directories'] != set()


class Photo(TypedDict):
    """ One photo as a numpy array and its filename. """
    photo: 'np.ndarray[np.uint8]'
    filename: str


class RawDataBatch(TypedDict):
    """
    It represents a batch of training data read in from disc before
    parsing.  The json_data string is the contents of the text data
    file.
    """
    json_data: str
    photos: List[Photo]
    not_enough_data: bool


def read_batch(state: State) -> RawDataBatch:
    """ It reads in one batch of data. """
    data_file_path = 'realData/{}/sensor_readings.dat'.format(
        state['current_directory'])
    with open(data_file_path, 'r') as data_file_handle:
        json_data = data_file_handle.read()

    photo_file_names = os.listdir(
        'realData/{}/photos'.format(state['current_directory']))
    photos: List[Photo] = [
        {'photo': cv2.imread(
            'realData/{}/photos/{}'.format(
                state['current_directory'], photo_file_name),
            cv2.IMREAD_GRAYSCALE),
         'filename': photo_file_name}
        for photo_file_name in photo_file_names]

    return {
        'json_data': json_data,
        'photos': photos,
        'not_enough_data': len(photo_file_names) < 16}


class ParsedDataBatch(TypedDict):
    """ A batch of data ready to feed into the neural network. """
    velocity_change: 'np.ndarray[np.float32]'
    target_direction: 'np.ndarray[np.float32]'
    photos: 'np.ndarray[np.uint8]'


def update_state(state: State) -> State:
    """ It calculates the next state given the current one. """
    directory = next(iter(state['unused_directories']))
    return {
        'current_directory': directory,
        'unused_directories': state['unused_directories'] - {directory}}


def parse_data_batch(raw: RawDataBatch) -> ParsedDataBatch:
    """
    It converts the raw input data into numpy arrays suitable for feeding
    into the neural network trainer.  There are 5 or 6 frames per second.
    The GPS runs at about 1 reading per second.
    """
    lines_of_json = [
        json.loads(line) for line in raw['json_data'].splitlines()][5:]
    directions = [float(j['microbit']['compass']) for j in lines_of_json]
    direction_changes = [
        new - old for new, old in zip(directions[10:], directions[:-10])]
    normalised_dir_changes = [d/2*math.pi for d in direction_changes]
    speeds = [float(j['speed']) for j in lines_of_json]
    speed_changes = [
        new - old for new, old in zip(speeds[10:], directions[:-10])]
    normalised_speed_changes = [s/20 for s in speed_changes]
    target_directions = [j['target_direction'] for j in lines_of_json][:-10]
    return {
        'target_direction': np.array(target_directions),
        'velocity_change': np.stack(  # type: ignore
            (np.array(normalised_speed_changes),
             np.array(normalised_dir_changes)),
            axis=1),
        'photos':
            np.stack(  # type: ignore
                (np.array([p['photo'] for p in raw['photos'][5:-10]]),
                 np.array([p['photo'] for p in raw['photos'][:-15]])),
                axis=3)}


def data_loader():
    """
    It is a generator that loads the training data.  It has to be a
    generator, rather than loading it all in one go, because there is
    too much data to fit in memory.
    """
    data_dirs = os.listdir('realData')
    state: State = {
        'unused_directories': set(data_dirs),
        'current_directory': data_dirs[0]}
    while carry_on_running(state):
        raw_data_batch = read_batch(state)
        if not raw_data_batch['not_enough_data']:
            parsed_data = parse_data_batch(raw_data_batch)
            print('Batch size is {}'.format(sys.getsizeof(parsed_data)))
            yield ({'photos': parsed_data['photos'],
                    'target_direction': parsed_data['target_direction']},
                   parsed_data['velocity_change'])
        state = update_state(state)


def load_network():
    """
    It makes the neural network.  The neural net architecture is:

    input) 480 x 640 x 2
    1) 2d convolution, 3 layers, 5x5 kernel, stride 2
    2) 2d convolution, 5 layers, 5x5 kernel, stride 2
    3) 2d convolution, 5 layers, 5x5 kernel, stride 2
    4) 2d convolution, 5 layers, 3x3 kernel, stride 1
    5) 2d convolution, 5 layers, 3x3 kernel, stride 1
    6) 1000 neuron fc layer
    7) 100 neuron fc layer
    8) 50 neuron fc layer
    9) 10 neuron fc layer
    10) 2 neuron output, one for speed and one for target direction
    """
    dense = keras.layers.Dense
    bn = keras.layers.BatchNormalization
    conv = keras.layers.convolutional.Conv2D
    flat = keras.layers.core.Flatten

    photos = keras.layers.Input(shape=(480, 640, 2), name='photos')
    layer1 = conv(3, 5, strides=(2, 2), activation='relu')
    layer2 = conv(5, 5, strides=(2, 2), activation='relu')
    layer3 = conv(5, 5, strides=(2, 2), activation='relu')
    layer4 = conv(5, 3, strides=(1, 1), activation='relu')
    layer5 = conv(5, 3, strides=(1, 1), activation='relu')
    conv_out = flat()(layer5(bn()(layer4(bn()(layer3(bn()(layer2(bn()(
        layer1(bn()(photos)))))))))))
    target_direction = keras.layers.Input(
        shape=(1,), name='target_direction')
    dense_input = keras.layers.concatenate(
        [conv_out, target_direction])
    layer6 = dense(1000, activation='relu')
    layer7 = dense(100, activation='relu')
    layer8 = dense(50, activation='relu')
    layer9 = dense(10, activation='relu')
    layer10 = dense(2, activation='softmax')
    dense_out = layer10(bn()(layer9(bn()(layer8(bn()(layer7(bn()(layer6(
        dense_input)))))))))
    return keras.models.Model(
        inputs=[photos, target_direction],
        outputs=[dense_out])


def find_number_of_data_batches_and_points() -> Tuple[int, int]:
    """ It counts the number of data points in the whole data batch. """
    batch_directories = os.listdir('realData')
    batches = 0
    data_points = 0
    for directory in batch_directories:
        number_of_photos_in_batch = len(os.listdir(
            'realData/{}/photos'.format(directory)))
        data_points_in_batch = max(0, number_of_photos_in_batch - 15)
        if data_points_in_batch > 0:
            batches += 1
            data_points += data_points_in_batch
    return batches, data_points


def main():
    """
    It loads the neural network, loads the data, and trains the net.
    """
    num_batches, num_data_points = find_number_of_data_batches_and_points()
    print("Number of data points is {}.".format(num_data_points))
    print("Number of data batches is {}.".format(num_batches))
    neural_network = load_network()
    neural_network.compile(
        loss='categorical_crossentropy',
        optimizer=keras.optimizers.Adam(),
        metrics=['accuracy'])
    neural_network.fit_generator(
        data_loader(), steps_per_epoch=num_batches)
    neural_network.save('nav_net.h5')


main()
