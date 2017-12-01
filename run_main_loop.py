import main_loop
import cv2
import serial
import time
import plan_route

initital_state: main_loop.State = main_loop.State(
    data_directory='realData/{}'.format(time.time()),
    button_A_was_on_last_cycle=False,
    destination=main_loop.random_destination(),
    recording_state=main_loop.RecordingState.OFF_PAUSED,
    brand_new_destination=False)

main_loop.main(
    initial_state, main_loop.state_updater, main_loop.state2view)
