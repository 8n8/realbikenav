#include "MicroBit.h"

MicroBit uBit;

// It represents the data for sending to the output
struct OutputSlashState {
    bool[16] display;
    int compass;
    int[3] acceleration;
    int buttonAcounter;
    int buttonBcounter;
};

// It represents all the raw inputs.
// struct RawInput {
//     bool new_input;
//     char display;
//     int compass;
//     int[3] acceleration;
//     bool buttonA;
//     bool buttonB;
// };

// It represents all the input readings after parsing.
struct InputData {
    bool new_input;
    bool[16] display;
    int direction;
    int[3] acceleration;
    bool buttonA;
    bool buttonB;
};

bool[16] parse_display(char raw_display) {
    int direction = (raw_display >> 1) & 15;
}

// It reads all the inputs.
void read_input(InputData &input_data) {
    input_data.new_input = uBit.serial.readable();
    input_data.display = parse_display(uBit.serial.read(uBit.ASYNC));
    input_data.compass = uBit.compass.heading();
    input_data.acceleration[0] = uBit.accelerometer.getX();
    input_data.acceleration[1] = uBit.accelerometer.getY();
    input_data.acceleration[2] = uBit.accelerometer.getZ();
    input_data.buttonA = uBit.buttonA.wasPressed();
    input_data.buttonB = uBit.buttonB.wasPressed();
}
    



int main() {
    // Initialise the micro:bit runtime.
    uBit.init();

    // Insert your code here!
    uBit.display.scroll("HELLO WORLD! :)");

    StateSlashOutput state;
    state.display = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    state.compass = 0;
    state.acceleration = {0, 0, 0};
    state.buttonAcounter = 0;
    state.buttonBcounter = 0;

    InputData input_data;

    while(True) {
        send_output(state);
        read_input(input_data);
        update_state(state, raw_input_data)

    // If main exits, there may still be other fibers running or
    // registered event handlers etc.
    // Simply release this fiber, which will mean we enter the scheduler.
    // Worse case, we then sit in the idle task forever, in a power
    // efficient sleep.
    release_fiber();
}

