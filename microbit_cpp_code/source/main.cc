#include "MicroBit.h"

MicroBit uBit;

struct leds {
    bool new_destination;
    int direction;
    int recording_state;
    bool error_state;
};

struct accel {
    int x;
    int y;
    int z;
};

// It represents the data for sending to the output
struct State {
    leds display;
    int compass;
    accel acceleration;
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
    leds display;
    int compass;
    accel acceleration;
    bool buttonA;
    bool buttonB;
};


// It converts the display byte received from the serial port into
// the led struct.
void parse_display(const char &raw_display, leds &display) {
    display.new_destination = raw_display & 1;
    display.direction = (raw_display >> 1) & 15;
    display.recording_state = (raw_display >> 5) & 3;
    display.error_state = (raw_display >> 7) & 1;
}

// It reads all the inputs.
void read_input(InputData &input_data) {
    input_data.new_input = uBit.serial.readable();
    parse_display(uBit.serial.read(1, ASYNC).charAt(0), input_data.display);
    input_data.compass = uBit.compass.heading();
    input_data.acceleration.x = uBit.accelerometer.getX();
    input_data.acceleration.y = uBit.accelerometer.getY();
    input_data.acceleration.z = uBit.accelerometer.getZ();
    input_data.buttonA = uBit.buttonA.isPressed();
    input_data.buttonB = uBit.buttonB.isPressed();
}
    

void update_state(State &state, const InputData &input_data) {
    if (input_data.new_input) {
        state.display = input_data.display;
    }
    state.compass = input_data.compass;
    state.acceleration = input_data.acceleration;
    state.buttonAcounter = 
        (state.buttonAcounter < 10000) * 
        (state.buttonAcounter + input_data.buttonA);
    state.buttonBcounter =
        (state.buttonBcounter < 10000) *
        (state.buttonBcounter + input_data.buttonB);
}


void sendInt(const int &the_int) {
    uBit.serial.sendChar(the_int & 0xff);
    uBit.serial.sendChar((the_int >> 8) & 0xff);
    uBit.serial.sendChar((the_int >> 16) & 0xff);
    uBit.serial.sendChar((the_int >> 24) & 0xff);
}


void send_output(const State &state) {
    sendInt(state.compass);
    sendInt(state.acceleration.x);
    sendInt(state.acceleration.y);
    sendInt(state.acceleration.z);
    sendInt(state.buttonAcounter);
    sendInt(state.buttonBcounter);
}


int main() {
    // Initialise the micro:bit runtime.
    uBit.init();

    // Insert your code here!
    uBit.display.scroll("HELLO WORLD! :)");

    State state;
    state.display = {false, 0, 0, false};
    state.compass = 0;
    state.acceleration = {0, 0, 0};
    state.buttonAcounter = 0;
    state.buttonBcounter = 0;

    InputData input_data;

    while(true) {
        send_output(state);
        read_input(input_data);
        update_state(state, input_data);
    }

    // If main exits, there may still be other fibers running or
    // registered event handlers etc.
    // Simply release this fiber, which will mean we enter the scheduler.
    // Worse case, we then sit in the idle task forever, in a power
    // efficient sleep.
    release_fiber();
}
