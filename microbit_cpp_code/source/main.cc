// It runs on the microbit.  It receives information about what to
// display on the LEDs from the parent computer via a USB serial port.
// It reads the sensors and sends the readings to the parent computer.
// It runs continuously.


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


// It represents all the input readings after parsing.
struct InputData {
    leds display;
    int compass;
    accel acceleration;
    bool buttonA;
    bool buttonB;
};


// It converts the display byte received from the serial port into
// the led struct.
void parse_display(const char &raw_display, leds &display) {
    ManagedString disp(raw_display);
    uBit.display.scroll(disp);
    display.new_destination = raw_display & 1;
    display.direction = (raw_display >> 1) & 15;
    display.recording_state = (raw_display >> 5) & 3;
    display.error_state = (raw_display >> 7) & 1;
}


// It reads all the inputs.
void read_input(InputData &input_data) {
    parse_display(
        uBit.serial.read(1).charAt(0),
        input_data.display);
    // ManagedString readable(uBit.serial.isReadable());
    // uBit.display.scroll(readable);
    // if (uBit.serial.isReadable() == 1) {
    //     ManagedString reading(uBit.serial.read(1, ASYNC));
    //     uBit.display.scroll("a");
    // } else {
    //     return;
    // }
    input_data.compass = uBit.compass.heading();
    input_data.acceleration.x = uBit.accelerometer.getX();
    input_data.acceleration.y = uBit.accelerometer.getY();
    input_data.acceleration.z = uBit.accelerometer.getZ();
    input_data.buttonA = uBit.buttonA.isPressed();
    input_data.buttonB = uBit.buttonB.isPressed();
}
    

// It updates the state, given the new input data from the sensors
// and the parent computer.
void update_state(State &state, const InputData &input_data) {
    state.display = input_data.display;
    state.compass = input_data.compass;
    state.acceleration = input_data.acceleration;
    state.buttonAcounter = 
        (state.buttonAcounter < 10000) * 
        (state.buttonAcounter + input_data.buttonA);
    state.buttonBcounter =
        (state.buttonBcounter < 10000) *
        (state.buttonBcounter + input_data.buttonB);
}


// It sends an int to the parent computer.
void sendInt(const int &the_int) {
    uBit.serial.sendChar(the_int & 0xff);
    uBit.serial.sendChar((the_int >> 8) & 0xff);
    uBit.serial.sendChar((the_int >> 16) & 0xff);
    uBit.serial.sendChar((the_int >> 24) & 0xff);
}


// It sends the message to the parent computer.
void send_message_to_laptop(const State &state) {
    ManagedString compass(state.compass);
    ManagedString accelx(state.acceleration.x);
    ManagedString accely(state.acceleration.y);
    ManagedString accelz(state.acceleration.z);
    ManagedString buttonAcounter(state.buttonAcounter);
    ManagedString buttonBcounter(state.buttonBcounter);
    ManagedString space(" ");
    ManagedString line_end("\n");
    ManagedString message =
        compass + space +
        accelx + space +
        accely + space +
        accelz + space +
        buttonAcounter + space +
        buttonBcounter + line_end;
    uBit.serial.send(message);
}


// It calculates the led number (0 - 24) from the direction code
// (0 - 15).
//
// The direction codes are:
// 14 15 0  1  2
// 13          3
// 12          4
// 11          5
// 10 9  8  7  6
//
// The led codes are:
// 0  1  2  3  4
// 5  6  7  8  9
// 10 11 12 13 14
// 15 16 17 18 19
// 20 21 22 23 24
int direction2led_num(const int &direction) {
    switch(direction) {
        case 0: return 2;
        case 1: return 3;
        case 2: return 4;
        case 3: return 9;
        case 4: return 14;
        case 5: return 19;
        case 6: return 24;
        case 7: return 23;
        case 8: return 22;
        case 9: return 21;
        case 10: return 20;
        case 11: return 15;
        case 12: return 10;
        case 13: return 5;
        case 14: return 0;
        case 15: return 1;
    }
}


// It lights up the leds, given the display information.
void light_up_leds(const leds &display) {
    bool led_on_off[25] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0};
    led_on_off[13] = display.new_destination;
    led_on_off[direction2led_num(display.direction)] = 1;
    switch(display.recording_state) {
        case 0: break;
        case 1:
            led_on_off[11] = 1;
            break;
        case 2:
            led_on_off[0] = 1;
            led_on_off[1] = 1;
            led_on_off[2] = 1;
            led_on_off[3] = 1;
            led_on_off[4] = 1;
            led_on_off[5] = 1;
            led_on_off[6] = 1;
            led_on_off[7] = 1;
            led_on_off[8] = 1;
            led_on_off[9] = 1;
            led_on_off[10] = 1;
            led_on_off[11] = 1;
            led_on_off[12] = 1;
            led_on_off[13] = 1;
            led_on_off[14] = 1;
            led_on_off[15] = 1;
    }
    if (display.error_state) {
        for (int i = 0; i < 25; i++) {
            led_on_off[i] = 1;
        }
    }
    MicroBitImage image(5, 5);
    int x, y;
    for (int i = 0; i < 25; i++) {
        x = i % 5;
        y = i / 5;
        image.setPixelValue(x, y, led_on_off[i] * 255);
    }
    uBit.display.print(image);
}


// It sends all the outputs.  It sends the message to the parent computer
// down the serial port, and updates the LED display.
void send_output(const State &state) {
    send_message_to_laptop(state);
    light_up_leds(state.display);
}


int main() {
    // Initialise the micro:bit runtime.
    uBit.init();

    // Insert your code here!
    // uBit.display.scroll("HELLO WORLD! :)");
    uBit.compass.calibrate();

    State state;
    state.display = {false, 0, 0, false};
    state.compass = 0;
    state.acceleration = {0, 0, 0};
    state.buttonAcounter = 0;
    state.buttonBcounter = 0;

    InputData input_data;
    input_data.display.new_destination = false;
    input_data.display.direction = 0;
    input_data.display.recording_state = 0; 
    input_data.display.error_state = false;
    input_data.compass = 0;
    input_data.acceleration.x = 0;
    input_data.acceleration.y = 0;
    input_data.acceleration.z = 0;
    input_data.buttonA = false;
    input_data.buttonB = false;

    while(true) {
        read_input(input_data);
        update_state(state, input_data);
        send_output(state);
    }

    // If main exits, there may still be other fibers running or
    // registered event handlers etc.
    // Simply release this fiber, which will mean we enter the scheduler.
    // Worse case, we then sit in the idle task forever, in a power
    // efficient sleep.
    release_fiber();
}
