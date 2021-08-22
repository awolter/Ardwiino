#include "lib_main.h"

#include <stddef.h>
#include <string.h>

#include "defines.h"
#include "usb/controller_reports.h"
#include "i2c.h"
#include "spi_impl.h"
#include "leds.h"
#define PIN_COUNT PORTS* PINS_PER_PORT
#ifdef __AVR__ 
    #define SPI_FREQ F_CPU / 2
#else
    #define SPI_FREQ 8000000
#endif
ConsoleType_t consoleType;
DeviceType_t deviceType;
InputType_t inputType;
TiltType_t tiltType;
FretLedMode_t ledMode;
bool guitar;
bool drum;
uint8_t pinCount;
Input_t pins[PIN_COUNT];
AnalogInput_t analogInfo[NUM_ANALOG_INPUTS];
uint32_t lastMillisStrum;
bool mergedStrum = false;
uint8_t inputCount;
void (*tickInput)(void) = NULL;
// Inside leds there is a colour and a forceColor
// if forceColor is set or the button is pressed, then the colour is set to color
// otherwise, it is cleared
Input_t* ledOrder[PIN_COUNT] = {};
void init() {
    ledMode = NONE;
    inputType = DIRECT;
    consoleType = XBOX360;
    deviceType = GUITAR_HERO_GUITAR;
    guitar = deviceType == GUITAR_HERO_GUITAR || deviceType == ROCK_BAND_GUITAR || deviceType == GUITAR_HERO_LIVE_GUITAR;
    drum = deviceType == GUITAR_HERO_DRUMS || deviceType == ROCK_BAND_DRUMS;
    tiltType = NONE;
    inputCount = PIN_COUNT;
    memset(pins, 0xFF, sizeof(pins));
    if (tiltType == MPU_6050 || inputType == WII) {
        twi_init();
    }
    if (inputType != PS2 && ledMode == APA102) {
        spi_begin(SPI_FREQ, true, true, false);
    }
    if (inputType == DIRECT) {
        initPins();
    } else if (inputType == WII) {
        // Init wii, load in the wii inputs as the "pins" instead
        // One interesting thing is we should be able to just load in tilt as a standard pin here too.
    } else if (inputType == PS2) {
        // Init ps2, load in the ps2 inputs as the "pins" instead
    }
}

uint32_t last = 0;
bool val = true;
uint8_t tick(uint8_t* data) {
    if (tickInput) {
        tickInput();
    }
    uint8_t* triggers;
    int16_t* joys = NULL;
    uint8_t* joys8 = NULL;
    uint8_t* buttons;
    size_t size;
    if (consoleType == KEYBOARD_MOUSE) {
        // if mouse has changed use mouse report, else use keyboard report.
        // Or, just alternate between reports
        TUSB_KeyboardReport_Data_t* report = (TUSB_KeyboardReport_Data_t*)data;
        report->rid = REPORT_ID_KBD;
        // bit_write(val, report->KeyCodeFlags[THID_KEYBOARD_SC_A / 8], THID_KEYBOARD_SC_A % 8);
        // bit_write(val, report->Modifier, (THID_KEYBOARD_MODIFIER_LEFTSHIFT));
        size = sizeof(TUSB_KeyboardReport_Data_t);
    } else if (consoleType == XBOX360) {
        USB_XInputReport_Arr_Data_t* report = (USB_XInputReport_Arr_Data_t*)data;
        report->rid = 0;
        report->rsize = sizeof(USB_XInputReport_Arr_Data_t);
        size = sizeof(USB_XInputReport_Data_t);
        triggers = report->triggers;
        joys = report->joys;
        buttons = (uint8_t*)&report->buttons;
    } else if (consoleType == MIDI) {
        // This one will need something specific as it isnt a controller.
    } else {
        USB_PS3Report_Arr_Data_t* report = (USB_PS3Report_Arr_Data_t*)data;
        report->rid = 0;
        size = sizeof(USB_PS3Report_Arr_Data_t);
        joys8 = report->joys;
        triggers = report->axis + PS3_L2;
        buttons = (uint8_t*)&report->buttons;
    }
    for (int i = 0; i < inputCount; i++) {
        Input_t* pin = pins + i;
        if (pin->binding != 0xFF) {
            if (pin->axisInfo && !pin->axisInfo->isADC) {
                int16_t val = pin->axisInfo->analogRead(pin);
                uint8_t val2 = (val >> 8) + 127;
                if (pin->binding <= XBOX_RT) {
                    triggers[pin->binding] = val2;
                } else if (joys) {
                    joys[pin->binding - XBOX_RT] = val;
                } else {
                    joys8[pin->binding] = val2;
                }
            } else {
                bool val = pin->digitalRead(pin);
                uint32_t current = millis();
                uint32_t last = pin->lastMillis;
                // TODO: this is only XPAD_DPAD_DOWN for xinput. we will just need to store the correct binding somewhere.
                bool usingMerged = mergedStrum && (pin->binding == XBOX_DPAD_DOWN || pin->binding == XBOX_DPAD_UP);
                if (usingMerged) {
                    last = lastMillisStrum;
                }
                if (current - last > pin->milliDeBounce) {
                    // >>3 is equiv to /8 (2^3), %8 is equiv to & 7 (8-1)
                    bit_write(val, buttons[pin->binding >> 3], pin->binding & 7);
                    pin->lastMillis = current;
                    if (usingMerged) {
                        lastMillisStrum = current;
                    }
                }
            }
        }
    }
    if (ledMode == APA102) {
        tickLEDs(ledOrder);
    }
    return size;
}
void packetReceived(uint8_t* data, uint8_t len) {
    if (consoleType == XBOX360) {
        if (data[0] == 0x01) {
            XInputLEDReport_t* leds = (XInputLEDReport_t*)data;
            (void)leds;
            // We can receive led reports here and do things with them
        } else if (data[0] == 0x00) {
            XInputRumbleReport_t* rumble = (XInputRumbleReport_t*)data;
            (void)rumble;
            // We can receive rumble reports here and do things with them
        }
    } else if (consoleType == PS3 || consoleType == WII_RB || consoleType == SWITCH) {
        ps3_output_report* report = (ps3_output_report*)data;
        (void)report;
    }
}