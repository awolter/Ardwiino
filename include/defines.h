#pragma once

#include <stdint.h>

#define NONE 0
#define INVALID_PIN 0xFF

#define XBOX_WHAMMY XBOX_R_X
#define XBOX_TILT XBOX_R_Y

// https://docs.microsoft.com/en-us/windows/win32/xinput/xinput-and-controller-subtypes
// http://forum.gimx.fr/viewtopic.php?f=11&t=2897&start=10
#define XINPUT_GAMEPAD 1
#define XINPUT_WHEEL 2
#define XINPUT_ARCADE_STICK 3
#define XINPUT_FLIGHT_STICK 4
#define XINPUT_DANCE_PAD 5
#define XINPUT_GUITAR 6
#define XINPUT_GUITAR_ALTERNATE 7
#define XINPUT_LIVE_GUITAR 8
#define XINPUT_DRUMS 9
#define XINPUT_ARCADE_PAD 19
#define XINPUT_TURNTABLE 23

#define GAMEPAD 1
#define WHEEL 2
#define ARCADE_STICK 3
#define FLIGHT_STICK 4
#define DANCE_PAD 5
#define ARCADE_PAD 6
#define GUITAR 7
#define LIVE_GUITAR 8
#define DRUMS 9
#define DJ_HERO_TURNTABLE 10

#define GUITAR_HERO 0
#define ROCK_BAND 1

#define UNIVERSAL 0
#define KEYBOARD_MOUSE 1
#define MIDI 2

#define XBOX360 3
#define PS3 4
#define WII_RB 5
#define SWITCH 6

#define PINMODE_PULLUP 0
#define PINMODE_PULLDOWN 1
#define PINMODE_FLOATING 2
#define PINMODE_BUS 3

#define LEDS_DISABLED 0 
#define LEDS_APA102 1
#define LEDS_WS2812 2

#define MIDI_TYPE_NOTE 0
#define MIDI_TYPE_CONTROL_COMMAND 1

// Right Motor
#define RUMBLE_TURNTABLE_START 0x1F
#define RUMBLE_TURNTABLE_END 0xFF

// Left Motor
#define RUMBLE_GUITAR_COMBO_1 0x1F
#define RUMBLE_GUITAR_COMBO_2 0x20
#define RUMBLE_GUITAR_COMBO_3 0x21
#define RUMBLE_GUITAR_COMBO_4 0x22
#define RUMBLE_GUITAR_OPEN 0x23
#define RUMBLE_GUITAR_START_POWER_FILL_START 0x20
#define RUMBLE_GUITAR_START_POWER_FILL_END 0x40
#define RUMBLE_GUITAR_START_POWER_ACTIVE_START 0x40
#define RUMBLE_GUITAR_START_POWER_ACTIVE_END 0x60
typedef enum {
    WII_NUNCHUK = 0x0000,
    WII_CLASSIC_CONTROLLER = 0x0001,
    WII_CLASSIC_CONTROLLER_PRO = 0x0101,
    WII_THQ_UDRAW_TABLET = 0xFF12,
    WII_UBISOFT_DRAWSOME_TABLET = 0xFF13,
    WII_GUITAR_HERO_GUITAR_CONTROLLER = 0x0003,
    WII_GUITAR_HERO_DRUM_CONTROLLER = 0x0103,
    WII_DJ_HERO_TURNTABLE = 0x0303,
    WII_TAIKO_NO_TATSUJIN_CONTROLLER = 0x0011,
    WII_MOTION_PLUS = 0x0005,
    WII_NO_EXTENSION = 0x180b,
    WII_NOT_INITIALISED = 0xFFFF
} WiiExtType_t;
typedef enum {
    PSX_UNKNOWN_CONTROLLER = 0,
    PSX_DIGITAL,
    PSX_DUALSHOCK_1_CONTROLLER,
    PSX_DUALSHOCK_2_CONTROLLER,
    PSX_GUITAR_HERO_CONTROLLER,
    PSX_NEGCON,
    PSX_JOGCON,
    PSX_GUNCON,
    PSX_FLIGHTSTICK,
    PSX_MOUSE,
    PSX_NO_DEVICE
} PsxControllerType_t;

#define XBOX_BTN_COUNT 16
#define XBOX_AXIS_COUNT 6
// Standard controller structure. axis' and triggers are seperated for easy modification
typedef struct {
    uint16_t buttons;
    uint8_t lt;
    uint8_t rt;
    int16_t l_x;
    int16_t l_y;
    int16_t r_x;
    int16_t r_y;
} Controller_t;
typedef struct {
    uint16_t buttons;
    uint8_t lt;
    uint8_t rt;
    int16_t l_x;
    int16_t l_y;
    uint16_t whammy;
    int16_t r_y;
} Guitar_t;

// Combined controller structure. Axis' and triggers are combined into arrays to allow for easy modification by direct
typedef struct {
    uint16_t buttons;
    uint8_t triggers[2];
    int16_t sticks[4];
} ControllerCombined_t;
enum ControllerAxisPS3 {
    PS3_DPAD_UP,
    PS3_DPAD_RIGHT,
    PS3_DPAD_LEFT,
    PS3_DPAD_DOWN,
    PS3_L2,
    PS3_R2,
    PS3_L1,
    PS3_R1,
    PS3_TRIANGLE,
    PS3_CIRCLE,
    PS3_CROSS,
    PS3_SQUARE
};
enum ControllerAxis {
    XBOX_LT,
    XBOX_RT,
    XBOX_L_X,
    XBOX_L_Y,
    XBOX_R_X,
    XBOX_R_Y
};
// Buttons used by all controllers
enum ControllerButtons {
    XBOX_DPAD_UP,
    XBOX_DPAD_DOWN,
    XBOX_DPAD_LEFT,
    XBOX_DPAD_RIGHT,
    XBOX_START,
    XBOX_BACK,
    XBOX_LEFT_STICK,
    XBOX_RIGHT_STICK,

    XBOX_LB,
    XBOX_RB,
    XBOX_HOME,
    XBOX_UNUSED,
    XBOX_A,
    XBOX_B,
    XBOX_X,
    XBOX_Y,
};

enum SwitchButtons {
    SWITCH_Y,
    SWITCH_B,
    SWITCH_A,
    SWITCH_X,
    SWITCH_L,
    SWITCH_R,
    SWITCH_ZL,
    SWITCH_ZR,
    SWITCH_SELECT,
    SWITCH_START,
    SWITCH_LEFT_STICK,
    SWITCH_RIGHT_STICK,
    SWITCH_HOME,
    SWITCH_CAPTURE,
};
enum PS3Buttons {
    PS3_TRIANGLE_BT,
    PS3_CIRCLE_BT,
    PS3_CROSS_BT,
    PS3_SQUARE_BT,
    PS3_L2_BT,
    PS3_R2_BT,
    PS3_L1_BT,
    PS3_R1_BT,
    
    PS3_SELECT_BT,
    PS3_START_BT,
    PS3_LEFT_STICK_BT,
    PS3_RIGHT_STICK_BT,
    PS3_PS_BT,
    
    // The HAT doesn't treat the dpad as buttons, but its easier to do that and then transform the hat later.
    PS3_DPAD_UP_BT=0x10,
    PS3_DPAD_DOWN_BT,
    PS3_DPAD_LEFT_BT,
    PS3_DPAD_RIGHT_BT,
};