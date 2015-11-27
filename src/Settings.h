#ifndef SETTINGS_H
#define SETTINGS_H

// Tasks configurations
#define TOTAL_TASKS 4
enum {
	T_LEDS,
    T_TONE,
	T_TOOGLER,
    T_SENSOR
};

// Timeout configurations
#define LED_TIMEOUT      30000 // 3000ms
#define DEBOUNCE_TIMEOUT 2000  // 200ms
#define TIMER_TIMEOUT    1    // 1ms (1Hz Tone)

// Definitions (ENUMS and DEFINES)
#define TOTAL_EVENTS 1
typedef enum {
	EVT_BTN_PRESSED = 1
} Events;

typedef enum {
	COLOR_RED,
	COLOR_GREEN,
	COLOR_BLUE
} LedStates;

typedef enum {
    REAR_BOX_DISENGAGED,
    REAR_BOX_ENGAGED,
} RearBoxStates;

#endif
