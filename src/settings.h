#ifndef SETTINGS_H
#define SETTINGS_H

// Tasks configurations
#define TOTAL_TASKS 2
enum {
	T_BEEP,
	T_LEDS
};

// Timeout configurations
#define LED_TIMEOUT      300 // 3000ms / 10ms (ticks)
#define DEBOUNCE_TIMEOUT 20  // 200ms

// Definitions (ENUMS and DEFINES)
enum {
	LED_BLINKING,
	LED_NOT_BLINKING
};

enum {
	EVT_BTN_PRESSED = 1
};

enum {
	COLOR_RED,
	COLOR_GREEN,
	COLOR_BLUE
};

#endif
