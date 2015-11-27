#ifndef SETTINGS_H
#define SETTINGS_H

// Tasks configurations
#define TOTAL_TASKS 3
enum {
	T_LEDS,
	T_TOOGLER,
	T_TONE
};

// Timeout configurations
#define LED_TIMEOUT      30000 // 3000ms
#define DEBOUNCE_TIMEOUT 2000  // 200ms
#define TIMER_TIMEOUT    1    // 1ms (1Hz Tone)

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
