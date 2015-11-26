#ifndef SETTINGS_H
#define SETTINGS_H

enum {
	OSC_OFF,
	OSC_ON,
};

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


#define TOTAL_TASKS 2
enum {
	T_BEEP,
	T_LEDS
};

#define LED_TIMEOUT      300 // 3000ms / 10ms (ticks)
#define DEBOUNCE_TIMEOUT 20  // 200ms
#endif
