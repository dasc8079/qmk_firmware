#include QMK_KEYBOARD_H


#define _LAYER0 0
#define _LAYER1 1
#define _LAYER2 2
#define _LAYER3 3
#define _LAYER4 4


/***************************
 * Trackball related defines
 **************************/

/ Motion Sensor Pinout GD -> GND	SS -> F7	SC -> B1	MO -> B2	MI -> B3	VI -> VCC
/ TRRS I2C 1 = D1	TRRS I2C 1 = D0		C(D4	C6	D7	E6	B4	B5	B7)	R(D5	C7	F1	F0	B6)	RESET = RST	VCC = MOTION&TRRS	GND = TRRS&MOTION

/ Diode faces away from hotswap

/ Flash for EEHANDS
/ Elite-C (left): ./util/docker_build.sh handwired/dactyl_manuform/3x5_3:yourusername:dfu-split-left
/ Elite-C (right): ./util/docker_build.sh handwired/dactyl_manuform/3x5_3:yourusername:dfu-split-right

#include "pointing_device.h"
#include "../../pmw3360/pmw3360.h"

uint8_t track_mode = 0; // 0 Mousecursor; 1 arrowkeys/carret; 2 scrollwheel; 3 sound/brightness
#define cursor_mode 0
#define carret_mode 1
#define scroll_mode 2
#define sound_brightness_mode 3
uint8_t prev_track_mode = 0;
bool integration_mode = false;
int16_t cum_x = 0;
int16_t cum_y = 0;
int16_t sensor_x = 0;
int16_t sensor_y = 0;

// Thresholds help to move only horizontal or vertical. When accumulated distance reaches threshold, only move one discrete value in direction with bigger delta.
uint8_t	carret_threshold = 24;		 // higher means slower
uint16_t carret_threshold_inte = 340; // in integration mode higher threshold

#define regular_smoothscroll_factor 8
bool smooth_scroll = true;
uint8_t	scroll_threshold = 8 / regular_smoothscroll_factor;	// divide if started smooth
uint16_t scroll_threshold_inte = 1200 / regular_smoothscroll_factor;

uint16_t cursor_multiplier = 250;	// adjust cursor speed
uint16_t cursor_multiplier_inte = 20;
#define CPI_STEP 20

int16_t cur_factor;

//End traackball Defines


enum custom_keycodes {
    	LAYER0 = SAFE_RANGE,
    	LAYER1,
    	LAYER2,
    	LAYER3,
	LAYER4,
	KC_SMO_SC	//Scroll Controll
};

 const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {


 [_LAYER0] = LAYOUT(

//left hand 4x7	
KC_ESC, 	KC_Q, 		KC_W, 				KC_E, 		KC_R,		KC_T, 		KC_NO
KC_TAB, 	KC_A,		KC_S, 				KC_D, 		KC_F, 		KC_G,		KC_NO 
KC_CAPS, 	KC_Z, 		KC_X, 				KC_C, 		KC_V, 		KC_B, 		KC_NO
						KC_LBRC, 			KC_RBRC,				
LT(2, KC_SPC), 	KC_DEL, 				
KC_LCTL, 		KC_LALT, 				
LT(3,KC_HOME), 	LT(4,KC_END), 
	
//Right Hand 4X7
KC_Y, 		KC_U, 		KC_I,				KC_O, 		KC_P, 		KC_MINS, 	KC_MPLY, 
KC_H, 		KC_BTN1, 	MT(KC_SMO_SC, KC_BTN2),		KC_L, 		KC_SCLN, 	KC_QUOT,	C_MNXT,
KC_N, 		KC_M, 		KC_COMM, 			KC_DOT, 	KC_SLSH,	KC_BSLS, 	KC_MPRV, 
						KC_EXLM, 			KC_EQL, 
LT(1, KC_ENT), 		KC_BSPC, 
LT(4, KC_BTN3), 	LT(3, KC_BTN4), 
KC_NO, 			KC_NO 

),


[_LAYER1] = LAYOUT(

//left hand 4x7	
KC_TRNS, 	KC_TRNS, 	KC_TRNS, 			KC_TRNS, 	KC_TRNS, 	KC_TRNS, 	KC_NO	
KC_TRNS, 	KC_TRNS, 	KC_TRNS, 			KC_TRNS, 	KC_TRNS, 	KC_TRNS, 	KC_NO	 
KC_TRNS, 	KC_TRNS, 	KC_TRNS, 			KC_TRNS, 	KC_TRNS, 	KC_TRNS, 	KC_NO	
						KC_TRNS,			KC_TRNS, 						
KC_LSFT, 	KC_TRNS, 						
KC_TRNS, 	KC_TRNS, 						
KC_TRNS, 	KC_TRNS,

//Right Hand 4X7					
KC_TRNS, 	KC_TRNS, 	KC_TRNS, 			KC_TRNS, 	KC_TRNS, 	KC_TRNS, 	KC_TRNS,
KC_TRNS, 	KC_J, 		KC_K, 				KC_TRNS, 	KC_TRNS, 	KC_TRNS,	KC_TRNS,
KC_TRNS, 	KC_TRNS, 	KC_TRNS, 			KC_TRNS, 	KC_TRNS, 	KC_TRNS, 	KC_TRNS,
						KC_TRNS, 			KC_TRNS, 
KC_TRNS, 	KC_TRNS, 
KC_TRNS, 	KC_TRNS, 
KC_NO, 		KC_NO

),


[_LAYER2] = LAYOUT(

//Left Hand 4x7	
KC_ESC, 		LSHIFT(KC_Q), 		LSHIFT(KC_W), 				LSHIFT(KC_E), 		LSHIFT(KC_R),		LSHIFT(KC_T), 		LSHIFT(KC_NO)
LSHIFT(KC_TAB) 	LSHIFT(KC_A),		LSHIFT(KC_S), 				LSHIFT(KC_D), 		LSHIFT(KC_F), 		LSHIFT(KC_G),		LSHIFT(KC_NO)
KC_LGUI, 		LSHIFT(KC_Z), 		LSHIFT(KC_X), 				LSHIFT(KC_C), 		LSHIFT(KC_V), 		LSHIFT(KC_B), 		LSHIFT(KC_NO)
						LSHIFT(KC_LBRC), 			LSHIFT(KC_RBRC),						 
KC_TRNS, 	KC_TRNS, 						 
KC_TRNS, 	KC_TRNS, 						
KC_TRNS, 	KC_TRNS, 						

//Right Hand 4X7
LSHIFT(KC_Y), 		LSHIFT(KC_U), 		LSHIFT(KC_I),				LSHIFT(KC_O), 		LSHIFT(KC_P), 		LSHIFT(KC_MINS), 	LSHIFT(KC_MPLY), 
LSHIFT(KC_H), 		LSHIFT(KC_J), 		MT(LSHIFT(KC_K),			LSHIFT(KC_L), 		LSHIFT(KC_SCLN), 	LSHIFT(KC_QUOT),	LSHIFT(KC_MNXT),
LSHIFT(KC_N), 		LSHIFT(KC_M), 		LSHIFT(KC_COMM), 			LSHIFT(KC_DOT), 	LSHIFT(KC_SLSH),	LSHIFT(KC_BSLS), 	LSHIFT(KC_MPRV), 
						LSHIFT(KC_EXLM), 			LSHIFT(KC_EQL), 

KC_TRNS, 	KC_TRNS,
KC_NO, 		KC_NO

),



[_LAYER3] = LAYOUT(

//Left Hand 4x7	
KC_AT, 		KC_HASH, 	KC_DLR, 			KC_PERC, 	KC_AMPR, 	KC_LBRC,	KC_NO	
KC_F1, 		KC_F2, 		KC_F3, 				KC_F4, 		KC_F5, 		KC_LPRN, 	KC_NO		
KC_F6, 		KC_F7, 		KC_F8, 				KC_F9, 		KC_F10, 	KC_F11, 	KC_NO		 
						KC_F12, 			KC_PSCR, 						 
KC_TRNS, 	KC_TRNS, 						 
KC_TRNS, 	KC_TRNS, 						
KC_TRNS, 	KC_TRNS, 						

//Right Hand 4X7
KC_RBRC, 	KC_P7, 		KC_P8, 				KC_P9, 		KC_ASTR, 	KC_PSLS, 	KC_TRNS,
KC_RPRN, 	KC_P4, 		KC_P5, 				KC_P6, 		KC_PMNS, 	KC_EQL, 	KC_TRNS,
KC_UNDS, 	KC_P1, 		KC_P2, 				KC_P3, 		KC_PPLS, 	KC_CIRC,	KC_TRNS,
						KC_P0, 				KC_PDOT,
KC_TRNS, 	KC_TRNS,
KC_TRNS, 	KC_TRNS,
KC_NO, 		KC_NO

),


[_LAYER4] = LAYOUT(

//Left Hand 4x7	
LALT(KC_F4),	KC_WSCH, 	KC_UP, 				KC_WREF, 	KC_WSTP, 	KC_WHOM, 	KC_NO	
LALT(KC_S), 	KC_LEFT, 	KC_DOWN, 			KC_RGHT, 	KC_FIND, 	LALT(KC_G), 	KC_NO	 
RCS(KC_Z), 	KC_UNDO, 	KC_CUT, 			KC_COPY, 	KC_PSTE, 	RCS(KC_T), 		KC_NO	
						LCTL(KC_T), 			LCTL(KC_W), 						
KC_LSHIFT,	KC_NO, 							
LALT(KC_TAB), 	LSA(KC_TAB), 						
LCTL(KC_TAB), 	RCS(KC_TAB), 						


//Right Hand 4X7
LCTL_T(KC_Y), 	KC_NO, 		KC_NO, 				KC_EJCT, 	KC_VOLU, 	KC_MPLY, 	KC_TRNS,
KC_BTN5, 	KC_NO, 		KC_NO, 				KC_NO, 		KC_VOLD, 	KC_MNXT,	KC_TRNS,
KC_MYCM, 	KC_NO, 		KC_NO, 				KC_MUTE, 	KC_MSTP, 	KC_MPRV, 	KC_TRNS,
						KC_NO, 				KC_NO, 
KC_RGUI, 	KC_SLEP,
KC_NO, 		KC_BOOT,  
KC_NO, 		KC_NO

) 

};



/***************************
 * Trackball handling
 **************************/

void pointing_device_init(void){
	if(!is_keyboard_master())
		return;
	pmw_init();
}

int max(int num1, int num2) { return (num1 > num2 ) ? num1 : num2; }
int min(int num1, int num2) { return (num1 > num2 ) ? num2 : num1; }

int8_t sign(int x) { return (x > 0) - (x < 0); }
int8_t CLAMP_HID(int value) { return value < -127 ? -127 : value > 127 ? 127 : value; }

void tap_code_fast(uint8_t code) {
	register_code(code);
	// Dont do this:
	// if (code == KC_CAPS) {
	//	 wait_ms(TAP_HOLD_CAPS_DELAY);
	// } else {
	//	 wait_ms(TAP_CODE_DELAY);
	// }
	unregister_code(code);
}

void tap_tb(uint8_t keycode0, uint8_t keycode1, uint8_t keycode2, uint8_t keycode3) {
	if(abs(cum_x) + abs(cum_y) >= cur_factor){
		if(abs(cum_x) > abs(cum_y)) {
			if(cum_x > 0) {
				for (int8_t i = 0; i <= (abs(cum_x) + abs(cum_y)) / cur_factor; i++) {
					tap_code_fast(keycode0);
					cum_x = max(cum_x - cur_factor, 0);
				}
				cum_y = 0;
			} else {
				for (int8_t i = 0; i <= (abs(cum_x) + abs(cum_y)) / cur_factor; i++) {
					tap_code_fast(keycode1);
					cum_x = min(cum_x + cur_factor, 0);
				}
				cum_y = 0;
			}
		} else {
			if(cum_y > 0) {
				for (int8_t i = 0; i <= (abs(cum_x) + abs(cum_y)) / cur_factor; i++) {
					tap_code_fast(keycode2);
					cum_y = max(cum_y - cur_factor, 0);
					}
				cum_x = 0;
			} else {
				for (int8_t i = 0; i <= (abs(cum_x) + abs(cum_y)) / cur_factor; i++) {
					tap_code_fast(keycode3);
					cum_y = min(cum_y + cur_factor, 0);
				}
				cum_x = 0;
			}
		}
	}
}

void handle_pointing_device_modes(void){
	report_mouse_t mouse_report = pointing_device_get_report();

	if (track_mode == cursor_mode) {
		if (integration_mode)
			cur_factor = cursor_multiplier_inte;
		else
			cur_factor = cursor_multiplier;
		mouse_report.x = CLAMP_HID( sensor_x * cur_factor / 100);
		mouse_report.y = CLAMP_HID(-sensor_y * cur_factor / 100);
	} else {
		// accumulate movement until threshold reached
		cum_x += sensor_x;
		cum_y += sensor_y;
		if (track_mode == carret_mode) {
			if (integration_mode)
				cur_factor = carret_threshold_inte;
			else
				cur_factor = carret_threshold;
			tap_tb(KC_RIGHT, KC_LEFT, KC_UP, KC_DOWN);

		} else if(track_mode == scroll_mode) {
				if (integration_mode)
					cur_factor = scroll_threshold_inte;
				else
					cur_factor = scroll_threshold;
				if(abs(cum_x) + abs(cum_y) >= cur_factor) {
					if(abs(cum_x) > abs(cum_y)) {
						mouse_report.h = sign(cum_x) * (abs(cum_x) + abs(cum_y)) / cur_factor;
					} else {
						mouse_report.v = sign(cum_y) * (abs(cum_x) + abs(cum_y)) / cur_factor;
					}
					cum_x = 0;
					cum_y = 0;
				}
		} else { // sound vol/brightness (3)
			cur_factor = carret_threshold;
			tap_tb(KC_BRIGHTNESS_UP, KC_BRIGHTNESS_DOWN, KC_AUDIO_VOL_UP, KC_AUDIO_VOL_DOWN);
		}
	}
	pointing_device_set_report(mouse_report);
	pointing_device_send();
}

void get_sensor_data(void) {
	if(!is_keyboard_master())
		return;
	report_pmw_t pmw_report = pmw_get_report();

	if (integration_mode) {
		sensor_x += pmw_report.x;
		sensor_y += pmw_report.y;
	} else {
		sensor_x = pmw_report.x;
		sensor_y = pmw_report.y;
	}
}

void pointing_device_task(void) {
#ifndef POLLING
	if ( is_keyboard_master() && integration_mode )
		handle_pointing_device_modes();
#else
	get_sensor_data();
	handle_pointing_device_modes();
#endif
	if(sticky_key && timer_elapsed(sticky_timer) > STICKY_TERM) {
		unstick_keys();
		activate_alt = false;
		activate_sft = false;
		activate_ctl = false;
		activate_gui = false;
		activate_rai = false;
		activate_low = false;
		sticky_key = false;
	}
}

#ifndef POLLING
	ISR(INT2_vect) {
		get_sensor_data();
		handle_pointing_device_modes();
	}
#endif

/***************************
 * process_record_kb
 **************************/

bool process_record_kb(uint16_t keycode, keyrecord_t *record) {
	if(!process_record_user(keycode, record)) {
		return false;
	}

	switch (keycode) {

		// no repetitive ::: with holding
		case KC_SCLN_INV:
			if (record->event.pressed) {
				if (is_sft_active) {
					unregister_mods(MOD_BIT(KC_LSFT));
					tap_code(KC_SCLN);
					register_mods(MOD_BIT(KC_LSFT));
				} else {
					register_mods(MOD_BIT(KC_LSFT));
					tap_code(KC_SCLN);
					unregister_mods(MOD_BIT(KC_LSFT));
				}
			}
			return false;

		// ; and : gets confused in some corner cases
		//case KC_SCLN_INV:
		//	if (record->event.pressed) {
		//	if (is_sft_active) {
		//		unregister_mods(MOD_BIT(KC_LSFT));
		//		register_code(KC_SCLN);
		//	} else {
		//		register_mods(MOD_BIT(KC_LSFT));
		//		register_code(KC_SCLN);
		//	}
		//	} else {
		//	if (is_sft_active) {
		//		unregister_code(KC_SCLN);
		//		register_mods(MOD_BIT(KC_LSFT));
		//	} else {
		//		unregister_code(KC_SCLN);
		//		unregister_mods(MOD_BIT(KC_LSFT));
		//	}
		//	}
		//	return false;

		case KC_TILD_MY:
			if (record->event.pressed) {
				tap_code16(KC_TILD);
				tap_code(KC_SPC);
			}
			return false;

		case KC_QUOT_MY:
			if (record->event.pressed) {
				tap_code(KC_QUOT);
				tap_code(KC_SPC);
			}
			return false;

		// handle mouse
		case KC_BTN1:
			on_mouse_button(MOUSE_BTN1, record->event.pressed);
			return false;

		case KC_BTN2:
			on_mouse_button(MOUSE_BTN2, record->event.pressed);
			return false;

		case KC_BTN3:
			on_mouse_button(MOUSE_BTN3, record->event.pressed);
			return false;

		case KC_BTN4:
			on_mouse_button(MOUSE_BTN4, record->event.pressed);
			return false;

		case KC_BTN5:
			on_mouse_button(MOUSE_BTN5, record->event.pressed);
			return false;

		case KC_CPI_DOWN:
			if (cursor_multiplier > CPI_STEP)
				cursor_multiplier = cursor_multiplier - CPI_STEP;
			return false;

		case KC_CPI_STD:
			cursor_multiplier = 250;
			return false;

		case KC_CPI_UP:
			cursor_multiplier = cursor_multiplier + CPI_STEP;
			return false;

		case KC_SMO_SC:
			if (record->event.pressed) {
				if (smooth_scroll) {
				scroll_threshold = scroll_threshold * regular_smoothscroll_factor;
				scroll_threshold_inte = scroll_threshold_inte * regular_smoothscroll_factor;
				smooth_scroll = false;
				} else {
				scroll_threshold = scroll_threshold / regular_smoothscroll_factor;
				scroll_threshold_inte = scroll_threshold_inte / regular_smoothscroll_factor;
				smooth_scroll = true;
				}
			}

		default:
			return true;
	}
}

