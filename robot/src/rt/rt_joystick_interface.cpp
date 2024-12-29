#include <errno.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <string>

#include "rt/rt_joystick_interface.h"
#include "rt/rt_rc_interface.h"
#include "Utilities/EdgeTrigger.h"


pthread_mutex_t joy_data_m;

struct js_event et;

struct axis_state {
    float x, y;
};

struct axis_state axes[6] = {{0,0}};

#define J_BUS_PORT "/dev/input/js0"
int selected_mode = 0;

// Controller Settings
rc_control_settings rc_control;

EdgeTrigger<int> js_mode_edge_trigger(0);

// Controller Settings
void get_js_control_settings(void *settings) {
  v_memcpy(settings, &rc_control, sizeof(rc_control_settings));
}

/*!
 * Receive data packets
 */
int receive_data(int fd) {
	ssize_t bytes;
    struct js_event event_;

	bytes = read(fd, &event_, sizeof(js_event));

	if (bytes == sizeof(js_event)) {
        pthread_mutex_lock(&joy_data_m);
        et = event_;
        pthread_mutex_unlock(&joy_data_m);
		return 1;
    }

	printf("Error, could not read full event.");
    return 0;
}

/*!
 * Initialize joystick port
 */
int init_joystick() {
	if (pthread_mutex_init(&joy_data_m, NULL) != 0) {
    	printf("Failed to initialize sbus data mutex.\n");
  	}

	int fd = open(J_BUS_PORT, O_RDONLY | O_NOCTTY | O_SYNC);
	if (fd < 0) {
    	printf("Error opening %s: %s\n", J_BUS_PORT, strerror(errno));
    }
	return fd;
}

/*
for more information:
https://www.kernel.org/doc/Documentation/input/joystick-api.txt
*/

/**
 * Current state of an axis.
 */


static float scale_joystick(short in) {

    float joystick_scaled = (in - (-32767)) * 2.f / (32767.f + 32767.f) - 1.f;
    if (joystick_scaled > -.1 && joystick_scaled < .1)
    {
        return 0.0;
    }else{
        return joystick_scaled;
    }
    
}

/**
 * Keeps track of the current axis state.
 *
 * NOTE: This function assumes that axes are numbered starting from 0, and that
 * the X axis is an odd number, and the Y axis is an even number. However, this
 * is usually a safe assumption.
 *
 * Returns the axis that the event indicated.
 */
void get_axis_state() {

    switch (et.number )
    {
    case 0:
        axes[0].x = scale_joystick(et.value);
        break;
    case 1:
        axes[0].y = scale_joystick(et.value);
        break;
    case 3:
        axes[1].x = scale_joystick(et.value);
        break;
    case 4:
        axes[1].y = scale_joystick(et.value);
        break;

    default:
        break;
    }
}


void update_joystick() {
    


	pthread_mutex_lock(&joy_data_m);
    if (et.type == JS_EVENT_BUTTON) {
        switch (et.number) {
            case 0:
                selected_mode = RC_mode::OFF;
                break;

            case 1:
                selected_mode = RC_mode::RECOVERY_STAND;
                break;

            case 2:
                selected_mode = RC_mode::LOCOMOTION;
                break;

            case 3:
                selected_mode = RC_mode::QP_STAND;
                break;

            case 4:
                selected_mode = RC_mode::STAND_UP;
                break;

            case 5:
                selected_mode = RC_mode::SQUAT_DOWN;
                break;

            default:
                printf("[Joystick Interface] unknown button\n");
                break;
        }
    } else if (et.type == JS_EVENT_AXIS) {
        get_axis_state();

        float v_scale = 1.0; //axes[2].x*1.5f + 2.0f; // from 0.5 to 3.5
        float w_scale = 2.*v_scale; // from 1.0 to 7.0

        // gait selection
        int mode_id = 0;

        constexpr int gait_table[9] = {0, //stand
            0, // trot
            1, // bounding
            2, // pronking
            3, // gallop
            5, // trot run
            6, // walk};
            7, // walk2?
            8, // pace
        }; 

        if (selected_mode == RC_mode::LOCOMOTION) {
            rc_control.variable[0] = gait_table[mode_id];
            //rc_control.v_des[0] = v_scale * axes[0].x * 0.5;
            //rc_control.v_des[1] = v_scale * axes[0].y * -1.;
            rc_control.v_des[0] = -v_scale * axes[0].y;
            rc_control.v_des[1] = -v_scale * axes[0].x;
            rc_control.v_des[2] = 0;

            rc_control.height_variation = axes[1].y;
            //rc_control.p_des[2] = 0.27 + 0.08 * axes[2].y; // todo or not todo?

            rc_control.omega_des[0] = 0;
            rc_control.omega_des[1] = 0;
            rc_control.omega_des[2] = -w_scale * axes[1].x;
            //rc_control.omega_des[2] = -v_scale * data.right_stick[0];
        } else if (selected_mode == RC_mode::QP_STAND) {
            //rc_control.rpy_des[0] = axes[0].y * 1.4;
            //rc_control.rpy_des[1] = axes[1].x * 0.46;
            printf("roll angle %f \n", rc_control.rpy_des[0]);
            printf("pitch angle %f \n", rc_control.rpy_des[1]);
            printf("yaw angle %f \n", rc_control.rpy_des[2]);
            printf("height %f \n", rc_control.height_variation);
            rc_control.rpy_des[0] = axes[0].x;
            rc_control.rpy_des[1] = axes[1].y;
            rc_control.rpy_des[2] = axes[1].x;

            rc_control.height_variation = axes[0].y;
            
            rc_control.omega_des[0] = 0;
            rc_control.omega_des[1] = 0;
            rc_control.omega_des[2] = 0;
            //rc_control.p_des[1] = -0.667 * rc_control.rpy_des[0];
            //rc_control.p_des[2] = data.axes[0].x * .12;
        }
    }
	pthread_mutex_unlock(&joy_data_m);

    bool trigger = js_mode_edge_trigger.trigger(selected_mode);
    //printf("selected-mode %d\n", selected_mode);
    if(trigger || selected_mode == RC_mode::OFF || selected_mode == RC_mode::RECOVERY_STAND) {
        if(trigger) {
            printf("MODE TRIGGER!\n");
        }
        rc_control.mode = selected_mode;
    }
}