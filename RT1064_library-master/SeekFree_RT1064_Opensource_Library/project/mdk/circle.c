#include "circle.h"
#include "motor.h"
#include "main.h"
#include "flash_param.h"

enum circle_type_e circle_type = CIRCLE_NONE;

const char *circle_type_name[CIRCLE_NUM] = {
        "CIRCLE_NONE",
        "CIRCLE_LEFT_BEGIN", "CIRCLE_RIGHT_BEGIN",
        "CIRCLE_LEFT_RUNNING", "CIRCLE_RIGHT_RUNNING",
        "CIRCLE_LEFT_IN", "CIRCLE_RIGHT_IN",
        "CIRCLE_LEFT_OUT", "CIRCLE_RIGHT_OUT",
        "CIRCLE_LEFT_END", "CIRCLE_RIGHT_END",
};

int64_t circle_encoder;

int none_left_line = 0, none_right_line = 0;
int have_left_line = 0, have_right_line = 0;

void check_circle() {
    if (circle_type == CIRCLE_NONE && Lpt0_found && !Lpt1_found && is_straight1) {
        circle_type = CIRCLE_LEFT_BEGIN;
    }
    if (circle_type == CIRCLE_NONE && !Lpt0_found && Lpt1_found && is_straight0) {
        circle_type = CIRCLE_RIGHT_BEGIN;
    }
}




void run_circle() {
	  int64_t current_encoder = get_total_encoder();
    if (circle_type == CIRCLE_LEFT_BEGIN) {
        track_type = TRACK_RIGHT;


        if (rpts0s_num < 0.2 / sample_dist) { none_left_line++; }
        if (rpts0s_num > 1.0 / sample_dist && none_left_line > 2) {
            have_left_line++;
            if (have_left_line > 1) {
                circle_type = CIRCLE_LEFT_IN;
                none_left_line = 0;
                have_left_line = 0;
                circle_encoder = current_encoder;
            }
        }
    }

    else if (circle_type == CIRCLE_LEFT_IN) {
        track_type = TRACK_LEFT;


        if (rpts0s_num < 0.1 / sample_dist ||
            current_encoder - circle_encoder >= ENCODER_PER_METER * (3.14 * 1 / 2)) { circle_type = CIRCLE_LEFT_RUNNING; }
    }

    else if (circle_type == CIRCLE_LEFT_RUNNING) {
        track_type = TRACK_RIGHT;

        if (Lpt1_found) rpts1s_num = rptsc1_num = Lpt1_rpts1s_id;

        if (Lpt1_found && Lpt1_rpts1s_id < 0.4 / sample_dist) {
            circle_type = CIRCLE_LEFT_OUT;
        }
    }

    else if (circle_type == CIRCLE_LEFT_OUT) {
        track_type = TRACK_LEFT;


        if (is_straight1) {
            circle_type = CIRCLE_LEFT_END;
        }
    }

    else if (circle_type == CIRCLE_LEFT_END) {
        track_type = TRACK_RIGHT;


        if (rpts0s_num < 0.2 / sample_dist) { none_left_line++; }
        if (rpts0s_num > 1.0 / sample_dist && none_left_line > 3) {
            circle_type = CIRCLE_NONE;
            none_left_line = 0;
        }
    }

    else if (circle_type == CIRCLE_RIGHT_BEGIN) {
        track_type = TRACK_LEFT;


        if (rpts1s_num < 0.2 / sample_dist) { none_right_line++; }
        if (rpts1s_num > 1.0 / sample_dist && none_right_line > 2) {
            have_right_line++;
            if (have_right_line > 1) {
                circle_type = CIRCLE_RIGHT_IN;
                none_right_line = 0;
                have_right_line = 0;
                circle_encoder = current_encoder;
            }
        }
    }

    else if (circle_type == CIRCLE_RIGHT_IN) {
        track_type = TRACK_RIGHT;


        if (rpts1s_num < 0.1 / sample_dist ||
            current_encoder - circle_encoder >= ENCODER_PER_METER * (3.14 * 1 / 2)) { circle_type = CIRCLE_RIGHT_RUNNING; }

    }

    else if (circle_type == CIRCLE_RIGHT_RUNNING) {
        track_type = TRACK_LEFT;


        if (Lpt0_found) rpts0s_num = rptsc0_num = Lpt0_rpts0s_id;
        if (Lpt0_found && Lpt0_rpts0s_id < 0.4 / sample_dist) {
            circle_type = CIRCLE_RIGHT_OUT;
        }
    }

    else if (circle_type == CIRCLE_RIGHT_OUT) {
        track_type = TRACK_RIGHT;


        if (is_straight0) {
            circle_type = CIRCLE_RIGHT_END;
        }
    }

    else if (circle_type == CIRCLE_RIGHT_END) {
        track_type = TRACK_LEFT;

  
        if (rpts1s_num < 0.2 / sample_dist) { none_right_line++; }
        if (rpts1s_num > 1.0 / sample_dist && none_right_line > 2) {
            circle_type = CIRCLE_NONE;
            none_right_line = 0;
        }
    }
}


void draw_circle() {

}
