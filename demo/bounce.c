#include "sdl_wrapper.h"
#include "polygon.h"
#include "vector.h"
#include "list.h"
#include "star.h"
#include "color.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>

const double WINDOW_HEIGHT = 500.0;
const double WINDOW_WIDTH = 1000.0;
const vector_t START_POS = {100.0, 100.0};
const double RADIUS = 50.0;
const double ROT_RATE = 5.0;
const int POINTS = 5;
const double INIT_VELOCITY = 500.0;

/** Returns true if the object will bounce in the specified direction */
bool will_bounce(star_t *s, double ctr_crd, double up_bound) {
    return (ctr_crd + get_radius(s) > up_bound || ctr_crd - get_radius(s) < 0);
}

/**
 * Moves star back into window, dir == 0 denotes x displacement, dir == 1
 * denotes y displacement
 */
void account_for_bounce(star_t *s, double ctr_crd, double up_bound, int dir) {
    double x_disp = 0.0;
    double y_disp = 0.0;
    if (ctr_crd + RADIUS > up_bound) {
        if (dir == 0) {
            x_disp = -2 * (ctr_crd + RADIUS - up_bound);
        }
        else {
            y_disp = -2 * (ctr_crd + RADIUS - up_bound);
        }
    }
    else {
        if (dir == 0) {
            x_disp = -2 * (-RADIUS + ctr_crd);
        }
        else {
            y_disp = -2 * (-RADIUS + ctr_crd);
        }
    }
    vector_t displacement = {x_disp, y_disp};
    translate_star(s, displacement);
}

/**
 * Checks for bounce if the star goes out of bounds and updates velocity
 * accordingly
 */
void check_for_bounce(star_t *s) {
    vector_t center = get_center(s);
    if (will_bounce(s, center.y, WINDOW_HEIGHT)) {
        add_velocity(s, 0.0, get_velocity(s)->y * -2);
        account_for_bounce(s, center.y, WINDOW_HEIGHT, 1);
    }
    else if (will_bounce(s, center.x, WINDOW_WIDTH)) {
        add_velocity(s, get_velocity(s)->x * -2, 0.0);
        account_for_bounce(s, center.x, WINDOW_WIDTH, 0);
    }
}

/** Moves the star to its new position */
void compute_new_positions(star_t *s, double dt) {
    vector_t translation = {dt * get_velocity(s)->x, dt * get_velocity(s)->y};
    rotate_star(s, ROT_RATE * dt);
    translate_star(s, translation);
    check_for_bounce(s);
}

/**
 * Initializes the scene and runs the demo, updating the frames
 *
 * @param argc number of arguments
 * @param argv array of arguments
 * @return status of program
 */
int main(int argc, char *argv[]) {
    if (argc != 1) {
        printf("USAGE: %s\n", argv[0]);
        return 1;
    }
    vector_t min = VEC_ZERO;
    vector_t max = {WINDOW_WIDTH, WINDOW_HEIGHT};
    sdl_init(min, max);
    star_t *s = init_star(START_POS, POINTS, RADIUS, INIT_VELOCITY, INIT_VELOCITY);
    while (!sdl_is_done(NULL)) {
        double time_elapsed = time_since_last_tick();
        compute_new_positions(s, time_elapsed);
        sdl_clear();
        sdl_draw_polygon(get_coords(s), s->color);
        sdl_show();
    }
    star_free(s);
    return 0;
}
