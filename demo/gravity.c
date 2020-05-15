#include "sdl_wrapper.h"
#include "polygon.h"
#include "vector.h"
#include "list.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "star.h"
#include "color.h"
#include <assert.h>

const double W_HEIGHT = 500.0;
const double W_WIDTH = 1000.0;
const vector_t START_POS = {0.0, 500.0};
const double ROT_RATE = 5.0;
const double RADIUS = 50.0;
const double X_VEL = 200.0;
const double DAMPING_CONST = -1.9;
const double G = -5.0;
const int INIT_LIST = 5;
const double STAR_DELAY = 0.5;

/** Returns true if the object will bounce in the specified direction */
bool will_bounce(star_t *s, double ctr_crd, double up_bound) {
    return (ctr_crd + get_radius(s) > up_bound || ctr_crd - get_radius(s) < 0);
}

/** Moves star back into window */
void account_for_bounce(star_t *s, double ctr_crd, double up_bound) {
  double x_disp = 0.0;
  double y_disp = 0.0;
  if (ctr_crd + get_radius(s) > up_bound) {
      y_disp = -2 * (ctr_crd + get_radius(s) - up_bound);
  }
  else {
      y_disp = -2 * (-get_radius(s) + ctr_crd);
  }
  vector_t displacement = {x_disp, y_disp};
  translate_star(s, displacement);
}

/**
 * Checks whether the star should bounce and updates velocity accordingly
 */
void check_for_bounce(star_t *s) {
    vector_t center = get_center(s);
    if (will_bounce(s, center.y, W_HEIGHT)) {
        add_velocity(s, 0.0, get_velocity(s)->y * DAMPING_CONST);
        account_for_bounce(s, center.y, W_HEIGHT);
    }
}

/** Accounts for gravity by continuously subtracting from y velocity */
void account_for_gravity(star_t *s) {
    add_velocity(s, 0.0, G);
}

/** Moves the star to its new position */
void compute_new_positions(star_t *s, double dt) {
    vector_t translation = {dt * get_velocity(s)->x, dt * get_velocity(s)->y};
    rotate_star(s, ROT_RATE * dt);
    translate_star(s, translation);
    account_for_gravity(s);
    check_for_bounce(s);
}

/**  Checks if the star bounces out of the screen */
bool is_off_screen(star_t *s) {
    return get_center(s).x - get_radius(s) > W_WIDTH;
}

/** Removes any stars in s_list that are off screen */
void remove_off_screen(list_t *s_list) {
    for (int k = (int)list_size(s_list) - 1; k >= 0; k--) {
        if (is_off_screen((star_t*) list_get(s_list, (size_t)k))) {
            star_free(list_remove(s_list, (size_t)k));
        }
    }
}

/** Updates all of the positions of the stars in the list */
void update_all_positions(list_t *s_list, double dt) {
    for (int k = (int)list_size(s_list) - 1; k >= 0; k--) {
        compute_new_positions((star_t*) list_get(s_list, (size_t) k), dt);
    }
}

int main(int argc, char *argv[]) {
    if (argc != 1) {
        printf("USAGE: %s\n", argv[0]);
        return 1;
    }
    vector_t min = {0.0, 0.0};
    vector_t max = {W_WIDTH, W_HEIGHT};
    sdl_init(min, max);
    list_t *s_list = list_init(INIT_LIST, (free_func_t) star_free);
    int pts = 2;
    double total_time_elapsed = 0.0;
    while (!sdl_is_done(NULL)) {
        double time_elapsed = time_since_last_tick();
        total_time_elapsed += time_elapsed;
        update_all_positions(s_list, time_elapsed);

        // Periodically intializes a new star
        if (total_time_elapsed > STAR_DELAY) {
            total_time_elapsed = 0.0;
            star_t *str = init_star(START_POS, pts, RADIUS, X_VEL, 0.0);
            star_set_color(str, (rgb_color_t) {(float)rand()/ (float)RAND_MAX,\
               (float)rand()/ (float)RAND_MAX, (float)rand()/ (float)RAND_MAX});
            list_add(s_list, str);
            pts++;
        }
        remove_off_screen(s_list);

        // Update screen with new positions of stars
        sdl_clear();
        for (int k = (int)list_size(s_list) - 1; k >= 0; k--) {
            star_t *curr = (star_t *) list_get(s_list, (size_t)k);
            sdl_draw_polygon(get_coords(curr), curr->color);
        }
        sdl_show();
    }
    list_free(s_list);
    return 0;
}
