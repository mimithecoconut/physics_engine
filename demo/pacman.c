#include "sdl_wrapper.h"
#include "vector.h"
#include "body.h"
#include "scene.h"
#include "list.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>

const double HEIGHT = 1000.0;
const double WIDTH = 1000.0;
const vector_t START_POS = {200.0, 200.0};
const double ROT_RATE = 5.0;
const double RADIUS = 10.0;
const double PACRADIUS = 100.0;
const vector_t ACC = {50.0, 50.0};
const double DEFAULT = 100.0;
const int INIT_LIST = 5;
const int CIRC_PTS = 48;
const int CIRC_FRAC = 24;
const double MASS = 10.0;
const int PAC_PTS = 45;
const int MOUTH_PT = 41;
const int NUM_PELLETS = 10;
const double PELLET_DELAY = 0.8;
const int MIN_PAC = 4;

/**
 * Returns a pointer to a body representing a circle
 *
 * @param r radius of circle expressed as a double
 * @param start initial starting position of circle, expressed as a vector
 * @return body_t pointer to circle
 */
body_t *init_circle(double r, vector_t start) {
  list_t *points = list_init(INIT_LIST, (free_func_t) free);
  for (int i = 0; i < CIRC_PTS; i++) {
    vector_t *point = malloc(sizeof(vector_t));
    point->x = start.x + r * cos(i * M_PI / CIRC_FRAC);
    point->y = start.y + r * sin(i * M_PI / CIRC_FRAC);
    list_add(points, point);
  }
  return body_init(points, MASS, (rgb_color_t) {1, 1, 0});
}

/**
 * Returns a pointer to a body representing pacman
 *
 * @param r radius of pacman expressed as a double
 * @param start initial starting position of pacman, expressed as a vector
 * @return body_t pointer to pacman
 */
body_t *init_pacman(double r, vector_t start) {
  list_t *points = list_init(INIT_LIST, (free_func_t) free);
  vector_t *center = malloc(sizeof(vector_t));
  // Initialize mouth of pacman
  center->x = start.x;
  center->y = start.y;
  list_add(points, center);
  // Initialize body of pacman
  for (int i = MIN_PAC; i < PAC_PTS; i++) {
    vector_t *point = malloc(sizeof(vector_t));
    point->x = start.x + r * cos(i * M_PI / CIRC_FRAC);
    point->y = start.y + r * sin(i * M_PI / CIRC_FRAC);
    list_add(points, point);
  }
  return body_init(points, MASS, (rgb_color_t) {1, 1, 0});
}

/**
 * Adds a pellet to the scene in a random location
 *
 * @param s scene to add the pellets to
 */
void init_pellet(scene_t *s) {
  double rand_x = (double) rand() / RAND_MAX * WIDTH;
  double rand_y = (double) rand() / RAND_MAX * HEIGHT;
  body_t *pellet = init_circle(RADIUS, (vector_t) {rand_x, rand_y});
  scene_add_body(s, pellet);
}

/**
 * Returns the larger of the two provided numbers, or either if they are equal
 *
 * @param a first number to compare
 * @param b second number to compare
 * @return larger of the two numbers
 */
double max(double a, double b) {
  if (a > b) {
    return a;
  }
  else {
    return b;
  }
}

/**
 * Returns the smaller of the two provided numbers, or either if they are equal
 *
 * @param a first number to compare
 * @param b second number to compare
 * @return smaller of the two numbers
 */
double min(double a, double b) {
  if (a < b) {
    return a;
  }
  else {
    return b;
  }
}

/**
 * Checks pellets in the scene and removes them if they are "eaten"
 *
 * @param s scene to remove the pellets from
 * @param pacman body representing the pacman
 */
void eat_pellet(scene_t *s, body_t *pacman) {
  vector_t *start_mouth = (vector_t*) list_get(body_get_shape(pacman), 1);
  vector_t *end_mouth = (vector_t*) list_get(body_get_shape(pacman), MOUTH_PT);
  vector_t *mid_mouth = (vector_t*) list_get(body_get_shape(pacman), 0);
  double max_x = max(start_mouth->x, max(end_mouth->x, mid_mouth->x));
  double min_x = min(start_mouth->x, min(end_mouth->x, mid_mouth->x));
  double max_y = max(start_mouth->y, max(end_mouth->y, mid_mouth->y));
  double min_y = min(start_mouth->y, min(end_mouth->y, mid_mouth->y));
  for (size_t i = 1; i < scene_bodies(s); i ++){
    vector_t centroid = body_get_centroid(scene_get_body(s, i));
    if (centroid.x <= max_x && centroid.x >= min_x && centroid.y <= max_y && centroid.y >= min_y) {
      scene_remove_body(s, i);
    }
  }
}

/**
 * Returns the appropriate velocity of pacman, depending on the current
 * orientation
 *
 * @param angle new orientation of pacman, in radians
 * @param pacman body representing pacman
 * @param held_time representing how long key is held
 * @return vector_t of new velocity of pacman
 */
vector_t accelerate_pacman(double angle, body_t *pacman, double held_time) {
  vector_t curr_v = body_get_velocity(pacman);
  if (body_get_orientation(pacman) == angle) {
    vector_t accel = (vector_t) {ACC.x * cos(angle), ACC.y * sin(angle)};
    return vec_add(curr_v, vec_multiply(held_time, accel));
  }
  else {
    return (vector_t) {DEFAULT * cos(angle), DEFAULT * sin(angle)};
  }
}

/**
 * Adjusts the velocity of the pacman based on keyboard input
 *
 * @param key representing which key is KEY_PRESSED
 * @param type representing if key is KEY_PRESSED
 * @param held_time representing how long key is held
 * @param body representing pacman object
 */
 void on_key(char key, key_event_type_t type, double held_time, void* pacman) {
    double angle = body_get_orientation(pacman);
    vector_t acc;
    double default_x = DEFAULT * cos(body_get_orientation(pacman));
    double default_y = DEFAULT * sin(body_get_orientation(pacman));
    vector_t default_v = {default_x, default_y};
    switch (type) {
      case KEY_RELEASED:
          body_set_velocity(pacman, default_v);
          break;
      case KEY_PRESSED:
        switch (key) {
          case DOWN_ARROW:
            angle = 3 * M_PI / 2;
            break;
          case UP_ARROW:
            angle = M_PI / 2;
            break;
          case LEFT_ARROW:
            angle = M_PI;
            break;
          case RIGHT_ARROW:
            angle = 0;
            break;
        }
        acc = accelerate_pacman(angle, pacman, held_time);
        body_set_velocity(pacman, acc);
        body_set_rotation(pacman, angle);
    }
}

/**
 * Updates position of pacman to wrap around once it goes off screen
 *
 * @param pacman body_t representing pacman
 */
void wrap_around(body_t *pacman) {
  vector_t curr_pos = body_get_centroid(pacman);
  if (curr_pos.x + PACRADIUS > WIDTH) {
    body_set_centroid(pacman, (vector_t) {PACRADIUS, curr_pos.y});
  }
  else if (curr_pos.x - PACRADIUS < 0) {
    body_set_centroid(pacman, (vector_t) {WIDTH - PACRADIUS, curr_pos.y});
  }
  else if (curr_pos.y + PACRADIUS > HEIGHT) {
    body_set_centroid(pacman, (vector_t) {curr_pos.x, PACRADIUS});
  }
  else if (curr_pos.y - PACRADIUS < 0) {
    body_set_centroid(pacman, (vector_t) {curr_pos.x, HEIGHT - PACRADIUS});
  }
}

int main(int argc, char *argv[]) {
  if (argc != 1) {
      printf("USAGE: %s\n", argv[0]);
      return 1;
  }
  vector_t min = {0.0, 0.0};
  vector_t max = {WIDTH, HEIGHT};
  sdl_init(min, max);
  body_t *pacman = init_pacman(PACRADIUS, START_POS);
  scene_t *scene = scene_init();
  scene_add_body(scene, pacman);
  for (int j = 0; j < NUM_PELLETS; j++) {
    init_pellet(scene);
  }
  double total_time_elapsed = 0.0;
  sdl_on_key((key_handler_t) on_key);
   while (!sdl_is_done(pacman)) {
    double time_elapsed = time_since_last_tick();
    total_time_elapsed += time_elapsed;

    // Initializes a new pellet after an interval
    if (total_time_elapsed > PELLET_DELAY) {
      total_time_elapsed = 0.0;
      init_pellet(scene);
    }

    eat_pellet(scene, pacman);
    wrap_around(pacman);
    scene_tick(scene, time_elapsed);
    sdl_render_scene(scene);
  }
  scene_free(scene);
  return 0;
}
