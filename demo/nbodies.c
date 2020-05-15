#include "sdl_wrapper.h"
#include "vector.h"
#include "body.h"
#include "scene.h"
#include "star.h"
#include "list.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "color.h"
#include "forces.h"

const double WIN_HEIGHT = 500.0;
const double WIN_WIDTH = 1000.0;
const int PTS = 4;
const double G = 1000.0;
const int NUM_STARS = 60;
const int POS_CORR = 50;
const int R_CORR = 10;
const int M_CORR = 100;
const int V_SEED = 50;
const int R_C = 1;
const int MASS_MIN = 600;

/**
 * Applies a gravitational force between all bodies in the scene
 *
 * @param scene the scene to apply the force to
 */
void apply_force_all(scene_t *scene){
  for (size_t i = 0; i < scene_bodies(scene) - 1; i++){
    for (size_t j = i + 1; j < scene_bodies(scene); j++){
      create_newtonian_gravity(scene, G, scene_get_body(scene, i), \
        scene_get_body(scene, j));
    }
  }
}

/**
 * Adds star bodies to the scene and initializes their velocities
 *
 * @param scene the scene to add the stars to
 */
void build_scene(scene_t *scene){
  for (int i = 0; i < NUM_STARS; i ++){
    double start_x = (double) (rand() % ((int)WIN_WIDTH - POS_CORR)) + POS_CORR;
    double start_y = (double) (rand() % ((int)WIN_HEIGHT- POS_CORR)) + POS_CORR;
    vector_t start_pos = (vector_t) {start_x, start_y};
    double radius = (double)(rand() %(POS_CORR - R_CORR + R_C)) + R_CORR;
    double mass = (double)(rand() % (MASS_MIN - M_CORR)) + M_CORR;
    star_t *str = init_star(start_pos, PTS, radius, 0.0, 0.0);
    star_set_color(str, (rgb_color_t){(float)rand()/ (float)RAND_MAX, \
      (float)rand()/ (float)RAND_MAX, (float)rand()/ (float)RAND_MAX});
    body_t *star_body = body_init(get_coords(str), mass, str->color);
    double v_x = (double)(rand() %(V_SEED)) - (double)(rand() %(V_SEED));
    double v_y = (double)(rand() %(V_SEED)) - (double)(rand() %(V_SEED));
    body_set_velocity(star_body, (vector_t) {v_x, -1 *v_y});
    scene_add_body(scene, star_body);
  }
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
  vector_t max = {WIN_WIDTH, WIN_HEIGHT};
  sdl_init(min, max);
  scene_t *scene = scene_init();
  build_scene(scene);
  apply_force_all(scene);
  while(!sdl_is_done(scene_get_body(scene, 0))){
    double time_elapsed = time_since_last_tick();
    scene_tick(scene, time_elapsed);
    sdl_render_scene(scene);
  }
  scene_free(scene);
  return 0;
}
