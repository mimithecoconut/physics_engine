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
#include "forces.h"
#include <float.h>

const double W_HEIGHT = 500.0;
const double W_WIDTH = 1000.0;
const int CIRCLE_R = 10;
const int CIRCLE_D = 20;
const double GAMMA = 5.0;
const double K_CONSTANT = 50;
const int INIT_LIST_SIZE = 5;
const int CIRCLE_POINTS = 48;
const int CIRCLE_FRAC = 24;
const double SIN_AMP = 100.0;
const double CIRCLE_MASS = 10.0;
const double ANCHOR_R = 0.01;

/**
 * Initializes a body with a circle shape
 *
 * @param r radius of circle
 * @param start vector_t representing initial center position of circle
 * @param mass double representing mass of circle
 * @return pointer to body of circle
 */
body_t *init_circle(double r, vector_t start, double mass) {
  list_t *points = list_init(INIT_LIST_SIZE, (free_func_t) free);
  for (int i = 0; i < CIRCLE_POINTS; i++) {
    vector_t *point = malloc(sizeof(vector_t));
    point->x = start.x + r * cos(i * M_PI / CIRCLE_FRAC);
    point->y = start.y + r * sin(i * M_PI / CIRCLE_FRAC);
    list_add(points, point);
  }
  return body_init(points, mass, (rgb_color_t) {1, 0, 0});
}

/**
 * Adds a line of circles across the scene, and invisible anchors for each
 * circle
 *
 * @param scene the scene to add the circles to
 */
void put_circles(scene_t *scene){
  for (int i = 0; i < (int) W_WIDTH; i+= CIRCLE_D){
    body_t *body = init_circle(CIRCLE_R, (vector_t){i + CIRCLE_R, W_HEIGHT/2 + \
      SIN_AMP * sin((i + CIRCLE_R) / W_HEIGHT * M_PI)}, CIRCLE_MASS);
    scene_add_body(scene, body);
    body_t *anchor = init_circle(ANCHOR_R, (vector_t){i + CIRCLE_R, \
      W_HEIGHT/2}, DBL_MAX);
    scene_add_body(scene, anchor);
    body_set_color(anchor, (rgb_color_t) {1, 1, 1});
  }
}

/**
 * Applies a spring force between each circle and its anchor
 *
 * @param scene the scene to add forces to
 */
void apply_spring(scene_t *scene){
  for (size_t i = 0; i < scene_bodies(scene) - 1; i += 2 ) {
      create_spring(scene, K_CONSTANT, scene_get_body(scene, i), scene_get_body(scene,(i + 1)));
  }
}

/**
 * Applies a drag force to every body in the scene
 *
 * @param scene the scene to apply the forces to
 */
void apply_drag(scene_t *scene){
  for (size_t i = 0; i < scene_bodies(scene); i++) {
    create_drag(scene, GAMMA, scene_get_body(scene, i));
  }
}

/**
 * Initializes the scene and runs the demo, updating the frames
 *
 * @param argc number of arguments
 * @param argv array of arguments
 * @return status of program
 */
int main(int argc, char *argv[]){
  if (argc != 1) {
      printf("USAGE: %s\n", argv[0]);
      return 1;
  }
  vector_t min = VEC_ZERO;
  vector_t max = {W_WIDTH, W_HEIGHT};
  sdl_init(min, max);
  scene_t *scene = scene_init();
  put_circles(scene);
  apply_spring(scene);
  apply_drag(scene);
  while(!sdl_is_done(scene_get_body(scene, 0))){
    double time_elapsed = time_since_last_tick();
    scene_tick(scene, time_elapsed);
    sdl_render_scene(scene);
  }
  return 0;
}
