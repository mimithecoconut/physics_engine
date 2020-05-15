#include "scene.h"
#include "forces.h"
#include <math.h>
#include "body.h"
#include <stdio.h>
#include <stdlib.h>
#include "collision.h"

const double MIN_DIST = 5.0;

void gravity_creator(void *aux) {
  body_t *bod1 = ((aux_t *) aux)->body1;
  body_t *bod2 = ((aux_t *) aux)->body2;
  vector_t pos1 = body_get_centroid(bod1);
  vector_t pos2 = body_get_centroid(bod2);
  double distance = sqrt(pow((pos2.x - pos1.x), 2) + pow((pos2.y - pos1.y), 2));
  if (distance > MIN_DIST) {
    vector_t unit = (vector_t) {(pos2.x - pos1.x) / distance, \
      (pos2.y - pos1.y) / distance};
    double force_mag = ((aux_t *) aux)->constant * body_get_mass(bod1) * \
      body_get_mass(bod2) / pow(distance, 2);
    body_add_force(bod1, vec_multiply(force_mag, unit));
    body_add_force(bod2, vec_negate(vec_multiply(force_mag, unit)));
  }
}

void spring_creator(void *aux) {
  vector_t pos1 = body_get_centroid(((aux_t*) aux)->body1);
  vector_t pos2 = body_get_centroid(((aux_t*) aux)->body2);
  vector_t force = vec_multiply(((aux_t*) aux)->constant, vec_subtract(pos2, pos1));
  body_add_force(((aux_t*) aux)->body1, force);
  body_add_force(((aux_t*) aux)->body2, vec_negate(force));
}

void drag_creator(void *aux) {
  vector_t vel = body_get_velocity(((aux_t*) aux)->body1);
  vector_t force = vec_multiply(((aux_t*) aux)->constant, (vector_t) \
    {-1 * vel.x, -1 * vel.y});
  body_add_force(((aux_t*) aux)->body1, force);
}

void collision_creator(void *aux) {
  if (find_collision((((aux_t*) aux)->body1)->shape, (((aux_t*) aux)->body2)->shape)) {
    body_remove(((aux_t*) aux)->body1);
    body_remove(((aux_t*) aux)->body2);
  }
}

void create_newtonian_gravity(scene_t *scene, double g, body_t *body1, body_t *body2) {
  aux_t *aux = malloc(sizeof(aux_t));
  aux->constant = g;
  aux->body1 = body1;
  aux->body2 = body2;
  scene_add_force_creator(scene, (force_creator_t) gravity_creator, \
    aux, free);
}

void create_spring(scene_t *scene, double k, body_t *body1, body_t *body2) {
  aux_t *aux = malloc(sizeof(aux_t));
  aux->constant = k;
  aux->body1 = body1;
  aux->body2 = body2;
  scene_add_force_creator(scene, (force_creator_t) spring_creator, \
   aux, free);
}

void create_drag(scene_t *scene, double gamma, body_t *body) {
  aux_t *aux = malloc(sizeof(aux_t));
  aux->constant = gamma;
  aux->body1 = body;
  aux->body2 = NULL;
  scene_add_force_creator(scene, (force_creator_t) drag_creator, \
  aux, free);
}

void create_destructive_collision(scene_t *scene, body_t *body1, body_t *body2) {
  aux_t *aux = malloc(sizeof(aux_t));
  aux->constant = 0;
  aux->body1 = body1;
  aux->body2 = body2;
  list_t *b_list = list_init(2, (free_func_t)body_free);
  list_add(b_list, body1);
  list_add(b_list, body2);
  scene_add_bodies_force_creator(scene, (force_creator_t) collision_creator, \
  aux, b_list, free);
}
