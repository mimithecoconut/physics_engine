#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include "forces.h"
#include "test_util.h"
#include "scene.h"

const double M = 10;
const double DT = 1e-6;
const double MIN_DISTANCE = 5.0;
const double G = 5;
const int STEPS = 100000;
const double INIT_V = 5;
const double GAMMA = 5;

list_t *make_shape() {
    list_t *shape = list_init(4, free);
    vector_t *v = malloc(sizeof(*v));
    *v = (vector_t){-1, -1};
    list_add(shape, v);
    v = malloc(sizeof(*v));
    *v = (vector_t){+1, -1};
    list_add(shape, v);
    v = malloc(sizeof(*v));
    *v = (vector_t){+1, +1};
    list_add(shape, v);
    v = malloc(sizeof(*v));
    *v = (vector_t){-1, +1};
    list_add(shape, v);
    return shape;
}
// Tests that a drag force acting on a mass operates correctly.
void test_drag() {
  scene_t *scene = scene_init();
  body_t *mass = body_init(make_shape(), M, (rgb_color_t){0, 0, 0});
  body_set_velocity(mass, (vector_t){1.0, 1.0});
  scene_add_body(scene, mass);
  create_drag(scene, GAMMA, mass);
  vector_t vel = body_get_velocity(mass);
  aux_t *aux = malloc(sizeof(aux_t));
  aux->constant = GAMMA;
  aux->body1 = mass;
  aux->body2 = NULL;
  drag_creator(aux);
  assert(vec_isclose(body_get_force(mass), (vector_t){GAMMA * -1 * vel.x,\
     GAMMA * -1 * vel.y}));
  scene_free(scene);
}

// Tests that a gravity force acting on two masses operates correctly.
void test_gravity() {
  scene_t *scene = scene_init();
  body_t *mass1 = body_init(make_shape(), M, (rgb_color_t){0, 0, 0});
  body_t *mass2 = body_init(make_shape(), M, (rgb_color_t){0, 0, 0});
  body_set_velocity(mass1, (vector_t){INIT_V, INIT_V});
  body_set_velocity(mass2, (vector_t){-INIT_V, -INIT_V});
  body_tick(mass1, DT);
  body_tick(mass2, DT);
  aux_t *aux = malloc(sizeof(aux_t));
  aux->constant = G;
  aux->body1 = mass1;
  aux->body2 = mass2;
  for (int j = 0; j < STEPS; j++) {
    gravity_creator(aux);
    vector_t pos1 = body_get_centroid(mass1);
    vector_t pos2 = body_get_centroid(mass2);
    double distance = sqrt(pow((pos2.x - pos1.x), 2) + pow((pos2.y - pos1.y), 2));
    if (distance > MIN_DISTANCE) {
      vector_t unit = (vector_t) {(pos2.x - pos1.x) / distance, \
        (pos2.y - pos1.y) / distance};
      double force_mag = ((aux_t *) aux)->constant * M * \
        M / pow(distance, 2);
      vector_t f1 = vec_multiply(force_mag, unit);
      vector_t f2 = vec_negate(vec_multiply(force_mag, unit));
      assert(vec_isclose(body_get_force(mass1), f1));
      assert(vec_isclose(body_get_force(mass2), f2));
    }
    body_tick(mass1, DT);
    body_tick(mass2, DT);
  }
  scene_free(scene);
  free(aux);
}

//Tests that a spring force acting on two masses operates correctly.
void test_spring_force() {
  const double M = 10;
  const double K = 2;
  const double DT = 1e-6;
  const int STEPS = 1000000;
  scene_t *scene = scene_init();
  body_t *mass0 = body_init(make_shape(), M, (rgb_color_t){0, 0, 0});
  body_t *mass1 = body_init(make_shape(), M, (rgb_color_t){0, 0, 0});
  body_set_velocity(mass0, (vector_t){1.0, 1.0});
  body_set_velocity(mass1, (vector_t){2.0, 2.0});
  body_tick(mass0, DT);
  body_tick(mass1, DT);
  aux_t *aux = malloc(sizeof(aux_t));
  aux->constant = K;
  aux->body1 = mass0;
  aux->body2 = mass1;
  for (int i = 0; i < STEPS; i++) {
    spring_creator(aux);
    vector_t pos1 = body_get_centroid(mass0);
    vector_t pos2 = body_get_centroid(mass1);
    vector_t force = vec_multiply(((aux_t*) aux)->constant, vec_subtract(pos2, pos1));
    assert(vec_isclose(body_get_force(mass0),force));
    assert(vec_isclose(body_get_force(mass1),vec_multiply(-1, force)));
    body_tick(mass0, DT);
    body_tick(mass1, DT);
  }
  scene_free(scene);
  free(aux);
}

int main(int argc, char *argv[]) {
  // Run all tests if there are no command-line arguments
  bool all_tests = argc == 1;
  // Read test name from file
  char testname[100];
  if (!all_tests) {
      read_testname(argv[1], testname, sizeof(testname));
  }
  DO_TEST(test_drag)
  DO_TEST(test_gravity)
  DO_TEST(test_spring_force)

  puts("students_test PASS");

}
