#include "collision.h"
#include <stdbool.h>
#include "list.h"
#include <stdlib.h>
#include <stdio.h>
#include "body.h"
#include "vector.h"
#include <assert.h>

bool find_collision(list_t *shape1, list_t *shape2) {
  list_t *axes = get_axes2(shape1, shape2);
  for (size_t i = 0; i < list_size(axes); i++) {
    double min1 = polygon_proj_min(shape1, list_get(axes, i));
    double max1 = polygon_proj_max(shape1, list_get(axes, i));
    double min2 = polygon_proj_min(shape2, list_get(axes, i));
    double max2 = polygon_proj_max(shape2, list_get(axes, i));
    if ((max2 < min1) || (max1 < min2)) {
      return false;
    }
  }
  return true;
}

vector_t *edge_perp(vector_t *vec) {
  vector_t *vec_normal = malloc(sizeof(vector_t));
  vec_normal->x =  -1 * vec->y;
  vec_normal->y = vec->x;
  return vec_normal;
}

list_t *get_axes1(list_t *shape) {
  size_t len = list_size(shape);
  list_t *result = list_init(len, free);
  for (size_t i = 0; i < len; i++) {
    vector_t *vec = malloc(sizeof(vector_t));
    *vec = *(vector_t *)(list_get(shape, i % len));
    *vec = vec_subtract(*vec, *(vector_t *)(list_get(shape, (i + 1) % len)));
    vector_t* vec_normal = edge_perp(vec);
    list_add(result, (void *) vec_normal);
  }
  return result;
}
list_t *get_axes2(list_t *shape1, list_t *shape2) {
  list_t *result = list_init(list_size(shape1) + list_size(shape2), free);
  list_t *axes1 = get_axes1(shape1);
  list_t *axes2 = get_axes1(shape2);
  for (size_t i = 0; i < list_size(shape1); i++) {
    list_add(result, list_get(axes1, i));
  }
  for (size_t i = 0; i < list_size(shape2); i++) {
    list_add(result, list_get(axes2, i));
  }
  return result;
}

double vertex_proj(vector_t *vertex, vector_t *line) {
  return vec_dot(*vertex, *line);
}

double polygon_proj_min(list_t *shape, vector_t *line) {
  double min = vec_dot(*((vector_t*) list_get(shape, 0)), *line);
  for (size_t i = 0; i < list_size(shape); i++) {
    if (vec_dot(*((vector_t*) list_get(shape, i)), *line) < min) {
      min = vec_dot(*((vector_t*) list_get(shape, i)), *line);
    }
  }
  return min;
}

double polygon_proj_max(list_t *shape, vector_t *line) {
  double max = vec_dot(*((vector_t*) list_get(shape, 0)), *line);
  for (size_t i = 0; i < list_size(shape); i++) {
    if (vec_dot(*((vector_t*) list_get(shape, i)), *line) > max) {
      max = vec_dot(*((vector_t*) list_get(shape, i)), *line);
    }
  }
  return max;
}
