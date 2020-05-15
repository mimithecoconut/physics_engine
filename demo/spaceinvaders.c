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

const double HEIGHT = 1000.0;
const double WIDTH = 1000.0;
const vector_t START_POS = {500.0, 40.0};
const double RADIUS = 50.0;
const int INIT_LIST = 5;
const int CIRC_FRAC = 12;
const double MASS = 10.0;
const int OVAL_PTS = 48;
const int ENEM_START = 7;
const int ENEM_END = 18;
const double PLAYER_V = 500.0;
const double ENEM_GAP = 5.0;
const double ENEM_V = 100.0;
const int ROWS = 3;
const double SHOOT_DELAY = 0.8;
const double PLAYER_X_RAD = 60.0;
const double PLAYER_Y_RAD = 20.0;
const double PROJ_WIDTH = 10.0;
const double PROJ_HEIGHT = 20.0;

/**
 * Returns a pointer to a body representing an enemy object
 *
 * @param r radius of enemy expressed as a double
 * @param center initial starting position of bottom point of shape, expressed
 * as a vector
 * @return body_t pointer to enemy
 */
body_t *init_enemy(double r, vector_t center) {
    list_t *points = list_init(INIT_LIST, (free_func_t) free);
    vector_t *c = malloc(sizeof(vector_t));
    c->x = center.x;
    c->y = center.y;
    list_add(points, c);
    for (int i = ENEM_START; i < ENEM_END; i++) {
        vector_t *point = malloc(sizeof(vector_t));
        point->x = center.x + r * cos(i * M_PI / CIRC_FRAC);
        point->y = center.y + r * sin(i * M_PI / CIRC_FRAC);
        list_add(points, point);
    }
    char *status = malloc(sizeof(char));
    *status = 'e';
    body_t *toReturn = body_init_with_info(points, MASS, \
      (rgb_color_t) {0.8, 0.8, 0.8}, status, free);
    body_set_rotation(toReturn, 3 * M_PI / 2);
    return toReturn;
}

/**
 * Returns a pointer to a body representing an oval
 *
 * @param smin semiminor axis of oval
 * @param smaj semimajor axis of oval
 * @param center initial starting position of center of oval, as a vector
 * @return body_t pointer to oval
 */
body_t *init_oval(double smin, double smax, vector_t center) {
    list_t *points = list_init(INIT_LIST, (free_func_t) free);
    for (int i = 0; i < OVAL_PTS * 2; i++) {
        vector_t *point = malloc(sizeof(vector_t));
        point->x = center.x + smin * cos(i * M_PI / OVAL_PTS);
        point->y = center.y + smax * sin(i * M_PI / OVAL_PTS);
        list_add(points, point);
    }
    char *status = malloc(sizeof(char));
    *status = 'p';
    body_t *toReturn = body_init_with_info(points, MASS, \
      (rgb_color_t) {0, 1, 0}, status, free);
    body_set_rotation(toReturn, M_PI / 2);
    return toReturn;
}

/**
 * Returns a pointer to a body representing a projectile
 *
 * @param w width of projectile
 * @param h height of projectile
 * @param centroid coordinates of centroid of projectile
 * @return body_t pointer to projectile
 */
body_t *init_projectile(double w, double h, vector_t centroid) {
    list_t *points = list_init(INIT_LIST, (free_func_t) free);
    vector_t *c1 = malloc(sizeof(vector_t));
    c1->x = 0;
    c1->y = 0;
    list_add(points, c1);
    vector_t *c2 = malloc(sizeof(vector_t));
    c2->x = w;
    c2->y = 0;
    list_add(points, c2);
    vector_t *c3 = malloc(sizeof(vector_t));
    c3->x = w;
    c3->y = h;
    list_add(points, c3);
    vector_t *c4 = malloc(sizeof(vector_t));
    c4->x = 0;
    c4->y = h;
    list_add(points, c4);
    char *status = malloc(sizeof(char));
    *status = 's';
    body_t *toReturn = body_init_with_info(points, MASS, \
      (rgb_color_t) {1, 1, 0}, status, free);
    body_set_centroid(toReturn, centroid);
    return toReturn;
}

/**
 * Initializes rows of enemies on screen
 *
 * @param scene the scene to add enemies to
 */
void init_enemies(scene_t *scene) {
    for (int j = 0; j < ROWS; j++) {
        for (int i = 1; i < WIDTH / (2 * RADIUS + ENEM_GAP); i++) {
            vector_t pos = (vector_t) {(2 * RADIUS + ENEM_GAP) * i, \
                            HEIGHT - (RADIUS + ENEM_GAP) * ((double) j + 0.5)};
            body_t *e = init_enemy(RADIUS, pos);
            body_set_velocity(e, (vector_t) {ENEM_V, 0});
            scene_add_body(scene, e);
            for (size_t j = 0; j < scene_bodies(scene); j++) {
                body_t *b = scene_get_body(scene, j);
                if (*(char*)(body_get_info(b)) == 'p') {
                    create_destructive_collision(scene, b, e);
                }
            }
        }
    }
}

/**
 * Shoots projectile from centroid of specified object, in direction of
 * object orientation
 *
 * @param scene the scene to shoot the projectile on
 * @param obj body that shoots the projectile
 */
void shoot_projectile(scene_t *scene, body_t *obj) {
    body_t *proj = init_projectile(PROJ_WIDTH, PROJ_HEIGHT, \
      body_get_centroid(obj));
    body_set_color(proj, body_get_color(obj));
    double v_y = PLAYER_V * sin(body_get_orientation(obj));
    body_set_velocity(proj, (vector_t) {0, v_y});
    scene_add_body(scene, proj);
    for (size_t j = 0; j < scene_bodies(scene); j++) {
        body_t *b = scene_get_body(scene, j);
        if (*(char*)(body_get_info(b)) == 'e' && *(char*)(body_get_info(obj)) == 'p') {
            create_destructive_collision(scene, b, proj);
        }
        else if (*(char*)(body_get_info(b)) == 'p' && *(char*)(body_get_info(obj)) == 'e') {
            create_destructive_collision(scene, b, proj);
        }
    }
}

/**
 * Prevents player from going offscreen
 *
 * @param player body representing the player
 */
void bound(body_t *player) {
    vector_t curr_pos = body_get_centroid(player);
    if (curr_pos.x + PLAYER_X_RAD > WIDTH) {
        body_set_centroid(player, (vector_t) {WIDTH - PLAYER_X_RAD, curr_pos.y});
    }
    else if (curr_pos.x - PLAYER_X_RAD < 0) {
        body_set_centroid(player, (vector_t) {PLAYER_X_RAD, curr_pos.y});
    }
}

/**
 * Moves enemy to the next row if it moves offscreen
 *
 * @param enemy body representing enemy
 */
void wrap_enemy(body_t *enemy) {
    vector_t curr_pos = body_get_centroid(enemy);
    if (curr_pos.x + RADIUS > WIDTH) {
        body_set_centroid(enemy, (vector_t) {WIDTH - PLAYER_X_RAD, \
          curr_pos.y - ROWS * (RADIUS + ENEM_GAP)});
          body_set_velocity(enemy, (vector_t) {-1 * body_get_velocity(enemy).x, 0});
    }
    else if (curr_pos.x - RADIUS < 0) {
        body_set_centroid(enemy, (vector_t) {RADIUS, curr_pos.y - ROWS * \
          (RADIUS + ENEM_GAP)});
        body_set_velocity(enemy, (vector_t) {-1 * body_get_velocity(enemy).x, 0});
    }
}

/**
 * Returns true if the body is offscreen
 *
 * @param body the body to check
 */
bool is_offscreen(body_t *body) {
    list_t *pts = body_get_shape(body);
    bool ans = true;
    for (size_t i = 0; i < list_size(pts); i++) {
        vector_t *p = (vector_t*) list_get(pts, i);
        if (p->x >= 0 && p->x <= WIDTH && p->y >=0 && p->y <= HEIGHT) {
            ans = false;
        }
    }
    return ans;
}

/**
 * Updates edge behavior of enemies and player
 *
 * @param scene the scene containing the bodies to update
 */
void update_edge(scene_t *scene) {
    for (size_t j = 0; j < scene_bodies(scene); j++) {
        body_t *b = scene_get_body(scene, j);
        if (*(char*)(body_get_info(b)) == 'e') {
            wrap_enemy(b);
        }
        else if (*(char*)(body_get_info(b)) == 'p') {
            bound(b);
        }
        else {
            if (is_offscreen(b)) {
                body_remove(b);
            }
        }
    }
}

/**
 * Adjusts the velocity of the player based on keyboard input
 *
 * @param key representing which key is KEY_PRESSED
 * @param type representing if key is KEY_PRESSED
 * @param held_time representing how long key is held
 * @param body representing player object
 * @param s representing current scene
 */
void on_key(char key, key_event_type_t type, double held_time, void* player, \
  void *s) {
    double angle = body_get_orientation(player);
    bool press = false;
    switch (type) {
        case KEY_RELEASED:
            switch (key) {
                case LEFT_ARROW:
                    body_set_velocity(player, VEC_ZERO);
                    break;
                case RIGHT_ARROW:
                    body_set_velocity(player, VEC_ZERO);
                    break;
            }
            break;
        case KEY_PRESSED:
            switch (key) {
                case LEFT_ARROW:
                    angle = M_PI;
                    press = true;
                    break;
                case RIGHT_ARROW:
                    angle = 0;
                    press = true;
                    break;
                case SPACE:
                    shoot_projectile(s, player);
                    break;
            }
        if (press) {
            double new_x = PLAYER_V * cos(angle);
            double new_y = PLAYER_V * sin(angle);
            vector_t new_v = {new_x, new_y};
            body_set_velocity(player, new_v);
        }
    }
}

/**
 * Returns whether there are enemies remaining in the scene
 *
 * @param scene the scene to check
 * @return true if enemies remaining, false otherwise
 */
bool enemies_remaining(scene_t *scene) {
    for (size_t j = 0; j < scene_bodies(scene); j++) {
        body_t *b = scene_get_body(scene, j);
        if (*(char*)(body_get_info(b)) == 'e') {
            return true;
        }
    }
    return false;
}

/**
 * Returns the index of a random enemy in the scene.
 *
 * @param scene the scene to return an enemy from
 * @return an index to an enemy in scene->bodies
 */
int get_random_enemy(scene_t *scene) {
    int rand_ind;
    do {
        rand_ind = rand() % scene_bodies(scene);
    } while (*(char*)(body_get_info(scene_get_body(scene, rand_ind))) != \
      'e');
    return rand_ind;
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
    vector_t max = {WIDTH, HEIGHT};
    sdl_init(min, max);
    scene_t *scene = scene_init();
    body_t *player = init_oval(PLAYER_Y_RAD, PLAYER_X_RAD, START_POS);
    scene_add_body(scene, player);
    init_enemies(scene);
    double total_time_elapsed = 0.0;
    sdl_on_key((key_handler_t) on_key, player, scene);
    while (!sdl_is_done()) {
        double time_elapsed = time_since_last_tick();
        total_time_elapsed += time_elapsed;
        if (total_time_elapsed > SHOOT_DELAY) {
            total_time_elapsed = 0.0;
            if (enemies_remaining(scene)) {
                int rand_ind = get_random_enemy(scene);
                shoot_projectile(scene, scene_get_body(scene, rand_ind));
            }
        }
        update_edge(scene);
        scene_tick(scene, time_elapsed);
        sdl_render_scene(scene);
    }
    scene_free(scene);
    return 0;
}
