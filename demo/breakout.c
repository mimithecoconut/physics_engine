#include "sdl_wrapper.h"
#include "vector.h"
#include "body.h"
#include "scene.h"
#include "list.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "forces.h"
#include "collision.h"

const double HEIGHT = 1000.0;
const double WIDTH = 1000.0;
const vector_t START_POS = {500.0, 40.0};
const double BLOCK_W = 90.0;
const double BLOCK_H = 40.0;
const double BLOCK_GAP = 9.0;
const int INIT_LIST = 5;
const double MASS = 10.0;
const double PADDLE_V = 500.0;
const int ROWS = 3;
const double PADDLE_W = 80.0;
const double PADDLE_H = 30.0;
const int CIRC_PTS = 48;
const int CIRC_FRAC = 24;
const double BALL_R = 10.0;
const vector_t BALL_V_RANGE = {-500.0, 500.0};
const vector_t BALL_POS_RANGE = {200.0, 300.0};
const double ELASTICITY = 1.0;
const double BALL_DELAY = 10.0;
const double WALL_THICKNESS = 50.0;

/**
 * Returns a list of rgb_color_t pointers in rainbow order
 *
 * @return list_t of rainbow colors
 */
list_t *init_rainbow() {
    list_t *ans = list_init(INIT_LIST, free);
    rgb_color_t *red = malloc(sizeof(rgb_color_t));
    red->r = 1, red->g = 0, red->b = 0;
    rgb_color_t *orange = malloc(sizeof(rgb_color_t));
    orange->r = 1, orange->g = 0.5, orange->b = 0;
    rgb_color_t *yellow = malloc(sizeof(rgb_color_t));
    yellow->r = 1, yellow->g = 1, yellow->b = 0;
    rgb_color_t *green1 = malloc(sizeof(rgb_color_t));
    green1->r = 0.5, green1->g = 1, green1->b = 0;
    rgb_color_t *green2 = malloc(sizeof(rgb_color_t));
    green2->r = 0, green2->g = 1, green2->b = 0;
    rgb_color_t *blue1 = malloc(sizeof(rgb_color_t));
    blue1->r = 0, blue1->g = 1, blue1->b = 1;
    rgb_color_t *blue2 = malloc(sizeof(rgb_color_t));
    blue2->r = 0, blue2->g = 0, blue2->b = 1;
    rgb_color_t *purple = malloc(sizeof(rgb_color_t));
    purple->r = 0.5, purple->g = 0, purple->b = 1;
    rgb_color_t *pink = malloc(sizeof(rgb_color_t));
    pink->r = 1, pink->g = 0, pink->b = 1;
    rgb_color_t *magenta = malloc(sizeof(rgb_color_t));
    magenta->r = 1, magenta->g = 0, magenta->b = 0.5;
    list_add(ans, red);
    list_add(ans, orange);
    list_add(ans, yellow);
    list_add(ans, green1);
    list_add(ans, green2);
    list_add(ans, blue1);
    list_add(ans, blue2);
    list_add(ans, purple);
    list_add(ans, pink);
    list_add(ans, magenta);
    return ans;
}

/**
 * Generates a random vector with x and y in the given range
 *
 * @param range the range of values that it can span
 * @return vector_t with random values for x and y
 */
vector_t generate_vector(vector_t range) {
    double x = (double) (rand() % (int) (range.y - range.x)) + range.x;
    double y = (double) (rand() % (int) (range.y - range.x)) + range.x;
    return (vector_t) {x, y};
}

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
    char *status = malloc(sizeof(char));
    *status = 'c';
    return body_init_with_info(points, MASS, (rgb_color_t) {1, 0, 0}, status,
      free);
}

/**
 * Returns a pointer to a body representing a rectangle
 *
 * @param width the width of rectangle
 * @param height the height of rectangle
 * @param center initial starting position of centroid of rectangle, as a vector
 * @param s status of body (w for wall, b for block)
 * @return body_t pointer to rectangle
 */
body_t *init_rectangle(double width, double height, vector_t centroid, char s) {
    list_t *points = list_init(INIT_LIST, (free_func_t) free);
    vector_t *c1 = malloc(sizeof(vector_t));
    c1->x = 0;
    c1->y = 0;
    list_add(points, c1);
    vector_t *c2 = malloc(sizeof(vector_t));
    c2->x = width;
    c2->y = 0;
    list_add(points, c2);
    vector_t *c3 = malloc(sizeof(vector_t));
    c3->x = width;
    c3->y = height;
    list_add(points, c3);
    vector_t *c4 = malloc(sizeof(vector_t));
    c4->x = 0;
    c4->y = height;
    list_add(points, c4);
    char *status = malloc(sizeof(char));
    *status = s;
    body_t *toReturn = body_init_with_info(points, INFINITY,
      (rgb_color_t) {1, 0, 0}, status, free);
    body_set_centroid(toReturn, centroid);
    return toReturn;
}

/**
 * Initializes bodies offscreen representing walls bounding the windows
 *
 * @param scene the scene to add blocks to
 */
void init_walls(scene_t *scene) {
    body_t *left_wall = init_rectangle(WALL_THICKNESS, HEIGHT,
      (vector_t) {-WALL_THICKNESS / 2, HEIGHT / 2}, 'w');
    body_t *right_wall = init_rectangle(WALL_THICKNESS, HEIGHT,
      (vector_t) {HEIGHT + WALL_THICKNESS / 2, HEIGHT / 2}, 'w');
    body_t *top_wall = init_rectangle(WIDTH, WALL_THICKNESS,
      (vector_t) {WIDTH / 2, HEIGHT + WALL_THICKNESS / 2}, 'w');
    body_set_color(left_wall, (rgb_color_t) {1, 1, 1});
    body_set_color(right_wall, (rgb_color_t) {1, 1, 1});
    body_set_color(top_wall, (rgb_color_t) {1, 1, 1});
    scene_add_body(scene, left_wall);
    scene_add_body(scene, right_wall);
    scene_add_body(scene, top_wall);
}

/**
 * Initializes rows of blocks on screen
 *
 * @param scene the scene to add blocks to
 */
void init_blocks(scene_t *scene) {
    list_t *colors = init_rainbow();
    for (int j = 0; j < ROWS; j++) {
        for (int i = 0; i < WIDTH / (BLOCK_W + BLOCK_GAP) - 1; i++) {
            vector_t pos = (vector_t) {(BLOCK_W / 2 + BLOCK_GAP) + i *
              (BLOCK_W + BLOCK_GAP), HEIGHT - ((BLOCK_H / 2 + BLOCK_GAP) + j *
              (BLOCK_H + BLOCK_GAP))};
            body_t *e = init_rectangle(BLOCK_W, BLOCK_H, pos, 'b');
            body_set_color(e, *(rgb_color_t *)list_get(colors, i));
            scene_add_body(scene, e);
        }
    }
    list_free(colors);
}

/**
 * Adds collisions between the ball and the other bodies in the scene
 *
 * @param ball body to be bounced
 * @param scene the scene to add collisions to
 */
void add_collisions(body_t *ball, scene_t *scene) {
    for (size_t i = 0; i < scene_bodies(scene); i++) {
        if (*(char *)body_get_info(scene_get_body(scene, i)) != 'c') {
            create_physics_collision(scene, ELASTICITY, ball,
              scene_get_body(scene, i));
        }
    }
}

/**
 * Prevents paddle from going offscreen
 *
 * @param paddle body representing the paddle
 */
void bound(body_t *paddle) {
    vector_t curr_pos = body_get_centroid(paddle);
    if (curr_pos.x + PADDLE_W / 2 > WIDTH) {
        body_set_centroid(paddle, (vector_t) {WIDTH - PADDLE_W / 2, curr_pos.y});
    }
    else if (curr_pos.x - PADDLE_W / 2 < 0) {
        body_set_centroid(paddle, (vector_t) {PADDLE_W / 2, curr_pos.y});
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
 * Adds a ball to the scene with a random velocity and position
 *
 * @param scene the scene containing the game
 * @param b_list list of pointers to balls
 */
void add_ball(scene_t *scene, list_t *b_list) {
    body_t *ball = init_circle(BALL_R, BALL_POS_RANGE);
    body_set_velocity(ball, generate_vector(BALL_V_RANGE));
    body_set_centroid(ball, generate_vector(BALL_POS_RANGE));
    list_add(b_list, ball);
    scene_add_body(scene, ball);
    add_collisions(ball, scene);
}

/**
 * Resets the game by re-initializing enemies and moving the paddle to its
 * starting position, scene only contains one ball
 *
 * @param scene the scene containing the game
 * @param paddle body representing paddle object
 * @param b_list list of pointers to balls
 */
void reset_game(scene_t *scene, body_t *paddle, list_t *b_list) {
    body_set_centroid(paddle, START_POS);
    init_walls(scene);
    init_blocks(scene);
    add_ball(scene, b_list);
}

/**
 * Checks if there are any collisions between the ball and blocks, and sets
 * block to be removed if so
 *
 * @param scene the scene containing the game
 * @param ball body representing the ball
 */
void check_block_collision(scene_t *scene, body_t *ball) {
    for (size_t i = 0; i < scene_bodies(scene); i++) {
        body_t *body = scene_get_body(scene, i);
        if (*(char *)body_get_info(body) == 'b') {
            if (find_collision(body_get_shape(body),
              body_get_shape(ball)).collided) {
                body_remove(body);
            }
        }
    }
 }

/**
 * Checks collisions for all balls in the scene
 *
 * @param scene the scene containing the game
 * @param b_list list of pointers to balls
 */
void check_all_collisions(scene_t *scene, list_t *b_list) {
   for (size_t j = 0; j < list_size(b_list); j++) {
      check_block_collision(scene, list_get(b_list, j));
   }
}

/**
 * Returns whether any balls are offscreen
 *
 * @param b_list list of pointers to balls
 */
bool any_ball_offscreen(list_t *b_list) {
  for (size_t j = 0; j < list_size(b_list); j++) {
     if (is_offscreen(list_get(b_list, j))) {
        return true;
     }
  }
  return false;
}

/**
 * Removes all bodies from the scene other than paddle and ball
 *
 * @param scene the scene containing the game
 * @param b_list list of pointers to balls
 */
void scene_clear(scene_t *scene, list_t *b_list) {
    for (size_t j = 0; j < scene_bodies(scene); j++) {
        body_t *body = scene_get_body(scene, j);
        if (*(char *)body_get_info(body) != 'p') {
            body_remove(body);
        }
    }
    for (int i = (int) list_size(b_list) - 1; i >= 0; i--) {
        list_remove(b_list, i);
    }
    scene_tick(scene, 0);
}

/**
 * Adjusts the velocity of the paddle based on keyboard input
 *
 * @param key representing which key is KEY_PRESSED
 * @param type representing if key is KEY_PRESSED
 * @param held_time representing how long key is held
 * @param paddle body representing paddle object
 * @param s representing current scene
 */
void on_key(char key, key_event_type_t type, double held_time, void* paddle,
  void *s) {
    double angle = body_get_orientation(paddle);
    bool press = false;
    switch (type) {
        case KEY_RELEASED:
            switch (key) {
                case LEFT_ARROW:
                    body_set_velocity(paddle, VEC_ZERO);
                    break;
                case RIGHT_ARROW:
                    body_set_velocity(paddle, VEC_ZERO);
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
            }
        if (press) {
            double new_x = PADDLE_V * cos(angle);
            double new_y = PADDLE_V * sin(angle);
            vector_t new_v = {new_x, new_y};
            body_set_velocity(paddle, new_v);
        }
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
    vector_t max = {WIDTH, HEIGHT};
    sdl_init(min, max);
    scene_t *scene = scene_init();
    list_t *ball_list = list_init(5, (free_func_t) body_free);
    body_t *paddle = init_rectangle(PADDLE_W, PADDLE_H, START_POS, 'p');
    scene_add_body(scene, paddle);
    reset_game(scene, paddle, ball_list);
    sdl_on_key((key_handler_t) on_key, paddle, scene);
    double total_time_elapsed = 0.0;
    while (!sdl_is_done()) {
        double time_elapsed = time_since_last_tick();
        total_time_elapsed += time_elapsed;
        if (total_time_elapsed > BALL_DELAY) {
            total_time_elapsed = 0.0;
            add_ball(scene, ball_list);
        }
        check_all_collisions(scene, ball_list);
        scene_tick(scene, time_elapsed);
        if (any_ball_offscreen(ball_list)) {
            total_time_elapsed = 0.0;
            scene_clear(scene, ball_list);
            reset_game(scene, paddle, ball_list);
        }
        bound(paddle);
        sdl_render_scene(scene);
    }
    free(ball_list);
    scene_free(scene);
    return 0;
}
