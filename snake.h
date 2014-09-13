/* *****************************************************************************
 *
 * ************************************************************************** */

//#include <allegro.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>


/* *****************************************************************************
 *
 * ************************************************************************** */
//#define MAP_HEIGHT 48
//#define MAP_WIDTH 64
#define MAP_HEIGHT  16
#define MAP_WIDTH   8

#define TILE_SIZE 10
//32x24 tiles, 20 px in size


/* *****************************************************************************
 *
 * ************************************************************************** */
typedef enum
{
	KEY_LEFT = 0, KEY_RIGHT, KEY_UP, KEY_DOWN, KEY_ESC, KEY_end
}t_key;

typedef enum
{
	left = 0, right, up, down, none
}direction;

typedef enum
{
	nothing = 0, apple, snake
}object;

typedef struct snake_node
{
	int dir;
	int x;
	int y;
	struct snake_node *next;
}snake_node;


char  key [KEY_end-1];


 
/* *****************************************************************************
 *
 * ************************************************************************** */
void snake_game_over();
void snake_move(snake_node* player);
snake_node* snake_move_body(snake_node* player, int tempx, int tempy);
void snake_generate_new_apple();
void snake_timer1(void);
void snake_rest1(void);
void snake_draw();
int  snake_main();


/* *****************************************************************************
 *
 * ************************************************************************** */
extern int objMap[MAP_HEIGHT][MAP_WIDTH]; //This will contain all the objects, will use it to keep track of collisions

//BITMAP *buffer;//This will be our temporary bitmap for double buffering
int score;//score for the game
int gamespeed; //this is a value between 1 and 3


/* *****************************************************************************
 *
 * ************************************************************************** */
//timer variables
volatile int counter;
volatile int ticks;
volatile int framerate;
volatile int resting, rested;



