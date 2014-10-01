
#include "TM4C1294_leds.h"
#include "snake.h"

/*
*   Snake: First game made with allegro.
*   Last Update: Dec 4, 2010
*   Author: Stephen Bryant
*/


/* *****************************************************************************
 * self explanatory function. May update it later
 * ************************************************************************** */
void snake_game_over ( void ) {
	//printf("\ngame over. score = %d\n",score);
	//exit(0);
}

extern t_sys_values sys;


/* *****************************************************************************
 *
 * ************************************************************************** */
void snake_move (snake_node * player) {

	if (player==NULL) return;

    static int tempx=0, tempy=0;
    //char one_shot=0;

    //this contains the array of flags which tell which button has been pressed.
	//It must be cleared before every input.
    //clear_keybuf();
    /*
    if (key[KEY_LEFT])	player->dir = left;
    if (key[KEY_RIGHT]) player->dir = right;
    if (key[KEY_DOWN])  player->dir = down;
    if (key[KEY_UP])    player->dir = up;
    */
	if ( sys.key_pressed==1 )
	{
	    sys.key_pressed=0;

		if (sys.key3) player->dir = left;
		else
	    if (sys.key2) player->dir = up;
		else
	    if (sys.key1) player->dir = down;
		else
	    if (sys.key0) player->dir = right;
		else
			player->dir = none;

	    tempx = player->x;
	    tempy = player->y;

		// <<== left
		if (player->dir == none) { }
		else
		// <<== left
		if (player->dir == left) {
			if (player->x > 0) {
					tempx = player->x - 1;
			}
			else{ //allow for wrap around
				snake_game_over();
			}
		}
		// ==>> right
		else if (player->dir == right) {
			if (player->x < MAP_WIDTH - 1) {
					tempx = player->x + 1;
			} else{
				snake_game_over();
			}
		}
	    // ^^ UP
	    else if (player->dir == up) {
			if (player->y > 0) 		{
				tempy = player->y - 1;
			} else{
				snake_game_over();
			}
	    }
		// VV down
	    else if (player->dir == down) {
	        if (player->y < MAP_HEIGHT - 1) 		{
					tempy = player->y + 1;
			}
			else{
				snake_game_over();
			}
	    } else {
	        exit(0);
	    }

	    if ( objMap[tempy][tempx]==apple ) { //the snake has run into an apple and another node is created
	    	score++;
	        snake_node*  temp = player;
			while (temp->next != NULL) { //the snake is essentially a linked list and we're traversing it
				temp = temp->next;
			}
			int newNodex = temp->x; //when we reach the final node, we store its location with two variables
			int newNodey = temp->y;

			player = snake_move_body( player, tempx, tempy );
			//snake_node* newNode = malloc(sizeof(snake_node)); //after we move the whole body we make a new node line( screen, 130, 130, 150, 150, makecol( 255, 0, 0));
			snake_node newNode; //after we move the whole body we make a new node line( screen, 130, 130, 150, 150, makecol( 255, 0, 0));
			newNode.x = newNodex;
			newNode.y = newNodey; //we set this new node's location variables
			newNode.dir = temp->dir;
			newNode.next = NULL;
			temp->next = &newNode; //we set the temp node (the previous tail)'s next node to our new tail!
			snake_generate_new_apple();
	    }
	    else
	    if (objMap[tempy][tempx] == snake) {
	    	snake_game_over();
	    } else {
	    	player = snake_move_body ( player, tempx, tempy );
	    }

	    objMap[tempy][tempx] = snake; //update the object map to the new snake head position
	}

   // drv_led_8x8_pixel_set ( player->x, player->y, 1 );
}


/* *****************************************************************************
 *  This function recursively moves the snake. It goes all the way to the last
 *  node of the snake, changes its coordinates,
 *  and then changes its coordinates, all the way up.
 * ************************************************************************** */
snake_node* snake_move_body(snake_node* player, int tempx, int tempy) {
	if (player->next == NULL) //we've reached the end of the snake's body
	{
		objMap[player->y][player->x] = nothing; //reset the object map at this position
	}
	else //we need to keep traversing
	{
		player->next = snake_move_body(player->next,player->x,player->y);
	}
	player->x = tempx;
	player->y = tempy;

	return player;
}


/* *****************************************************************************
 *
 * ************************************************************************** */
void snake_generate_new_apple(void) {
	int randy, randx, tmp;

	srand(100);
	do {
		tmp = rand();
		tmp = tmp%8;
		randy = tmp;

		tmp = rand();
		tmp = tmp%16;
		randx = tmp;
	} while (objMap[randy][randx] != nothing); //while we generate a spot that's taken, keep going;

	objMap[randy][randx] = apple;
}


/* *****************************************************************************
 * calculate framerate every second
 * ************************************************************************** */
/*
void snake_timer1 ( void ) {
    counter++;
    framerate = ticks;
    ticks=0;
    rested=resting;
}
END_OF_FUNCTION(timer1)
*/


/* *****************************************************************************
 * do something while resting (?)
 * ************************************************************************** */
/*
void snake_rest1(void) {
    resting++;
}
*/


/* *****************************************************************************
 * this is our main draw function, called every loop.
 * ************************************************************************** */
void snake_draw ( void ) {
    int i,j;
    //acquire_screen();	//O(N^2) runtime for this, 24^2 is pretty big.. so we may change this
                        //but for now, we draw every tile every frame!
    for (i = 0; i < MAP_HEIGHT; i++) {
		for (j = 0; j < MAP_WIDTH; j++) {
			//int c; //color to draw

			if ( objMap[i][j] == nothing ) {
			//	c = makecol(0,0,0);
			}
			else if ( objMap[i][j] == apple ) {
			//	c = makecol(255,0,0);
			}
			else if ( objMap[i][j] == snake ) {
			//	c = makecol(0,255,0);
			}
			//rectfill ( buffer, TILE_SIZE*j, TILE_SIZE*i, TILE_SIZE*(j+1),TILE_SIZE*(i+1), c);
		}
    }

/*
    //draw the score
    char scoretxt[10];
    sprintf(scoretxt,"score: %d",score);
    textout_ex(buffer, font, scoretxt, TILE_SIZE*(MAP_WIDTH)*3/4, TILE_SIZE, makecol(255,255,255), makecol(0,0,0));

    //draw an outline of the game map
    rect( buffer, 0, 0, TILE_SIZE*MAP_WIDTH-1, TILE_SIZE*MAP_HEIGHT-1, makecol( 0, 0, 255));

    draw_sprite( screen, buffer, 0, 0); //draw buffer image on screen every time we draw.
    release_screen();
*/
}


/* *****************************************************************************
 *
 * ************************************************************************** */
int snake_main ( void ) {
/*
    //basic info from user about game speed.
    printf("\nSet your game speed (1,2,3): \n");
    scanf("%d",&gamespeed);
    score = 0;

    allegro_init();
    install_keyboard();

    set_color_depth(16); //graphics
    set_gfx_mode( GFX_AUTODETECT, MAP_WIDTH * TILE_SIZE, MAP_HEIGHT * TILE_SIZE, 0, 0);
    buffer = create_bitmap( MAP_WIDTH * TILE_SIZE, MAP_HEIGHT * TILE_SIZE);

    install_timer();

    //lock interrupt variables

    LOCK_VARIABLE(counter);
    LOCK_VARIABLE(framerate);
    LOCK_VARIABLE(ticks);
    LOCK_VARIABLE(resting);
    LOCK_VARIABLE(rested);
    LOCK_FUNCTION(snake_timer1);
    LOCK_FUNCTION(rest1);

    install_int(snake_timer1, 1000);
*/

	static char snake_init = 0;

	//snake_node* player = malloc(sizeof(snake_node));
	static snake_node  player = {0};

	//if ( player==NULL ) return -1;

	if (snake_init==0)
	{
		player.dir =  left; //init direction
		player.x = MAP_WIDTH/2;
		player.y = MAP_HEIGHT/2;
		player.next = NULL;

		int i, j; //this is so we don't get errors because the object map references nothing.
		for (i = 0; i < MAP_HEIGHT; i++) {
			for (j = 0; j < MAP_WIDTH; j++) {
				objMap[i][j] = nothing;
			}
		}
		//we need to seed our rand() and generate our first random object
		//srand(time(NULL));

		srand(time(NULL));
		snake_generate_new_apple();

        snake_init=1;
    }

    if (snake_init==1)
    //while ( !key[KEY_ESC] )
    { //this is our main game loop
        snake_move(&player);
        snake_draw();

        //update ticks
        ticks++;

        //slow the game down
        resting=0;

        //this formula is hacked together. This number is the one which determines how slow the game is. The
        //higher the number the longer we wait. So higher gamespeed means a lower wait.
        //rest_callback(100-gamespeed * 30, rest1);
    }

    return 0;
}
//END_OF_MAIN();

