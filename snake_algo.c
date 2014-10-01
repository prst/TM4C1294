/* *****************************************************************************
 *
 * ************************************************************************** */

#include <stdlib.h>

#include "TM4C1294_leds.h"

#include "snake.h"


/* *****************************************************************************
 *
 * ************************************************************************** */
extern  uint8_t   a_field [8][16];
extern  uint8_t   a_field_eat [8][16];
extern  uint8_t   a_field_eat_16b [16];


/* *****************************************************************************
 *
 * ************************************************************************** */
t_ret_code algo_Snake ( t_io_values *p_io,  t_io_Snake *p_snake );

int objMap[MAP_HEIGHT][MAP_WIDTH]; //This will contain all the objects, will use it to keep track of collisions


/* *****************************************************************************
 *
 * ************************************************************************** */
// Fill field as random
uint32_t rnd_fill_field ( uint8_t in_x ) {
	uint8_t      yy;
	uint32_t     rnd;
	uint32_t     rnd_tmp;

	srand ( in_x );

	//for ( xx=0; xx<8; xx++ )
//{
	for ( yy=0; yy<16; yy++ ) {
		rnd = (uint8_t)rand(); // get random number
		//if (!(rnd&0x01)) rnd&=0x33; else rnd&=0xCC;
		//if (!(rnd&0x02)) rnd&=0x66; else rnd&=0x99;
		//if (!(rnd&0x04)) rnd&=0x24; else rnd&=0x42;
		//if (!(rnd&0x08)) rnd&=0x88; else rnd&=0x11;

		rnd_tmp = rnd;
		//a_field_eat [xx][yy] = ( rnd_tmp < 8 ) ? 1 : 0;
		a_field_eat_16b[yy] = rnd_tmp;
	}

	return  rnd_tmp;
}



/* *****************************************************************************
 *
 * ************************************************************************** */
t_ret_code algo_Snake ( t_io_values *p_io,  t_io_Snake *p_snake ) {
/*
 * ) Set startup values (Random?)
 * )     <--------------------------------------------------------------------.
 * ) Check direction, check border limit    <-------------------------------. |
 * ) Replase last-bit into position which before head (...now it head...!)  | |
 * ) If exist Eat-bit, enlarge for length                                   | |
 * ) ...If no, do jump to ----------------------------------------------------'
 * ) random                                                                 |
 * ) if border limits is OK, direction to left/right/up/down ---------------'
 * ) ...If border is limited ->End
 * )
 */
	t_ret_code  rc = RC_FAILED;
	//static t_algo_step  algo_step;
	uint8_t      xx, yy;
	//uint32_t     rnd, rnd_tmp;
	//static char  snake_init = 0;
	static snake_node  player = {0};
	//snake_node* player = malloc(sizeof(snake_node));

	if ( p_io == NULL || p_snake == NULL ){
		rc=RC_PTR_FAIL;
		return rc;
	}

	rc = RC_OK;

	switch ( p_snake->algo_step ) {
		case S_INIT:
			/*p_snake->body = p_snake->ptr_field;
			p_snake->ptr_field     = &a_field[0][0];
			//p_snake->ptr_field_eat = &a_field_eat[0][0];
			p_snake->ptr_field_eat = &buf_out;
			p_snake->curr_x        = 0;
			p_snake->curr_y        = 0;

			// Clean field
			for ( xx=0; xx<8; xx++ ) {
				for ( yy=0; yy<16; yy++ ) { a_field_eat[xx][yy]=0; }
			}*/

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

			p_snake->algo_step = S_START_RANDOM;
		break;

		case S_START_RANDOM:
			/*
			//srand ( 255 ); // Initiate seek random number
			//srand ( p_io->x );
			p_snake->rnd = rnd_fill_field( p_io->rnd );
			// Fill field as random
			//for ( xx=0; xx<8; xx++ )
			//{
			//	for ( yy=0; yy<16; yy++ ) {
			//		rnd = (uint8_t)rand(); // get random number
			//		if (rnd&0x01) rnd&=0x69; else rnd&=0x96;
			//		p_snake->rnd = rnd;
			//		rnd_tmp = rnd;
			//		//a_field_eat [xx][yy] = ( rnd_tmp < 8 ) ? 1 : 0;
			//		a_field_eat_16b[yy] = rnd_tmp;
			//	}
			//}
			*/

			//we need to seed our rand() and generate our first random object
			//srand(time(NULL));
			srand ( 255 ); // Initiate seek random number
			snake_generate_new_apple();

			p_snake->algo_step = S_BEGIN;
		break;

		case S_BEGIN:
	        snake_move(&player);
	        snake_draw();

	        ticks++;    //update ticks
	        resting=0;  //slow the game down

	        //this formula is hacked together. This number is the one which determines how slow the game is. The
	        //higher the number the longer we wait. So higher gamespeed means a lower wait.
	        //rest_callback(100-gamespeed * 30, rest1);

			//char val;
			static char prev_x, prev_y;

			//drv_led_8x8_clear( 0, 0, 1 );
			objMap[prev_x][prev_y]=0;
			objMap[player.x][player.y]=1;
	        xx=0;
			for ( xx=0; xx<8; xx++ )
			{
				yy=0;
				while ( yy<16 )
				//if ( _TIMER_READY == delay_timer_leds8x8('?') )
				{
					//val=objMap[xx][yy];
					drv_led_8x8_show_byte( objMap[xx][yy], xx, yy );
					//drv_led_8x8_show_byte( a_field_eat_16b[yy], xx, yy);
					//drv_led_8x8_pixel_set ( player.x, player.y, 1 );
					yy++;
				}
			}
			prev_x = player.x;
			prev_y = player.y;


			p_snake->algo_step = S_BEGIN;
			//p_snake->algo_step = S_START_RANDOM; //S_BEGIN;
			//p_snake->algo_step = S_MOVE;
		break;

		case S_MOVE:
			p_snake->algo_step = S_EAT_YES;
		break;

		case S_EAT_YES:
			p_snake->algo_step = S_EAT_NO;
		break;

		case S_EAT_NO:
			p_snake->algo_step = S_RANDOM;
		break;

		case S_RANDOM:
			p_snake->algo_step = S_CHANGE_DIRECTION;
		break;

		case S_CHANGE_DIRECTION:
			p_snake->algo_step = S_BORDER_END;
		break;

		case S_BORDER_END:
			p_snake->algo_step = S_BORDER_END;
		break;

		default :
			p_snake->algo_step = S_BORDER_END;
		break;
	}

	return rc;
}
//******************************************************************************

