
void  drv_usr_init_scheduler_end_timers (void);
void  drv_usr_init_rtc (void);

t_timer_stat delay_timer ( timer_name name, uint8_t cfg );

/*
t_timer_stat delay_timer_leds8x8 (uint8_t cfg);
t_timer_stat delay_timer_keys (uint8_t cfg);
t_timer_stat delay_timer_leds5x8 (uint8_t cfg);
t_timer_stat delay_timer_rnd (uint8_t cfg);
t_timer_stat delay_timer_mouse (uint8_t cfg);
*/
