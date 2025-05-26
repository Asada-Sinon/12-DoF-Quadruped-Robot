#include "fsm.h"
#include "dog.h"
#include "timer.h"
#include "gait.h"
#include "stdio.h"
#include "motor.h"
GaitState passive_state;    
/* ×èÄá×´Ì¬µÄ´¦Àíº¯Êı */
static void passive_enter(void) {
    // printf("½øÈë×èÄá×´Ì¬\n");
    
}

static void passive_run(void) {
    for(int i = 0; i < 4; i++){
        leg_set_motor_kp_kd(i, 0, 8, 0, 8, 0, 8);
    }
}

static void passive_exit(void) {
    // printf("ÍË³ö×èÄá×´Ì¬\n");

}

/* ×èÄá×´Ì¬µÄ×ª»»¼ì²é */
static bool passive_check_transition(StateName next) {
    if (next == STATE_STAND) // ×èÄá×´Ì¬Ö»ÄÜÇĞ»»µ½Õ¾Á¢×´Ì¬
        return true;
    return false;
}

/* ×¢²áÕ¾Á¢×´Ì¬ */
void register_passive_state(void) {
    fsm_register_state(STATE_PASSIVE,
                     passive_enter,
                     passive_run,
                     passive_exit,
                     passive_check_transition);
}
