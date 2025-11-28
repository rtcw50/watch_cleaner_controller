/*
 * File: wcc_motor_controller.cpp
 * Author: John
 * Date: 2025-11-23
 * Description:
 *   Motor control and logic 
 *   Motor ramping is done in chunks via a timer callback to avoid long
 *      duration ramping in a for loop. Long duration function make the UI 
 *      unresponsive b/c the timer handler is not called often enough.
 */
#include <Arduino.h>
#include <lvgl.h>
#include "wcc_common.h"

/* Defines */

/* Externs */
extern lv_subject_t max_rpm_int_subject;
extern lv_subject_t spin_up_rate_int_subject;
extern enum class OperatingState g_operating_state;

/* Statics */
static uint8_t in1;
static uint8_t in2;
static lv_timer_t * ramp_up_timer;
static lv_timer_t * ramp_down_timer;

/* Local typedefs */
typedef struct ramp_user_data {
    int32_t rpm_tracker;
    wcc_cb_t on_done;
} ramp_user_data;

static void invert_in_pins()
{
    uint8_t tmp;
    tmp = in1;
    in1 = in2;
    in2 = tmp;
}

static void ramp_up_internal_cb(lv_timer_t * motor_timer)
{
    ramp_user_data * tracker_data = (ramp_user_data *)lv_timer_get_user_data(motor_timer);
    extern pwm_info pwm_values;
    //Serial.println("ramp_up_internal_cb");

    // Invoking the reverse function ramps down and then up (in opposite direction)
    // So it is possible to have two ramp timers going concurrently.
    // But it makes no sense for one timer cb to ramp down and the other to ramp up
    // So do nothing if until the other timer is finished.
    if (g_operating_state != OperatingState::stopped) {
        //Serial.println("up returned");
        return;
    }

    Serial.printf("ramp_up_int: AW (%d) on %d\n", tracker_data->rpm_tracker, (in2-2));
    analogWrite(in2, tracker_data->rpm_tracker);

    tracker_data->rpm_tracker += pwm_values.pwm_increment; 

    if (tracker_data->rpm_tracker > pwm_values.pwm_rpm) {
        // done ramping up 
        lv_timer_delete(motor_timer); // Alias for ramp_up_timer
        ramp_up_timer = NULL;  // Nullify reference to motor timer
        if (tracker_data->on_done != NULL) {
            tracker_data->on_done();
        }
        g_operating_state = OperatingState::running;

        Serial.println("ramp_up_complete");
    digitalWrite(D6, HIGH);
    }
}

static void ramp_down_internal_cb(lv_timer_t * motor_timer)
{
    ramp_user_data * tracker_data = (ramp_user_data *)lv_timer_get_user_data(motor_timer);
    extern pwm_info pwm_values;

    // Invoking the reverse function ramps down and then up (in opposite direction)
    // So it is possible to have two ramp timers going concurrently.
    // But it makes no sense for one timer cb to ramp down and the other to ramp up
    // So do nothing if until the other timer is finished.
    if (g_operating_state != OperatingState::running) {
        //Serial.println("down returned");
        return;
    }

    Serial.printf("ramp_down_int: AW(%d) on %d\n", tracker_data->rpm_tracker, (in2-2));
    analogWrite(in2, tracker_data->rpm_tracker);

    tracker_data->rpm_tracker -= pwm_values.pwm_increment; 

    if (tracker_data->rpm_tracker < 0) {
        // done ramping down
        lv_timer_delete(motor_timer); // Alias for ramp_down_timer
        ramp_down_timer = NULL; // Nullify reference to motor timer
        if (tracker_data->on_done != NULL) {
            tracker_data->on_done();
        }
        g_operating_state = OperatingState::stopped;
        Serial.println("ramp_down_complete");
    digitalWrite(D6, LOW);
    }
}

static void ramp_motor_up(lv_timer_cb_t ramp_cb, wcc_cb_t action_on_done)
{
    extern pwm_info pwm_values;
    // Starting value for the tracking variable 
    static int32_t rpm_tracker;
    static ramp_user_data tracker_data;
    tracker_data.rpm_tracker = 0;
    tracker_data.on_done = action_on_done;
    
    // Write one input to static level 
    #if 0
    for (int32_t i = 0; i<10; i++) {
        digitalWrite(D4, LOW);
        delay(10);
        digitalWrite(D4,HIGH);
        delay(10);
    }
    #endif
    // digitalWrite(in1,LOW);

    if (ramp_up_timer == NULL) {
        //Serial.println("timer created");
        ramp_up_timer = lv_timer_create(ramp_cb, RAMP_UPDATE_MS, &tracker_data);
    }
}

static void ramp_motor_down(lv_timer_cb_t ramp_cb, wcc_cb_t action_on_done)
{
    extern pwm_info pwm_values;
    // Starting value for the tracking variable 
    static ramp_user_data tracker_data; 
    tracker_data.rpm_tracker = pwm_values.pwm_rpm;
    tracker_data.on_done = action_on_done;
    
    // Write one input to static level 
    digitalWrite(in1, LOW);

    if (ramp_down_timer == NULL) {
        //Serial.println("timer created");
        ramp_down_timer = lv_timer_create(ramp_cb, RAMP_UPDATE_MS, &tracker_data);
    }
}

void wcc_drv8871_ramp_up()
{
    Serial.println("ramp up");
    ramp_motor_up(ramp_up_internal_cb, NULL);
}

void wcc_drv8871_ramp_down(boolean invert_driver_pins)
{
    Serial.println("ramp down");
    if (invert_driver_pins) {
        ramp_motor_down(ramp_down_internal_cb, invert_in_pins);
    }
    else {
        ramp_motor_down(ramp_down_internal_cb, NULL);
    }
}

void wcc_drv8871_reverse()
{
    wcc_drv8871_ramp_down(true /*invert inputs after ramp down*/);
    wcc_drv8871_ramp_up();
}

void wcc_drv8871_init_in_pins(uint8_t p1, uint8_t p2)
{
    in1 = p1;
    in2 = p2;
    pinMode(in1,OUTPUT);
    pinMode(in2,OUTPUT);
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
    
    pinMode(D6,OUTPUT);
    digitalWrite(D6, LOW);
}
