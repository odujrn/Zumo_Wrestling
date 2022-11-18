#include <project.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "LSM303D.h"
#include "IR.h"
#include "Beep.h"
#include "mqtt_sender.h"
#include <time.h>
#include <sys/time.h>
#include "serial1.h"
#include <unistd.h>
#include <stdlib.h>
#define BLACK 1
#define WHITE 0
#include <stdbool.h>

#if 1
void init_sensors(void){
    Ultra_Start();
    reflectance_start();
    IR_Start();
    motor_start();
    reflectance_set_threshold(11000, 11000, 11000, 11000, 11000, 11000);
    IR_flush();
}
void stop_motor(void){
    motor_forward(0,0);
    motor_stop();
}
void tankturn(void){
    SetMotors(1, 0, 100, 100, 400);
}
void get_start_time(void){
    TickType_t start;
    start = xTaskGetTickCount();
    printf("Start time: %d\n",(int)start);
    print_mqtt("Zumo101/start", "%d",start);
}
void get_end_time(void){
    TickType_t end;
    end = xTaskGetTickCount();
    print_mqtt("Zumo101/stop", "%d", end);
}
void get_obstacle_time(void){
    TickType_t obstacle;
    obstacle = xTaskGetTickCount();
    print_mqtt("Zumo101/obstacle", "%d", obstacle);
}
void zumo_wait_line(void){
    motor_forward(0,0);
    print_mqtt("Zumo101/ready", "Zumo");
	IR_wait();
    get_start_time();
}
void line_detect_black(struct sensors_ dig){
    while((dig.L3 == 1) && (dig.R3 == 1)){
        reflectance_digital(&dig);
        motor_forward(100,50);
    }
}

void zmain(void){
    bool stop = false;
    TickType_t start;
    TickType_t end;
    struct sensors_ dig;
    init_sensors();
    reflectance_digital(&dig);
    while(SW1_Read() == 1){
	    vTaskDelay(100);
    }
    while((dig.L3 == 0) && (dig.R3 == 0)){
        reflectance_digital(&dig);
        motor_forward(100,0);
    }
    zumo_wait_line();
    start = xTaskGetTickCount();
    line_detect_black(dig);
    while (!stop){
        int d = Ultra_GetDistance();
        reflectance_digital(&dig);
        motor_forward(100,0);
        if (d < 8){
            tankturn();
            get_obstacle_time();    
        }
        if ((dig.R3 == 1) || (dig.L3 == 1)){
            tankturn();
        }
        if (SW1_Read() == 0){
            stop_motor();
            get_end_time();
            end = xTaskGetTickCount();
            TickType_t time = end - start; 
            print_mqtt("Zumo101/time", "%d", time);
            stop = true;
        } 
    }
}
#endif
