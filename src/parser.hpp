#pragma once

#include <cmath>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <atomic>
#include "defines.hpp"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "freertos/portmacro.h"
#include "driver/pcnt.h"

using namespace std;
////////////////////////ZASTAVENI, SPUSTENI MOTORU///////////////////////////
bool motor_pause_resume_delay(uint delay,bool state){
    int motor[4];
    for(int i=0; i<4; i++){
        motor[i]=motor_speed[i];
    }
    while(!state && delay==0){
        for(int i=0; i<4; i++){
            motor_speed[i]=0;
        }
        break;
    }
    while(!state && (delay>0)){
        for(int i=0; i<4; i++){
            motor_speed[i]=0;
        }
        vTaskDelay(delay / portTICK_PERIOD_MS);
        state = true;
    }
    while(state){
        for(int i=0; i<4; i++){
            motor_speed[i]=motor[i];
        }
        break;
    }
    return state;
}
//////////////////////////HOMING MOTORU/////////////////////
bool motor_homing(void){
    int DOJEZD[4] = {34,35,36,39};
    bool homing = false;
    int x=4;
    int count=0;
    while(!homing){

        for(int i = 0; i < 4; i++){
            if(gpio_get_level(static_cast<gpio_num_t>(DOJEZD[i]))){
                motor_speed[i] = 0;
                if(i!=x){
                    x=i;
                    count++;
                }
                else{
                    break;
                }
            }
        }
        if(count==4){
            homing = true;
            //break;
        }
    }
    return homing;
}
/////////////////////PARSOVANI HODNOT PRO MOTORY 0,1,2,3///////////////////////
int xyzw_parsing(int i,char g_code[]){
        int pos=1;
                for(int e=1; e<10; e++){
                    if(g_code[i+e]=='-' && e==1){
                        pos=pos*(-1);
                    }
                    if(g_code[i+e]==' ' && (pos<0)){
                        switch(e){
                            case 3:
                                pos=pos*g_code[i+(e-1)];
                            break;
                            case 4:
                                pos=pos*(g_code[i+(e-1)]+g_code[i+(e-2)]*10);
                            break;
                            case 5:
                                pos=pos*(g_code[i+(e-1)]+g_code[i+(e-2)]*10+g_code[i+(e-3)]*100);
                            break;
                            default:
                                printf("chyba pri zadavani\n");
                            break;
                        }
                    }
                    if(g_code[i+e]==' ' && (pos>0)){
                        switch(e){
                            case 2:
                                pos=pos*g_code[i+(e-1)];
                            break;
                            case 3:
                                pos=pos*(g_code[i+(e-1)]+g_code[i+(e-2)]*10);
                            break;
                            case 4:
                                pos=pos*(g_code[i+(e-1)]+g_code[i+(e-2)]*10+g_code[i+(e-3)]*100);
                            break;
                            default:
                                printf("chyba pri zadavani\n");
                            break;
                        }    
                    }
                }
       return pos;         
   }
////////////////////////TASK G CODE PARSERU/////////////////////
void g_code_parser(void* param){
        char g_code[64];
        bool init_done = false;
        bool read_done = false;

while(!init_done){
    cin.getline(g_code, 64);
        switch(g_code[0]){
            case '%':{
                init_done=true; 
                printf("init\n"); 
            }
            default:
            break;
        }
while(init_done && !read_done){ 
    cin.getline(g_code, 64);
        int count=0;
        int motor_pos[4];
        read_done=true;  
        int temp[8];
    for(int i=0; i<4; i++){
        temp[i]=h_limits[i];
        temp[i+1]=l_limits[i];
    }  
    for(int i=0; i<64; i++){
        switch(g_code[i]){
            case 'G':{
                int predcisli;
                for(int e=1; i<4; i++){
                    if(g_code[i+e]==' ' && e==3){
                        predcisli=g_code[i+(e-1)]*10+g_code[i+(e-2)];
                        break;
                    }
                    else if(g_code[i+e]==' ' && (e<3)){
                        printf("chyba pri zadavani\n");
                        predcisli=-1;
                        break;
                    }
                }
                switch(predcisli){
                    case -1:
                        printf("rezim nevybrán\n");
                        break;
                    case 0:
                        //00
                        break;
                    case 1:
                        //01
                        break;
                    case 2:
                        //02
                        break;
                    default:
                        break;
                }
            break;
            }
            case 'X':{
                int pos1=xyzw_parsing(i, g_code);
                if(pos1<0){
                    pcnt_set_event_value(PCNT_UNIT_0,PCNT_EVT_L_LIM,pos1);
                }
                else if(pos1>0){
                    pcnt_set_event_value(PCNT_UNIT_2,PCNT_EVT_H_LIM,pos1);
                }
                else{
                    count++;
                }
                motor_pos[1]=pos1;
            break;
            }
            case 'Y':{
                int pos2=xyzw_parsing(i, g_code);
                if(pos2<0){
                    pcnt_set_event_value(PCNT_UNIT_2,PCNT_EVT_L_LIM,pos2);
                }
                else if(pos2>0){
                    pcnt_set_event_value(PCNT_UNIT_2,PCNT_EVT_H_LIM,pos2);
                }
                else{
                    count++;
                }
                motor_pos[2]=pos2;
            break;
            }
            case 'Z':{
                int pos3=xyzw_parsing(i, g_code);
                if(pos3<0){
                    pcnt_set_event_value(PCNT_UNIT_3,PCNT_EVT_L_LIM,pos3);
                }
                else if(pos3>0){
                    pcnt_set_event_value(PCNT_UNIT_3,PCNT_EVT_H_LIM,pos3);
                }
                else{
                    count++;
                }
                motor_pos[3]=pos3;
            break;
            }
            case 'W':{
                int pos0=xyzw_parsing(i, g_code);
                if(pos0<0){
                    pcnt_set_event_value(PCNT_UNIT_0,PCNT_EVT_L_LIM,pos0);
                }
                else if(pos0>0){
                    pcnt_set_event_value(PCNT_UNIT_0,PCNT_EVT_H_LIM,pos0);
                }
                else{
                    count++;
                }
                motor_pos[0]=pos0;
            break;    
            }
            case 'F':{
                float speed=143.216;
                for(int e=1; i<6; i++){
                    if(g_code[i+e]==' '){
                        switch(e){
                            case 2:
                                speed=speed*(g_code[i+(e-1)]);
                                break;
                            case 3:
                                speed=speed*(g_code[i+(e-1)]+g_code[i+(e-2)]*10);
                                break;
                            case 4:
                                speed=speed*(g_code[i+(e-1)]+g_code[i+(e-2)]*10+g_code[i+(e-3)]*100);
                                break;
                            case 5:
                                speed=speed*(g_code[i+(e-1)]+g_code[i+(e-2)]*10+g_code[i+(e-3)]*100+g_code[i+(e-4)]*1000);
                                break;  
                            default:
                                printf("chyba pri zadavani\n");
                            break;      
                        }
                    }    
                }
                for(int i=0; i<4; i++){
                    if(motor_pos[i]<0){
                        motor_speed[i]=round(speed)*(-1);
                    }
                    else if(motor_pos[i]>0){
                        motor_speed[i]=round(speed);
                    }
                    else{
                        motor_speed[i]=0;
                    }
                }
            break;
            }
            case '%':{
                init_done=false;
                printf("konec g_codu\n");
            }
            default:
                break;
            }
        }
        while(read_done){
            if(count!=4){
                for(int i=0; i<4; i++){
                    if(temp[i]<h_limits[i] || temp[i+1]<l_limits[i]){
                        count++;
                    }
                }
            }
            else if(count==4){
                    read_done=false;
                    printf("motory nastaveny\n");
                    break;
                }
            }
        }
    }
}
   /*vector<int> reading_gcode_stream(void){
        std::ifstream gcode_file ("gcode.txt");
        //gcode_file.open("gcode.txt");
        //std::string stream;
        std::vector<int> stream{};
        if(gcode_file.is_open()){
            while(gcode_file){
                stream.push_back(gcode_file.get());
                //std::getline(file,stream);
                //read[i]=int(strtol(stream, NULL, 10));
            }
            //gcode_file.close("gcode.txt");
        }
        return stream;
   }*/
   

