
#define Filters_H_




#include "stdio.h"
#include "stdint.h" //dont know if its needed


typedef struct {

float y1;
float y2;
float y3;
float y4;
float alpha;

}LOW_FilterTypeDef;


typedef struct {
    float y1;
    float y2;
    float y3;
    float y4;
    float beta;
    float previous_input;
    float previous_y1;
    float previous_y2;
    float previous_y3;
}HIGH_FilterTypeDef;



void LOW_PASS_1_ORDER_Init(LOW_FilterTypeDef *f,float alpha_value); //Init low pass filter 1st order!
float LOW_PASS_1_ORDER(LOW_FilterTypeDef *f,uint16_t input); //basic function for an EMA 1st order low pass filter

void HIGH_PASS_1_ORDER_Init(HIGH_FilterTypeDef *f,float beta_value); //Init High pass filter 1st order!
float HIGH_PASS_1_ORDER(LOW_FilterTypeDef *f,uint16_t input); //basic function for an EMA 1st order high pass filter



void LOW_PASS_4_ORDER_Init(LOW_FilterTypeDef *f,float alpha_value); // Init low pass filter 4th order!
float LOW_PASS_4_ORDER(LOW_FilterTypeDef *f,uint16_t input); //basic function for an EMA 4th order low pass filter

void HIGH_PASS_4_ORDER_Init(HIGH_FilterTypeDef *f,float beta_value); // Init high pass filter 4th order!
float HIGH_PASS_4_ORDER(HIGH_FilterTypeDef *f,uint16_t input); //basic function for an EMA 4th order low pass filter





float LOW_PASS_IR_AC(float alpha,uint16_t input); //might be obsolete
float LOW_PASS_R_AC(float alpha,uint16_t input); //might be obsolete