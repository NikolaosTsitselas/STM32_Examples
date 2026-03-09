#include "Filters.h"
#include "stdio.h"
#include "math.h"

//Initialization of 1st order EMA Low pass filter;

void LOW_PASS_1_ORDER_Init(LOW_FilterTypeDef *f,float alpha_value){
f->y1=0.0f;
f->alpha=alpha_value;

}


//Our 1st order EMA Low pass filter
float LOW_PASS_1_ORDER(LOW_FilterTypeDef *f,uint16_t input){
    float a=f->alpha;
    float in=(float)input;
    f->y1=a*in + (1-a)*f->y1;

    return f->y1;
}

//Initialization of 1st order EMA High pass filter;
void HIGH_PASS_1_ORDER_Init(HIGH_FilterTypeDef *f,float beta_value){
    f->y1=0.0f;
    f->previous_input=0.0f;
    f->beta=beta_value;
}

float HIGH_PASS_1_ORDER(HIGH_FilterTypeDef *f,uint16_t input){
    float in=(float)input;
    float b=f->beta;

    f->y1=0.5f * (2-b) * (in-(f->previous_input)) + (1.0f-b)*f->y1;
    f->previous_input=in;

    return f->y1;
}




//Initialization of 4th order EMA Low pass filter;

void LOW_PASS_4_ORDER_Init(LOW_FilterTypeDef *f,float alpha_value){

    f->y1=0.0f;
    f->y2=0.0f;
    f->y3=0.0f;
    f->y4=0.0f;
    f->alpha=alpha_value;

}

//Our 4th order EMA low pass filter;

float LOW_PASS_4_ORDER(LOW_FilterTypeDef *f,uint16_t input){
    float a=f->alpha;
    float in=(float)input;

    f->y1=a*in + (1.0f-a)*f->y1;
    f->y2=a*f->y1 + (1.0f-a)*f->y2;
    f->y3=a*f->y2 + (1.0f-a)*f->y3;
    f->y4=a*f->y3 + (1.0f-a)*f->y4;

    return f->y4;

}

//Initialization of 4th order EMA High pass filter
void HIGH_PASS_4_ORDER_Init(HIGH_FilterTypeDef *f,float beta_value){

    f->y1=0.0f;
    f->y2=0.0f;
    f->y3=0.0f;
    f->y4=0.0f;
    f->beta=beta_value;
    f->previous_input=0;
    f->previous_y1=0.0;
     f->previous_y2=0.0;
     f->previous_y3=0.0;

}

//Our 4th order EMA High Pass filter
float HIGH_PASS_4_ORDER(HIGH_FilterTypeDef* f,uint16_t input){
    float b=f->beta;
    float in=(float)input;

    f->y1=0.5f*(2.0f-b)*(in-(f->previous_input)) + (1.0f-b)*f->y1;
    f->previous_input=in;
    f->y2=0.5f*(2.0f-b)*(f->y1-(f->previous_y1)) + (1.0f-b)*f->y2;
    f->previous_y1=f->y1;
    f->y3=0.5f*(2.0f-b)*(f->y2-(f->previous_y2)) + (1.0f-b)*f->y3;
    f->previous_y2=f->y2;
    f->y4=0.5f*(2.0f-b)*(f->y3-(f->previous_y3)) + (1.0f-b)*f->y4;
    f->previous_y3=f->y3;

    return f->y4;
}





//might be obsolete.
float LOW_PASS_IR_AC(float alpha,float beta,uint16_t input){
    static float y1=0.0;
    static float y2=0.0;
    static float y3=0.0;
    static float y4=0.0;
    static float y5=0.0;
    static float y6=0.0;
    static float y7=0.0;
    static float y8=0.0;
    static float y9=0.0;
    static float y10=0.0;
    static float final_signal_ir;
    

y1=alpha*input + (1.0f-alpha)*y1;
y2=alpha*y1 + (1.0f-alpha)*y2;
y3=alpha*y2 + (1.0f-alpha)*y3;
y4=alpha*y3 + (1.0f-alpha)*y4;

y5=input-y4;
y6=y5*y5; //eisodos gia to deutero xamiloperato filtro

y7=beta*y6 + (1.0f-beta)*y7;
y8=beta*y7 + (1.0f-beta)*y8;
y9=beta*y8 + (1.0f-beta)*y9;
y10=beta*y9 + (1.0f-beta)*y10;

final_signal_ir=sqrt(y10);

return final_signal_ir;

}

//might be obsolete

float LOW_PASS_R_AC(float alpha,float beta,uint16_t input){
    static float y1=0.0;
    static float y2=0.0;
    static float y3=0.0;
    static float y4=0.0;
    static float y5=0.0;
    static float y6=0.0;
    static float y7=0.0;
    static float y8=0.0;
    static float y9=0.0;
    static float y10=0.0;
    static float final_signal_r;
    

y1=alpha*input + (1.0f-alpha)*y1;
y2=alpha*y1 + (1.0f-alpha)*y2;
y3=alpha*y2 + (1.0f-alpha)*y3;
y4=alpha*y3 + (1.0f-alpha)*y4;

y5=input-y4;
y6=y5*y5; //eisodos gia to deutero xamiloperato filtro

y7=beta*y6 + (1.0f-beta)*y7;
y8=beta*y7 + (1.0f-beta)*y8;
y9=beta*y8 + (1.0f-beta)*y9;
y10=beta*y9 + (1.0f-beta)*y10;

final_signal_r=sqrt(y10);

return final_signal_r;

}




