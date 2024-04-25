/*
 * PI.h
 *
 *  Created on: Apr 17, 2024
 *      Author: Chuong
 */

#ifndef PI_H_
#define PI_H_

#define PI_DEFAULT {{0,0},0,0,0,0,0,0}

typedef struct{
    float Err[2]; // Error k and k-1
    float Up;     // Output proportional
    float Ui;     // Output integral
    float Kp;
    float Ki;
    float Out;    // Output PI controller
    float Ts;
}PI_VAL;

// Must update Err[0] before call this function
#pragma FUNC_ALWAYS_INLINE(Compute_PI)
inline Compute_PI(volatile PI_VAL *v){
    v->Up = v->Kp*v->Err[0];
    v->Ui = v->Ki*v->Ts*(v->Err[0]+v->Err[1])/2 + v->Ui;
    v->Out = v->Up+v->Ui;
    v->Err[1] = v->Err[0];
}


#endif /* PI_H_ */
