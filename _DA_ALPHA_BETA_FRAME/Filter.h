/*
 * Filter.h
 *
 *  Created on: Jan 28, 2024
 *      Author: CHUONG
 */

#ifndef FILTER_H_
#define FILTER_H_

#include <math.h>
#define PIx2 6.28318530718

typedef struct{
    float y[3],x[3];
}SECOND_ORDER_FILTER;

#pragma FUNC_ALWAYS_INLINE(Compute_2nd_Filter)
static inline void Compute_2nd_Filter(volatile SECOND_ORDER_FILTER *Filter, float x_k,const float *FILTER_COEFF) {

    Filter->x[0] = x_k;         //x(k)
    // Compute y(k) = B0*x(k) + B1*x(k-1) + B2*x(k-2) - A1*y(k-1) - A2*y(k-2)
    Filter->y[0] =  FILTER_COEFF[0] * Filter->x[0]        //B0*x(k)
                 +  FILTER_COEFF[1] * Filter->x[1]        //B1*x(k-1)
                 +  FILTER_COEFF[2] * Filter->x[2]        //B2*x(k-2)
                 -  FILTER_COEFF[3] * Filter->y[1]        //A1*y(k-1)
                 -  FILTER_COEFF[4] * Filter->y[2];       //A2*y(k-2)

    // Store values for next iteration
    Filter->x[2] = Filter->x[1];
    Filter->x[1] = Filter->x[0];
    Filter->y[2] = Filter->y[1];
    Filter->y[1] = Filter->y[0];
}

typedef struct{
    float y[2],x[2];
}FIRST_ORDER_FILTER;

#pragma FUNC_ALWAYS_INLINE(Compute_1st_Filter)
static inline void Compute_1st_Filter(volatile FIRST_ORDER_FILTER *Filter, float x_k,const float *FILTER_COEFF) {

    Filter->x[0] = x_k;         //x(k)
    // Compute y(k) =  B0*x(k) + B1*x(k-1) - A1*y(k-1)
    Filter->y[0] =  FILTER_COEFF[0] * Filter->x[0]        //B0*x(k)
                 +  FILTER_COEFF[1] * Filter->x[1]        //B1*x(k-1)
                 -  FILTER_COEFF[2] * Filter->y[1];       //A1*y(k-1)

    // Store values for next iteration
    Filter->x[1] = Filter->x[0];
    Filter->y[1] = Filter->y[0];
}

#pragma FUNC_ALWAYS_INLINE(Integrator_mod_2_pi)
static inline void Integrator_mod_2_pi(volatile FIRST_ORDER_FILTER *Filter, float x_k,const float *FILTER_COEFF) {

    Filter->x[0] = x_k;         //x(k)
    // Compute y(k)
    Filter->y[0] =  FILTER_COEFF[0] * Filter->x[0]        //B0*x(k)
                 +  FILTER_COEFF[1] * Filter->x[1]        //B1*x(k-1)
                 -  FILTER_COEFF[2] * Filter->y[1];       //A1*y(k-1)
    Filter->y[0] = fmod(Filter->y[0],PIx2);

    // Store values for next iteration
    Filter->x[1] = Filter->x[0];
    Filter->y[1] = Filter->y[0];
}


typedef struct{
    float num[3];   //!< Internal: Numerator    b0 b1 b2
    float den[3];   //!< Internal: Denominator  a0 a1 a2
}DELTA_SECOND_COEFF;

typedef struct{
    float W_k;                      //!< Internal: Output of sum
    float W_k1;                     //!< Internal: Delay 1st of Sum
    float W_k2;                     //!< Internal: Delay 2nd of Sum
    float Delta;                    //!< Internal: Delta number

    DELTA_SECOND_COEFF d_coeff;     //!< Internal: Delta coefficient

    float X_k;                      //!< Input
    float Y_k;                      //!< Output
}DELTA_SECOND_ORDER;

static inline void Init_Delta_Filter(volatile DELTA_SECOND_ORDER *d, const float q_coeff[],float Delta){
    d->Delta = Delta;
    d->d_coeff.num[0] = 65.938098122403020;
    d->d_coeff.num[1] = 2.290614768661490E2;
    d->d_coeff.num[2] = 63.038533685481525;

    d->d_coeff.den[0] = 1;
    d->d_coeff.den[1] = 0.082696172682589;
    d->d_coeff.den[2] = 1.009590711579108;

    d->W_k  = 0;
    d->W_k1 = 0;
    d->W_k2 = 0;
    d->X_k  = 0;
    d->Y_k  = 0;
}

//! \brief Implement Delta Operator for 2nd order filter
//!
//! \details
//!  Delta Operator Implementation (DFII)
//!                             W_k
//!  x[k] ------>(++)-------->()----b0--->(++)------> y[k]
//!               ^           |             ^
//!               |        |Y^(-1)|         |
//!               |           |             |
//!               |           v (W_k1)      |
//!              (--)<---a1---()----b1--->(++)
//!               ^           |             ^
//!               |        |Y^(-1)|         |
//!               |           |             |
//!               |           v (W_k2)      |
//!              (  )<---a2---()----b2--->(  )
//!
//!  Where Y^(-1) = Delta*Z^(-1)/(1-Z^(-1)       (*)
//!
//!  From (*) =>
//!  W_k2 = Delta*W_k1 + W_k2;
//!  W_k1 = Delta*W_k + W_k1;
//!
//!
//! \param *d The DELTA_SECOND_ORDER structure pointer
//! \return None
//---------------------------------------------------//
// WARNING: MUST UPDATE X_k before call this function
//---------------------------------------------------//
#pragma FUNC_ALWAYS_INLINE(Compute_delta_2nd_filter)
static inline void Compute_delta_2nd_filter(volatile DELTA_SECOND_ORDER* d) {
    d->W_k = d->X_k - d->d_coeff.den[1] * d->W_k1 - d->d_coeff.den[2] * d->W_k2;
    d->Y_k = d->d_coeff.num[0] * d->W_k + d->d_coeff.num[1] * d->W_k1 + d->d_coeff.num[2] * d->W_k2;

    // Store Value for Next Iteration
    d->W_k2 = d->Delta * d->W_k1 + d->W_k2;
    d->W_k1 = d->Delta * d->W_k + d->W_k1;
}









#endif /* FILTER_H_ */
