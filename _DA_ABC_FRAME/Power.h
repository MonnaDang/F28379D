/*
 * Power.h
 *
 *  Created on: May 12, 2024
 *      Author: CHUONG
 */


#ifndef POWER_MEAS_SINE_ANALYZER_H
#define POWER_MEAS_SINE_ANALYZER_H

#ifdef __cplusplus

extern "C"
{
#endif

//
// Included Files
//
#include <stdint.h>
#include <math.h>

//! \brief Defines the POWER_MEAS_3PHASE_ANALYZER structure
//!
//!
typedef volatile struct {
    float32_t va;           //!< Input: Voltage Sine Signal Va
    float32_t vb;           //!< Input: Voltage Sine Signal Vb
    float32_t vc;           //!< Input: Voltage Sine Signal Vc
    float32_t ia;           //!< Input Current Signal Ia
    float32_t ib;           //!< Input Current Signal Ib
    float32_t ic;           //!< Input Current Signal Ic

    float32_t vaRms;        //!< Output: RMS Value A
    float32_t vbRms;        //!< Output: RMS Value B
    float32_t vcRms;        //!< Output: RMS Value C

    float32_t iaRms;        //!< Output: RMS Value of current A
    float32_t ibRms;        //!< Output: RMS Value of current B
    float32_t icRms;        //!< Output: RMS Value of current C

    float32_t P_m;          //!< Output: Active Power
    float32_t Q_m;          //!< Output: Reactive Power
    float32_t S_m;          //!< Output: Apparent Power

    float32_t val;          //!< Internal : V alpha
    float32_t vbe;          //!< Internal : V beta
    float32_t ial;          //!< Internal : I alpha
    float32_t ibe;          //!< Internal : I beta

    float32_t vaSqrSum;     //!< Internal : running sum for vacc square calculation over one sine cycle
    float32_t iaSqrSum;     //!< Internal : running sum for Iacc_rms calculation over one sine cycle
    float32_t vbSqrSum;     //!< Internal : running sum for vbcc square calculation over one sine cycle
    float32_t ibSqrSum;     //!< Internal : running sum for Ibcc_rms calculation over one sine cycle
    float32_t vcSqrSum;     //!< Internal : running sum for vccc square calculation over one sine cycle
    float32_t icSqrSum;     //!< Internal : running sum for Iccc_rms calculation over one sine cycle
    int32_t  nSamples;      //!< Internal: No of samples in one cycle of the sine wave
} POWER_MEAS_3PHASE_ANALYZER;

static inline void POWER_MEAS_3PHASE_ANALYZER_reset(POWER_MEAS_3PHASE_ANALYZER *v){

    v->iaRms = 0;
    v->ibRms = 0;
    v->icRms = 0;

    v->vaRms = 0;
    v->vbRms = 0;
    v->vcRms = 0;

    v->iaSqrSum = 0;
    v->ibSqrSum = 0;
    v->icSqrSum = 0;

    v->vaSqrSum = 0;
    v->vbSqrSum = 0;
    v->vcSqrSum = 0;

    v->nSamples = 0;
}

// Function run at 10kHz isr Frequency
static inline void POWER_MEAS_3PHASE_ANALYZER_run(POWER_MEAS_3PHASE_ANALYZER* v){

    // RMS calculation
    v->nSamples++;
    v->vaSqrSum = v->vaSqrSum + v->va*v->va;
    v->vbSqrSum = v->vbSqrSum + v->vb*v->vb;
    v->vcSqrSum = v->vcSqrSum + v->vc*v->vc;

    v->iaSqrSum = v->iaSqrSum + v->ia*v->ia;
    v->ibSqrSum = v->ibSqrSum + v->ib*v->ib;
    v->icSqrSum = v->icSqrSum + v->ic*v->ic;

    if(v->nSamples > 200){
        v->vaRms = sqrt(v->vaSqrSum)*0.070710678118655;  // sqrt((V1^2+V2^2+...+Vn^2)/n); n = 200
        v->vbRms = sqrt(v->vbSqrSum)*0.070710678118655;
        v->vcRms = sqrt(v->vcSqrSum)*0.070710678118655;

        v->iaRms = sqrt(v->iaSqrSum)*0.070710678118655;
        v->ibRms = sqrt(v->ibSqrSum)*0.070710678118655;
        v->icRms = sqrt(v->icSqrSum)*0.070710678118655;

        v->iaSqrSum = 0;
        v->ibSqrSum = 0;
        v->icSqrSum = 0;

        v->vaSqrSum = 0;
        v->vbSqrSum = 0;
        v->vcSqrSum = 0;

        v->nSamples = 0;
    }
    // RMS calculation

    // Power calculation
    v->val = (2*v->va - v->vb - v->vc)*0.333333333333333;
    v->vbe = (v->vb - v->vc)*0.577350269189626;

    v->ial = (2*v->ia - v->ib - v->ic)*0.333333333333333;
    v->ibe = (v->ib - v->ic)*0.577350269189626;

    v->P_m = 1.5*(v->val*v->ial + v->vbe*v->ibe);
    v->Q_m = 1.5*(v->vbe*v->ial - v->val*v->ibe);

//    v->S_m = sqrt(v->P_m*v->P_m+v->Q_m*v->Q_m);
    // Power calculation
}


#endif // POWER_MEAS_3PHASE_ANALYZER

//
// End of File
//

