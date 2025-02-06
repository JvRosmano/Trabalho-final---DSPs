/*
 * Header file for: LCL_3Ph_ABCdq_PWMPLECS
 * Generated with : PLECS 4.9.2
 *                  PLECS RT Box 1 3.1.1
 * Generated on   : 6 Feb 2025 12:34:13
 */
#ifndef PLECS_HEADER_LCL_3Ph_ABCdq_PWMPLECS_h_
#define PLECS_HEADER_LCL_3Ph_ABCdq_PWMPLECS_h_

#include <stdbool.h>
#include <stdint.h>

/* Model floating point type */
typedef double LCL_3Ph_ABCdq_PWMPLECS_FloatType;

/* Model checksum */
extern const char * const LCL_3Ph_ABCdq_PWMPLECS_checksum;

/* Model error status */
extern const char * LCL_3Ph_ABCdq_PWMPLECS_errorStatus;


/* Model sample time */
extern const double LCL_3Ph_ABCdq_PWMPLECS_sampleTime;


#if defined(EXTERNAL_MODE) && EXTERNAL_MODE
/* External mode signals */
#define LCL_3Ph_ABCdq_PWMPLECS_NumExtModeSignals 36
extern const double * const LCL_3Ph_ABCdq_PWMPLECS_ExtModeSignals[];
/* Tunable parameters */
#define LCL_3Ph_ABCdq_PWMPLECS_NumTunableParameters 0
#endif /* defined(EXTERNAL_MODE) */


/* Entry point functions */
void LCL_3Ph_ABCdq_PWMPLECS_initialize(double time);
void LCL_3Ph_ABCdq_PWMPLECS_step(void);
void LCL_3Ph_ABCdq_PWMPLECS_terminate(void);

#endif /* PLECS_HEADER_LCL_3Ph_ABCdq_PWMPLECS_h_ */
