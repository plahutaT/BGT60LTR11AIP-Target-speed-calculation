/*
 * fft.h
 *
 *  Created on: Nov 24, 2024
 *      Author: tilen
 */

#ifndef INC_FFT_H_
#define INC_FFT_H_


#include "stm32h750xx.h"
#define ARM_MATH_CM7
#include "arm_math.h"
#include "stdint.h"
#include "arm_const_structs.h"

#define FFT_BUFFER_SIZE 256

void fft256_spectrum(float32_t* data);
void test_fft(float32_t* max_value, float32_t* peak_index,float32_t* target_velocity);
void find_peak_frequency(const float32_t *spectrum, uint32_t fft_size, float32_t sampling_rate, float32_t *peak_freq, float32_t *peak_value, float32_t *target_velocity);






#endif /* INC_FFT_H_ */
