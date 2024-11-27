/*
 * fft.c
 *
 *  Created on: Nov 24, 2024
 *      Author: tilen
 */

#include "fft.h"

#define SAMPLING_RATE 1000.0f  // Sampling rate in Hz
#define TEST_FREQUENCY 50.0f   // Test sine wave frequency in Hz
#define C 3.0e8f
#define Fc 61.25e8f
#define LAMBDA C/Fc



float32_t sampled_data[2 * FFT_BUFFER_SIZE];




/**
 * @brief Remove mean value of complex input data (float version)
 *
 * Compute the mean of the complex input data and remove the
 * mean value.
 *
 * data must contain in total 2*len elements, len real values and len imaginary
 * values. The format of data is expected to be interleaved, i.e.:
 *   	data = {real0, imag0, real1, imag1, ..., realN, imagN}
 * where N=len-1.
 *
 * @param [in,out]	data	complex input/output data (float)
 * @param [in]		len		number of complex numbers
 */
static void mean_removal_float(float32_t *data, uint32_t len)
{
	float32_t sum_real = 0.0f, sum_imag = 0.0f;

    // Compute the sum of real and imaginary components
    for (uint32_t i = 0; i < len; i++)
    {
        sum_real += data[2 * i + 0];
        sum_imag += data[2 * i + 1];
    }

    // Compute the mean values
    const float32_t mean_real = sum_real / (float32_t)len;
    const float32_t mean_imag = sum_imag / (float32_t)len;

    // Subtract the mean from each data point
    for (uint32_t i = 0; i < len; i++)
    {
        data[2 * i + 0] -= mean_real;
        data[2 * i + 1] -= mean_imag;
    }
}



/**
 * @brief Apply Hann windowing function
 *
 * The Hann windowing function is defined as
 * 		w(n) = ( 1+cos(2*pi*n/(N-1)) )/2
 * where N is the length of the input data. For this function N=128.
 *
 * To speed up the functions, the values are precomputed. The values can be
 * computed using the following python code:
 *      from math import cos,pi
 *  	hanning = lambda n: 0.5-0.5*cos(2*pi*n/(128-1))
 *  	for n in range(128):
 *  		print(n, int(hanning(n)*2**31))
 *
 * The windowing function is applied to the complex input in data. The format
 * of data is expected to be interleaved, i.e.:
 *   	data = {real0, imag0, real1, imag1, ..., real127, imag127}
 *
 * @param [in,out]	complex input/output data
 */
static void hanning256_float(float32_t* data)
{
	static const float weights[128] = {
	    0.000000, 0.000152, 0.000607, 0.001365, 0.002427, 0.003790, 0.005454, 0.007419,
	    0.009683, 0.012244, 0.015102, 0.018253, 0.021698, 0.025433, 0.029455, 0.033764,
	    0.038355, 0.043227, 0.048376, 0.053800, 0.059494, 0.065456, 0.071681, 0.078166,
	    0.084908, 0.091902, 0.099143, 0.106628, 0.114351, 0.122309, 0.130496, 0.138907,
	    0.147537, 0.156382, 0.165435, 0.174691, 0.184144, 0.193790, 0.203621, 0.213632,
	    0.223818, 0.234170, 0.244684, 0.255354, 0.266171, 0.277131, 0.288226, 0.299449,
	    0.310794, 0.322255, 0.333823, 0.345492, 0.357254, 0.369104, 0.381032, 0.393033,
	    0.405099, 0.417223, 0.429397, 0.441614, 0.453866, 0.466146, 0.478447, 0.490761,
	    0.503080, 0.515398, 0.527706, 0.539997, 0.552264, 0.564500, 0.576696, 0.588845,
	    0.600941, 0.612976, 0.624941, 0.636831, 0.648638, 0.660355, 0.671974, 0.683489,
	    0.694893, 0.706178, 0.717338, 0.728366, 0.739256, 0.750000, 0.760592, 0.771027,
	    0.781296, 0.791395, 0.801317, 0.811056, 0.820607, 0.829962, 0.839118, 0.848067,
	    0.856805, 0.865327, 0.873626, 0.881699, 0.889540, 0.897145, 0.904508, 0.911626,
	    0.918495, 0.925109, 0.931464, 0.937558, 0.943387, 0.948946, 0.954233, 0.959243,
	    0.963976, 0.968426, 0.972592, 0.976471, 0.980061, 0.983359, 0.986364, 0.989074,
	    0.991487, 0.993601, 0.995416, 0.996930, 0.998142, 0.999052, 0.999659, 0.999962,
	};

	for (uint32_t i = 0; i < 128; i++)
	{
		const float32_t weight = weights[i];

		data[2 * i + 0] = weight * data[2 * i + 0];
		data[2 * i + 1] = weight * data[2 * i + 1];

		data[2 * (255 - i) + 0] = weight * data[2 * (255 - i) + 0];
		data[2 * (255 - i) + 1] = weight * data[2 * (255 - i) + 1];
	}
}


/**
 * @brief  Compute Complex FFT of size 128 for data.
 *
 * The format of data is expected to be interleaved, i.e.:
 *   	data = {real0, imag0, real1, imag1, ..., real127, imag127}
 *
 * @param [in,out]	data	input/output float32_t data
 */
static inline void fft256_float(float32_t* data)
{
	arm_cfft_f32(&arm_cfft_sR_f32_len256, data, 0, 1);
}

//============================================================================

/**
 * @brief  Compute Complex FFT of size 256 for data.
 *
 * The format of data is expected to be interleaved, i.e.:
 *   	data = {real0, imag0, real1, imag1, ..., real255, imag255}
 *
 * @param [in,out]	data	input/output q31_t data

static inline void fft256(q31_t data[512])
{
	arm_cfft_q31(&fftHandle_256, data, 0, 1);
}
*/


/**
 * @brief Compute Euclidean distance
 *
 * This function returns sqrt(a*a + b*b). This is the length of the
 * hypotenuse of a right-angled triangle with sides of length a and b,
 * or the distance of the point (x,y) from the origin.
 *
 * @param [in]   a    length a
 * @param [in]   b    length b
 * @retval       hypotenuse    sqrt(a*a + b*b)
 */
static float32_t hypot_f32(float32_t a, float32_t b)
{
    float32_t a2 = a * a;
    float32_t b2 = b * b;
    return sqrtf(a2 + b2);
}





void fft256_spectrum(float32_t* data)
{
	/* remove the mean from the input data */
	mean_removal_float(data, FFT_BUFFER_SIZE);

	/* apply window function */
	hanning256_float(data);

	/* compute FFT */
	fft256_float(data);

	/* compute spectrum */
	for(uint32_t i = 0; i < 256; i++)
	{
		const float32_t real = data[2*i+0], imag = data[2*i+1];
		data[i] = hypot_f32(real, imag);
	}


}


/* --- TEST FUNCTIONS     -------------------------------------------------------- */

static void generate_sine_wave(float32_t* data, uint32_t len) {

	for (uint32_t i = 0; i < len; i++) {
        float32_t t = i / SAMPLING_RATE; // Time index
        float32_t value = sinf(2.0f * M_PI * TEST_FREQUENCY * t);

        // Interleave real and imaginary parts
        data[2 * i + 0] = value; // Real part
        data[2 * i + 1] = 0.0f;  // Imaginary part
    }
}


void find_peak_frequency(const float32_t *spectrum, uint32_t fft_size, float32_t sampling_rate, float32_t *peak_freq, float32_t *peak_value, float32_t *target_velocity)
{
    float32_t max_value = 0.0f;
    uint32_t peak_index = 0;

    // Ensure fft_size is valid
	if (fft_size == 0 || spectrum == NULL || peak_freq == NULL || peak_value == NULL || target_velocity == NULL) {
		return; // Invalid input, handle error appropriately in production
	}

    for (uint32_t i = 0; i < fft_size; i++) {
        if (spectrum[i] > max_value) {
            max_value = spectrum[i];
            peak_index = i;
        }
    }

    *peak_value = max_value;
    *peak_freq = (sampling_rate * peak_index) / fft_size;
    //*target_velocity = (*peak_freq) *  3.6f *LAMBDA / 2.0f;

    // Handle velocity direction
	float32_t nyquist_freq = sampling_rate / 2.0f; // Nyquist frequency
	if (*peak_freq <= nyquist_freq) {
		// Approaching: positive velocity
		*target_velocity = (*peak_freq) * 3.6f * LAMBDA / 2.0f; // Convert to km/h if needed
	} else {
		// Departing: negative velocity
		float32_t adjusted_freq = sampling_rate - *peak_freq; // Fold frequency into [0, fs/2]
		*target_velocity = -adjusted_freq * 3.6f * LAMBDA / 2.0f; // Negative sign for departing
	}
}


void test_fft(float32_t* max_value, float32_t* peak_index,float32_t* target_velocity)
{

    generate_sine_wave(sampled_data, FFT_BUFFER_SIZE);

    // Perform FFT
    fft256_spectrum(sampled_data);

    find_peak_frequency(sampled_data, FFT_BUFFER_SIZE, SAMPLING_RATE, peak_index, max_value, target_velocity);

    // Output results (for debugging, use a breakpoint or UART to observe `sampled_data`)
}











