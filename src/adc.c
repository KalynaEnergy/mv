#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

#include <math.h>
#include "arm_math.h"
#include "arm_const_structs.h"
void arm_hft95_f32(
        float32_t * pDst,
        uint32_t blockSize);

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/timing/timing.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>


#define SQR(x) ((x)*(x))


#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};


#define BLOCK_SIZE 4096
#define SAMPLE_RATE 16340.4f // constant is sampling rate, determined by experimental calibration
#define VOLTAGE_DIVIDER_SF 0.241f // scale factor 241 is 2*820k/6.8k, *1e-3 (mV to V)

uint16_t raw_data[BLOCK_SIZE*2] = {0};
float32_t sample_data[BLOCK_SIZE] = {0.f};
float32_t data_detrend[BLOCK_SIZE] = {0.f};
float32_t fftout[BLOCK_SIZE/2+2]; // output of real FFT
float32_t ps[BLOCK_SIZE]; // power spectrum, in V^2 for each bin (*not* distribution in V^2/Hz)
float32_t block_window[BLOCK_SIZE]; // window, needs to be computed only once
float32_t window_sum, window_sumsq; // window normalizations
static arm_rfft_fast_instance_f32 arm_rfft_S; // needs to be computed only once


#define DIE_TEMP_ALIAS(i) DT_ALIAS(_CONCAT(die_temp, i))
#define DIE_TEMPERATURE_SENSOR(i, _)                                                               \
	IF_ENABLED(DT_NODE_EXISTS(DIE_TEMP_ALIAS(i)), (DEVICE_DT_GET(DIE_TEMP_ALIAS(i)),))

/* support up to 16 cpu die temperature sensors */
static const struct device *const sensors[] = {LISTIFY(16, DIE_TEMPERATURE_SENSOR, ())};
static const struct device *const die_temp_sensor = DEVICE_DT_GET(DT_ALIAS(die_temp0));
static float64_t die_temperature(const struct device *dev);


void adc_init() {

	/* Configure channels individually prior to sampling. */
	for (size_t chan_i= 0U; chan_i< ARRAY_SIZE(adc_channels); chan_i++) {
		if (!device_is_ready(adc_channels[chan_i].dev)) {
			printk("ADC controller device %s not ready\n", adc_channels[chan_i].dev->name);
			// return 0; // XXX
		}
		int err = adc_channel_setup_dt(&adc_channels[chan_i]);
		if (err < 0) {
			printk("Could not setup channel #%d (%d)\n", chan_i, err);
			// return 0; // XXX should fail better
		}
	}
	
	// fft initialization
	arm_status status = arm_rfft_fast_init_f32(&arm_rfft_S, BLOCK_SIZE);
	if (status != ARM_MATH_SUCCESS) {
		printk("arm_rfft_fast_init failure\n");
	}
	arm_hft95_f32(block_window, BLOCK_SIZE); // window function, good to about 0.05% amplitude, ~4 bins wide
	//	arm_accumulate_f32(block_window, BLOCK_SIZE, &window_sum);
	window_sum = window_sumsq = 0.f;
	for (size_t i=0; i< BLOCK_SIZE; i++) {
		window_sum += block_window[i];
		window_sumsq += SQR(block_window[i]);
	}
	
	// initialize raw data to something nonzero by doing a read
	adc_measure();

	if (!device_is_ready(die_temp_sensor)) {
		printk("sensor: device %s not ready.\n", die_temp_sensor->name);
		return 0;
	}
}

void adc_measure() {
	struct adc_sequence sequence = {
		.buffer = &raw_data[0],
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(raw_data),
	};
	struct adc_sequence_options opts = {
		.extra_samplings = BLOCK_SIZE-1U,
	};


		// start_time = timing_counter_get(); // cpu time not wall time

		// first, configure sequence using channel 0. channel number doesn't matter
		// sequence.channels will be incorrect, we will fix after
		(void)adc_sequence_init_dt(&adc_channels[0], &sequence);
		sequence.channels = BIT(0) | BIT(7); // we have channel 7 config also
		sequence.options = &opts;
		int err = adc_read(adc_channels[0].dev, &sequence); // I think what device is linked doesn't depend on the channel
		if (err < 0) {
			printk("Could not read (%d)\n", err);
			return; // XXX error handling
		} 
		// else {
		// 	for (size_t sample_i = 0; sample_i < BLOCK_SIZE; sample_i++) {
		// 		printk("%" PRId32 "\t%" PRId32 "\n", (int32_t) raw_data[2*sample_i+ 0], (int32_t) raw_data[2*sample_i+ 1]);
		// 	}
		// }


		// end_time = timing_counter_get();
		// total_cycles = timing_cycles_get(&start_time, &end_time);
		// total_ns = timing_cycles_to_ns(total_cycles); // not wall time
		// printk("done with reading, datarate %.2f ksps\n", BLOCK_SIZE/(1.e-6*total_ns)); 

}

void adc_calc() {
		float32_t vsum_volt = 0.f;
		int32_t vddsum_mv = 0;

		float32_t v_shift = 0.f;
		float32_t vsum_shift = 0.f;
		float32_t vsumsq_shift = 0.f;
		
		int32_t vdd_shift = 0;
		int32_t vddsum_shift = 0;
		int32_t vddsumsq_shift = 0;

		int32_t val_mv[ARRAY_SIZE(adc_channels)];

		int32_t v0_mv, vdd_mv;

    // start_time = timing_counter_get(); // cpu time not wall time


		// let api scale to mV using devicetree
		for (size_t i = 0; i < BLOCK_SIZE; i++) {
			v0_mv = raw_data[2*i+0]; 
			int err = adc_raw_to_millivolts_dt(&adc_channels[0],
									&v0_mv);
			if (err < 0) {
					printk(" (value in mV not available)\n");
			}
			vdd_mv = raw_data[2*i+1]; 
			err = adc_raw_to_millivolts_dt(&adc_channels[1],
						&vdd_mv);
			if (err < 0) {
					printk(" (value in mV not available)\n");
			}
			// then offset and scale to volts based on voltage dividers
			sample_data[i] = VOLTAGE_DIVIDER_SF*(v0_mv - vdd_mv/2); 
		}

		// stats calculations on block


		// // manual mean and sd
		// float32_t tmpsum = 0.f, tmpsumsq = 0.f;
		// for (int32_t i = 0; i < BLOCK_SIZE; i++) {
		//   tmpsum += sample_data[i];
		//   tmpsumsq += SQR(sample_data[i]);
		// }
		// printk("Mean (sd) %.6f (%.2f) V\n", tmpsum/BLOCK_SIZE, sqrt((tmpsumsq-SQR(tmpsum)/BLOCK_SIZE)/BLOCK_SIZE));

		// now stats and fourier w dsp library
		arm_status status = ARM_MATH_SUCCESS;
		float32_t maxValue, meanValue;
		uint32_t maxIndex;

		float32_t binWidth = (SAMPLE_RATE/BLOCK_SIZE); 
		arm_mean_f32(sample_data, BLOCK_SIZE, &meanValue);
		arm_offset_f32(sample_data, -meanValue, data_detrend, BLOCK_SIZE);
		arm_mult_f32(data_detrend, block_window, data_detrend, BLOCK_SIZE);
		arm_rfft_fast_f32(&arm_rfft_S, data_detrend, fftout, 0);
		// for (size_t i = 0; i < 35; i++) {
		// 	printk("%5.2f\t%10.2f\t%10.2f\n", binWidth*i, fftout[2*i], fftout[2*i+1]);
		// }
		ps[0] = 0.f; // zero out DC from power spectrum (also f_nyquist is in here too)
		arm_cmplx_mag_squared_f32(&fftout[1], &ps[1], BLOCK_SIZE/2-1);
		arm_max_f32(ps, BLOCK_SIZE, &maxValue, &maxIndex);
		if (maxIndex < 5) {
			printk("Max power index %" PRId32 " too small, results will be wrong\n", maxIndex);
		}
		float32_t tonePower = ps[maxIndex]; 
		float32_t harmonicPower = 0.f;
		size_t k = 0U; // will end at number harmonics plus one
		for (k = 2; k < 51; k++ ) { // 50 harmonics or whatever is inside our bandwidth
			if (k*maxIndex >= BLOCK_SIZE/2) {
				break;
			}
			harmonicPower += ps[k*maxIndex];
		}

		float64_t noisePower = 0.f;
		int32_t noiseBins = 0U;
//		printk("[\n");
		for (size_t i = 0; i < BLOCK_SIZE/2; i++) {
			size_t ii = i % maxIndex;
			if (ii == maxIndex-2 || ii == maxIndex - 1 || ii == 0 || ii == 1 || ii == 2) {
				; // skip bins counted for tone or harmonics, or a couple bins either side
			} else {
				noisePower += ps[i];
				noiseBins++;
			}
//			printk("[%.3f,%.6g],\n", binWidth*i, (2*ps[i]/SQR(window_sum))); // power spectrum, voltage scaling
//			printk("[%.3f,%.6g],\n", binWidth*i, (2*ps[i]/(binWidth*window_sumsq))); // power spectral distribution, voltage/rtHz scaling

		}
//		printk("]\n");
		harmonicPower -= (k-2)*noisePower/noiseBins; // subtract white noise background from harmonic distortion measurement
		if (harmonicPower < 0.f) {
			harmonicPower = 0.f; // if harmonic distortion outweighed by noise, display 0.
		}
		printk("DC %.2f Tone: %.2f Hz mag %.2f Vrms phase %.3f rad THD %.2f%% rms noise %.2f V/rtHz %.2f C\n", 
			meanValue,
			binWidth*maxIndex,
			sqrt(2*tonePower/SQR(window_sum)), 
			atan2(fftout[2*maxIndex+1], fftout[2*maxIndex]), 
			100.f*sqrt(harmonicPower/tonePower),
			sqrt(2.f*noisePower/(binWidth*noiseBins*window_sumsq)),
			die_temperature(die_temp_sensor)
			);

		// end_time = timing_counter_get();
		// total_cycles = timing_cycles_get(&start_time, &end_time);
		// total_ns = timing_cycles_to_ns(total_cycles); // not wall time
		// printk("done with calc, %.3f ms\n", (1.e-6*total_ns)); 

		//   k_sleep(K_MSEC(10000));

		// end_time = timing_counter_get();
		// total_cycles = timing_cycles_get(&start_time, &end_time);
		// total_ns = timing_cycles_to_ns(total_cycles);
		// printk("Analysis time %.3f s\n", 1.e-9f*total_ns);



}

void adc_mainloop() {
   adc_measure();
   adc_calc();
}

static float64_t die_temperature(const struct device *dev)
{
	struct sensor_value val;
	int rc;
	float64_t die_temp = 0.f;

	/* fetch sensor samples */
	rc = sensor_sample_fetch(dev);
	if (rc) {
		printk("Failed to fetch sample (%d)\n", rc);
		return rc;
	}

	rc = sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &val);
	if (rc) {
		printk("Failed to get data (%d)\n", rc);
		return rc;
	}
	die_temp = sensor_value_to_double(&val);
//	printk("CPU Die temperature[%s]: %.1f °C\n", dev->name, die_temp);
	return die_temp;
}



// from https://github.com/ARM-software/CMSIS-DSP/blob/main/Source/WindowFunctions/arm_hft95_f32.c
void arm_hft95_f32(
        float32_t * pDst,
        uint32_t blockSize)
{
   float32_t k = 2.0f / ((float32_t) blockSize);
   float32_t w;

   for(uint32_t i=0;i<blockSize;i++)
   {
    w = PI * (i * k);
        w =
    (1.0f -
     1.9383379f * cosf (w) +
     1.3045202f * cosf (2.f * w) -
     0.4028270f * cosf (3.f * w) + 0.0350665f * cosf (4.f * w));

   
     pDst[i] = w;
   }
}