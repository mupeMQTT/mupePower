#include <stdio.h>
#include "mupePower.h"
#include <math.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "mupeClientMqtt.h"
#include <esp_log.h>


#include "esp_adc/adc_oneshot.h"
#include "mupeMdnsNtp.h"

static const char *TAG = "mupePowe";

float lf[3][40];

float getl(size_t xi) {
	double ret = 0;
	for (size_t x = 0; x < 40; ++x) {
		ret = ret + lf[xi][x];

	}
	ret = (ret / 41) * 0.022;
	ret = ret < 2 ? 0 : ret;
	return ret;
}

float getl1() {
	return getl(0);
}
float getl2() {
	return getl(1);
}

float getl3() {
	return getl(2);
}

void mupePowerInit(void) {


}

double goertzel(int numSamples, int TARGET_FREQUENCY, int SAMPLING_RATE,
		double *data) {
	int k, i;
	double floatnumSamples;
	double omega, sine, cosine, coeff, q0, q1, q2, result, real, imag;

	floatnumSamples = (float) numSamples;
	k = (int) (0.5 + ((floatnumSamples * TARGET_FREQUENCY) / SAMPLING_RATE));
	omega = (2.0 * M_PI * k) / floatnumSamples;
	sine = sin(omega);
	cosine = cos(omega);
	coeff = 2.0 * cosine;
	q0 = 0;
	q1 = 0;
	q2 = 0;

	for (i = 0; i < numSamples; i++) {
		q0 = coeff * q1 - q2 + data[i];
		q2 = q1;
		q1 = q0;
	}
	real = (q1 - q2 * cosine);
	imag = (q2 * sine);
	result = sqrtf(real * real + imag * imag);
	return result;
}

void mupePowerServerTask(void *pvParameters) {

	adc_oneshot_unit_handle_t adc1_handle;
	adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1, };
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
	adc_oneshot_chan_cfg_t config = { .bitwidth = ADC_BITWIDTH_12, .atten =
			ADC_ATTEN_DB_11, };
	int adc_raw;
	ESP_ERROR_CHECK(
			adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config));
	ESP_ERROR_CHECK(
			adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_5, &config));
	ESP_ERROR_CHECK(
			adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config));

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(1000);
	xLastWakeTime = xTaskGetTickCount();

	double f[256];
	waitForNTPConnect();

	size_t lx = 0;
	uint64_t time=getNowMs()+1000*60;

	while (1) {
		uint64_t t = getNowMs();
		for (int i = 0; i < 256; i++) {

			ESP_ERROR_CHECK(
					adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &adc_raw));
			//	ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1,ADC_CHANNEL_4, adc_raw);
			f[i] = (double) adc_raw;
			delayMicroseconds(550);

		}
		t = getNowMs() - t;
		int freq = 256.0 / (((float) t) / 1000);
		lf[0][lx] = goertzel(256, 50, freq, f);
		t = getNowMs();
		for (int i = 0; i < 256; i++) {

			ESP_ERROR_CHECK(
					adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &adc_raw));
			//	ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1,ADC_CHANNEL_4, adc_raw);
			f[i] = (double) adc_raw;
			delayMicroseconds(550);

		}
		t = getNowMs() - t;
		freq = 256.0 / (((float) t) / 1000);

		lf[1][lx] = goertzel(256, 50, freq, f);
		t = getNowMs();
		for (int i = 0; i < 256; i++) {

			ESP_ERROR_CHECK(
					adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &adc_raw));
			//	ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1,ADC_CHANNEL_4, adc_raw);
			f[i] = (double) adc_raw;
			delayMicroseconds(550);

		}
		t = getNowMs() - t;
		freq = 256.0 / (((float) t) / 1000);

		lf[2][lx] = goertzel(256, 50, freq, f);

	//	ESP_LOGI(TAG, "l1,l2,l3 %f %f %f ", getl1(), getl2(), getl3());
		lx++;
		lx = (lx < 40) ? lx : 0;
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		if (time <getNowMs()) {
			time=time+1000*60;
			uint64_t now = getNowMs();
			char json[255];
			char topic[] = "mupe/power";
			sprintf(json,
					"{\"src\":\"%s\", \"parmas\":{\"ts\":%llu,\"l1\":%f,\"l2\":%f,\"l3\":%f,\"l123\":%f }}",
					topic, now / 1000, getl1(), getl2(), getl3(),
					getl1() + getl2() + getl3());
			mupeClientSend(topic, json);
			ESP_LOGI(TAG, "vTaskModbus %s", json);

		}

	}
}

