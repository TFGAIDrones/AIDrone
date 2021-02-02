void VoiceCommand() //This will be our function since void setup is made in the main program
{
	ei_printf("Starting inferencing in 2 seconds...\n");
	delay(2000);
	ei_printf("Recording...\n");
	bool m = microphone_inference_record();
	if (!m) {
		ei_printf("ERR: Failed to record audio...\n");
		return;
	}
	ei_printf("Recording done\n");
	signal_t signal;
	signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
	signal.get_data = &microphone_audio_signal_get_data;
	ei_impulse_result_t result = { 0 };
	EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
	if (r != EI_IMPULSE_OK) {
		ei_printf("ERR: Failed to run classifier (%d)\n", r);
		return;
	}
	// print the predictions
	ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
	result.timing.dsp, result.timing.classification, result.timing.anomaly);
	for (size_t ix = 2; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
		noise = result.classification[ix].value;
		Serial.println("Noise: ");
		Serial.println(noise);
	}  
	for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix--) {
		lightoff = result.classification[ix].value;
		Serial.println("Light Off: ");
		Serial.print(lightoff);
	} 
	lighton = 1- (noise +lightoff);
	Serial.println("Light ON: ");
	Serial.print(lighton);
	if (lighton > 0.60){
		digitalWrite(13, HIGH);
		Enciendete=true;
	}
	if (lightoff > 0.29){
		digitalWrite(13, LOW);
		Enciendete=false;
	}  
	#if EI_CLASSIFIER_HAS_ANOMALY == 1
	ei_printf("    anomaly score: %.3f\n", result.anomaly);
	#endif
}
void ei_printf(const char *format, ...) {
	static char print_buf[1024] = { 0 };
	va_list args;
	va_start(args, format);
	int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
	va_end(args);
	if (r > 0) {
		Serial.write(print_buf);
	}
}

static void pdm_data_ready_inference_callback(void)
{
	int bytesAvailable = PDM.available();
	// read into the sample buffer
	int bytesRead = PDM.read((char *)&sampleBuffer[0], bytesAvailable);
	if (record_ready == true || inference.buf_ready == 1) {
		for(int i = 0; i < bytesRead>>1; i++) {
			inference.buffer[inference.buf_count++] = sampleBuffer[i];
			if(inference.buf_count >= inference.n_samples) {
				inference.buf_count = 0;
				inference.buf_ready = 1;
			}
		}
	}
}
static bool microphone_inference_start(uint32_t n_samples) 
{
	inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));
	if(inference.buffer == NULL) {
		return false;
	}
	inference.buf_count  = 0;
	inference.n_samples  = n_samples;
	inference.buf_ready  = 0;
	// configure the data receive callback
	PDM.onReceive(&pdm_data_ready_inference_callback);
	// optionally set the gain, defaults to 20
	PDM.setGain(80);
	//ei_printf("Sector size: %d nblocks: %d\r\n", ei_nano_fs_get_block_size(), n_sample_blocks);
	PDM.setBufferSize(4096);
	// initialize PDM with:
	// - one channel (mono mode)
	// - a 16 kHz sample rate
	if (!PDM.begin(1, EI_CLASSIFIER_FREQUENCY)) {
		ei_printf("Failed to start PDM!");
	}
	record_ready = true;
	return true;
}

static bool microphone_inference_record(void) //Starts recording
{
	inference.buf_ready = 0;
	inference.buf_count = 0;
	while(inference.buf_ready == 0) {
		delay(10);
	}
	return true;
}
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)//Gets data from our audio pdm signal
{
	arm_q15_to_float(&inference.buffer[offset], out_ptr, length);
	return 0;
}
static void microphone_inference_end(void) //This function ends the recording
{
	PDM.end();
	free(inference.buffer);
}
#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif
