

void accelTask(void *pvParameters);
void monitorAdxl345Task(void *pvParameters);
// from analysis.c
void analysis_init();
int alarm_check();
void accel_handler(ADXL345_IVector *data, uint32_t num_samples);
void do_analysis();
void check_fall();
int getAmpl(int nBin);
