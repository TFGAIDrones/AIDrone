#define usCiclo 6000  // 
// ==========================================
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

//======= VARIABLES ===========//
// TIME
long tiempo_ejecucion, loop_timer, loop_timer1, esc_loop_timer, t_ON, t_1, t_2, previousMillis;

// VBAT
bool AvisoVbatLow = false;
float real_voltage, battery_read = 0.00;
int cbat, Cont_low_Vbat, low_bat = 0;

// IMU
float angulo_pitch, angulo_roll, angulo_yaw, yawGyro, pitchGyro, rollGyro, angle_pitch_acc, angle_roll_acc, temperature;
int emerg, gx, gy, gz;
float acc_total_vector, ax, ay, az;
float gyro_X_cal, gyro_Y_cal, gyro_Z_cal, gyro_roll_input , gyro_pitch_input , gyro_yaw_input;
float angle_pitch_acc_cal, angle_roll_acc_cal, angle_yaw_acc_cal;

/// MOTORS
float esc1, esc2, esc3, esc4;

// RC CONTROLLER
float calPotencia, pulsoThrottle, pulsoPotenciaAnt, PotenciaFilt = 0.00;
float wPitchConsigna, wRollConsigna, wYawConsigna = 0.00;
float calPitch, wPitch,  wPitchFilt = 0.00;
float calRoll, wRoll, wRollFilt = 0.00;
float calYaw, wYaw,  wYawlFilt = 0.00;

// PID
float PIDwInPitch, pitch_w_giroscopio, pitch_w_error_prop, ITerm_pitch_w, pitch_w_giroscopio_anterior, PID_pitch_w, DPitch_w;
float PIDwInRoll, roll_w_giroscopio, roll_w_error_prop, ITerm_roll_w, roll_w_giroscopio_anterior, PID_roll_w, DRoll_w;
float PIDwInYaw, yaw_w_giroscopio, yaw_w_error_prop, ITerm_yaw_w, yaw_w_giroscopio_anterior, PID_yaw_w, PID_yaw_w_ant, DYaw_w;
float pitch_ang_giroscopio, pitch_ang_error_prop, ITerm_pitch_ang, pitch_ang_giroscopio_anterior, PID_pitch_ang, DPitch_ang;
float  roll_ang_giroscopio, roll_ang_error_prop, ITerm_roll_ang, roll_ang_giroscopio_anterior, PID_roll_ang, DRoll_ang;

// CALIBRATION
int cal_int = 0;

// AUTOMATIC TAKEOFF
bool Enciendete = false;
float hoverDistance = 40;
bool hover = false;
Despegado = false;
//=============================================//////////////////// SETUP ////////////////////=============================================
void setup() {
	TWBR = 24;    //Set the I2C clock speed to 400kHz.
	lcd.init();
	lcd.backlight();
	
	// LEDs
	pinMode(11, OUTPUT); // Red battery LED
	
	// RC Controller
	pinMode(7, INPUT_PULLUP);                  // YAW
  	attachInterrupt(digitalPinToInterrupt(7), INTyaw, CHANGE);
	pinMode(8, INPUT_PULLUP);                  // POTENCIA
  	attachInterrupt(digitalPinToInterrupt(8), ComputeThrottle, CHANGE);
	pinMode(12, INPUT_PULLUP);                 // PITCH
  	attachInterrupt(digitalPinToInterrupt(12), INTpitch, CHANGE);
	pinMode(9, INPUT_PULLUP);                  // ROLL
  	attachInterrupt(digitalPinToInterrupt(9), INTroll, CHANGE);

	
	// Motors
	pinMode(3, OUTPUT);  //Motor 1
	pinMode(4, OUTPUT);  //Motor 2
	pinMode(5, OUTPUT);  //Motor 3
	pinMode(6, OUTPUT);  //Motor 4
	
	}
	
	BatCheck();     // Read battery voltage
	IMU_calibration();
	Calib_Motores(); // Motor calibration
	
	loop_timer = micros();
}

// =============================================//////////////////// LOOP ////////////////////=============================================
void loop() {
	Compute_angles();                     // Get angles 
	PID_ang();     // Angle PID output
	PID_w();                         // Velocity PID output
	Mixer();                     // Mixer input + PIDs

	while (micros() - loop_timer < usCiclo); // Comienzo de un nuevo ciclo
	loop_timer = micros();          // Registrar instante de comienzo del ciclo
	PWM();                          // Generate PWMs to motors

	//Wait 2200us (our PWM time is at maximum 2000us)to read our IMU again.
	while (micros() - loop_timer < 2200);
	Compute_angles();    

}
