bool set_gyro_angles, accCalibOK  = false;

void IMU_calibration() {

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  // Gyroscope calibration
  for (cal_int = 0; cal_int < 3000 ; cal_int ++) {
    if (IMU.gyroscopeAvailable()){
        IMU.readGyroscope(gxcal, gycal, gzcal);
      }
    gyro_X_cal += gxcal;
    gyro_Y_cal += gycal;
    gyro_Z_cal += gzcal;
    delayMicroseconds(1000);
  }
  gyro_X_cal = gyro_X_cal / 3000;       // average sample value
  gyro_Y_cal = gyro_Y_cal / 3000;
  gyro_Z_cal = gyro_Z_cal / 3000;
 
  // Accelerometer calibration
  for (cal_int = 0; cal_int < 3000 ; cal_int ++) {
   if (IMU.accelerationAvailable()){
        IMU.readAcceleration(axcal, aycal, azcal);
      }
    angle_pitch_acc_cal += axcal;
    angle_roll_acc_cal += aycal;
    angle_yaw_acc_cal += azcal;
    delayMicroseconds(1000);
  }
  angle_pitch_acc_cal = angle_pitch_acc_cal / 3000;
  angle_roll_acc_cal = angle_roll_acc_cal / 3000;
  angle_yaw_acc_cal = angle_yaw_acc_cal / 3000;
}

void Compute_angles(){
  float ax, ay, az;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    ax -= angle_pitch_acc_cal;
    ay -= angle_roll_acc_cal;
    az -= angle_yaw_acc_cal;
    acc_total_vector = sqrt(pow(ay, 2) + pow(ax, 2) + pow(az, 2));
    angle_pitch_acc = asin((float)ax / acc_total_vector) * 57.2958;
    angle_roll_acc = asin((float)ay / acc_total_vector) * -57.2958;
  }

   float gx, gy, gz;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    pitchGyro = (gx - gyro_X_cal); 
    rollGyro = (gy - gyro_Y_cal);
    yawGyro = (gz - gyro_Z_cal);
  }
  angulo_pitch += pitchGyro * tiempo_ejecucion / 1000 ;
  angulo_roll += rollGyro * tiempo_ejecucion / 1000 ;
  angulo_yaw += yawGyro * tiempo_ejecucion/1000;
  angulo_pitch = angulo_pitch * 0.96 + angle_pitch_acc * 0.04;      // Drift correction complementary filter
  angulo_roll = angulo_roll * 0.96 + angle_roll_acc * 0.04;
}
