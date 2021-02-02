bool aM1, aM2, aM3, aM4 = false;
float accion_m1, accion_m2, accion_m3, accion_m4, loop_timer, esc_loop_timer;

void Mixer () {
  //If throttle is under 1300us, PID will not take part in PWM signals.
  if (pulsoThrottle <= 1300) {
    ITerm_pitch_w = 0;
    ITerm_roll_w = 0;
    ITerm_yaw_w = 0;
    ITerm_pitch_ang = 0;
    ITerm_roll_ang = 0;
    esc1 = PotenciaFilt;
    esc2 = PotenciaFilt;
    esc3 = PotenciaFilt;
    esc4 = PotenciaFilt;
  }
  if (pulsoThrottle > 1300) {
    if (pulsoThrottle > 1800)pulsoThrottle = 1800; // Constraining throttle input so PID can work properly for at least 10%
    esc1 = PotenciaFilt + PID_pitch_w - PID_roll_w - PID_yaw_w; // Motor 1
    esc2 = PotenciaFilt + PID_pitch_w + PID_roll_w + PID_yaw_w; // Motor 2
    esc3 = PotenciaFilt - PID_pitch_w + PID_roll_w - PID_yaw_w; // Motor 3
    esc4 = PotenciaFilt - PID_pitch_w - PID_roll_w + PID_yaw_w; // Motor 4

    if (esc1 < 1100) esc1 = 1100; // Avoid motors to stop in mid-air
    if (esc2 < 1100) esc2 = 1100;
    if (esc3 < 1100) esc3 = 1100;
    if (esc4 < 1100) esc4 = 1100;
    if (esc1 > 2000) esc1 = 2000; // Avoid sending signals bigger than 2000us
    if (esc2 > 2000) esc2 = 2000;
    if (esc3 > 2000) esc3 = 2000;
    if (esc4 > 2000) esc4 = 2000;
  }
}

void PWM() {
  digitalWrite(3, HIGH); //Motor 1 HIGH
  digitalWrite(4, HIGH); //Motor 2 HIGH
  digitalWrite(5, HIGH); //Motor 3 HIGH
  digitalWrite(6, HIGH); //Motor 4 HIGH

  aM1 = true;
  aM2 = true;
  aM3 = true;
  aM4 = true;

  t_1 = micros();
  accion_m1 = esc1 + loop_timer;
  accion_m2 = esc2 + loop_timer;
  accion_m3 = esc3 + loop_timer;
  accion_m4 = esc4 + loop_timer;

  lectura_mandoRC(); // Leer mando RC
  VbatCheck();       // Leer Vbat

  // ===================================================== 1ms max!!!!!!!

  while (aM1 || aM2 || aM3 || aM4 == true) {
    esc_loop_timer = micros();
    if (accion_m1 <= esc_loop_timer) { // Motor 1 LOW
      aM1 = false;
      digitalWrite(3, LOW);
    }
    esc_loop_timer = micros();
    if (accion_m2 <= esc_loop_timer) { // Motor 2 LOW
      aM2 = false;
      digitalWrite(4, LOW);
    }
    esc_loop_timer = micros();
    if (accion_m3 <= esc_loop_timer) { // Motor 3 LOW
      aM3 = false;
      digitalWrite(5, LOW);
    }
    esc_loop_timer = micros();
    if (accion_m4 <= esc_loop_timer) { // Motor 4 LOW
      aM4 = false;
      digitalWrite(6, LOW);
    }
  }
}
