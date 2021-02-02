void Calib_Motores() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibrar motores");
	//We will remain in this loop until roll stick is pushed to the right.
  while (wRoll < 20) {
    pulsoThrottle =  ((PulsoMaxPotencia - PulsoMinPotencia) / (tMinPotencia - tMaxPotencia)) * ((pulsoThrottle) / 1000.00 - tMaxPotencia) + PulsoMinPotencia;
    wRoll =   ((wMaxRoll - wMinRoll) / (tMax - tMin)) * ((PulsoRoll - calRoll) / 1000.00 - tMin) + wMinRoll;
    digitalWrite(3, HIGH); //Motor 1
    digitalWrite(4, HIGH); //Motor 2
    digitalWrite(5, HIGH); //Motor 3
    digitalWrite(6, HIGH); //Motor 4
    delayMicroseconds(pulsoThrottle);
    digitalWrite(3, LOW); //Motor 1
    digitalWrite(4, LOW); //Motor 2
    digitalWrite(5, LOW); //Motor 3
    digitalWrite(6, LOW); //Motor 4
    delayMicroseconds(usCiclo - pulsoThrottle);
  }
  lcd.clear();
  lcd.noBacklight();
}
