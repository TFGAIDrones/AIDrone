void BatCheck() {
	battery_read = analogRead(A5); // Read analog pin A5
	real_voltage = (330+120)/120 * (battery_read * 3.3  / 1023); //Scale it back to real voltage and print it 
	lcd.backlight();
	lcd.setCursor(0,0);
	lcd.print(real_voltage);
	//Start worrying if real voltage drains under 10.5, and use a count to avoid false readings.
	if (real_voltage < 10.5) {
		Cont_low_Vbat++;
		AvisoVbatLow = true;
	}
	else Cont_low_Vbat = 0;
	if (visu == 0 && matlab == 0 && Cont_low_Vbat > 500) {
		low_bat = 1;
		digitalWrite(11,HIGH);//If low battery is met, light on a warning LED and print low battery into LCD
		lcd.backlight();
		lcd.setCursor(0, 0);
		lcd.print("LOW BATTERY");
	}
	else low_bat = 0;
}
