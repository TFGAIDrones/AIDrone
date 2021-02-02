//Throttle boundaries
const int PulsoMaxPotencia = 2000;
const int PulsoMinPotencia = 1000;
const float tMaxPotencia = 1.82;  // <-- Si teneis la entrada Throttle invertida sustituid este valor
const float tMinPotencia = 1.10;  // <-- por este y viceversa
 
const float tMax = 2;
const float tMin = 1;
 
//Pitch boundaries
const int wMaxPitch = -25; // <-- Si teneis la entrada Pitch invertida sustituid este valor
const int wMinPitch = 25;  // <-- por este y viceversa
const float tMaxPitch = 1.95;  // <-- Si teneis la entrada Throttle invertida sustituid este valor
const float tMinPitch = 1.04;

//Roll boundaries
const int wMaxRoll = 25;  // 
const int wMinRoll = -25; // 
const float tMaxRoll = 1.04; 
const float tMinRoll = 1.98;  
 
//Yaw boundaries
const int wMaxYaw = 25;  // <-- Si teneis la entrada Yaw invertida sustituid este valor
const int wMinYaw = -25; // <-- por este y viceversa

//Throttle function when stick is used
volatile long contadorPotenciaInit; //Throttle is computed from time 0 to time end of counter
volatile int PulsoThrottle;
void ComputeThrottle() {
  if (digitalRead(8) == HIGH) contadorPotenciaInit = micros();
  if (digitalRead(8) == LOW)PulsoThrottle = micros() - contadorPotenciaInit;
}

//Pitch function when stick is used
volatile long contadorPitchInit; 
volatile int PulsoPitch;
void INTpitch() {
  if (digitalRead(12) == HIGH) contadorPitchInit = micros();
  if (digitalRead(12) == LOW) PulsoPitch = micros() - contadorPitchInit;
}

//Roll function when stick is used
volatile long contadorRollInit; 
volatile int PulsoRoll;
void INTroll() {
  if (digitalRead(9) == HIGH) contadorRollInit = micros();
  if (digitalRead(9) == LOW) PulsoRoll = micros() - contadorRollInit;
}

//Yaw function when stick is used
volatile long contadorYawInit; 
volatile int PulsoYaw;
void INTyaw() {
  if (digitalRead(7) == HIGH) contadorYawInit = micros();
  if (digitalRead(7) == LOW) PulsoYaw = micros() - contadorYawInit;
}
void lectura_mandoRC() {
  // =============== From pulse to degrees equations
  wPitch =   ((wMinPitch - wMaxPitch) / (tMaxPitch - tMinPitch)) * ((PulsoPitch ) / 1000.00 - tMinPitch) + wMaxPitch;
  wRoll =   ((wMaxRoll - wMinRoll) / (tMaxRoll - tMinRoll)) * ((PulsoRoll) / 1000.00 - tMinRoll) + wMinRoll;
  wYaw =  ((wMinYaw - wMaxYaw) / (tMax - tMin)) * ((PulsoYaw) / 1000.00 - tMin) + wMaxYaw;
  pulsoThrottle =  ((PulsoMaxPotencia - PulsoMinPotencia) / (tMinPotencia-tMaxPotencia)) * ((PulsoThrottle) / 1000.00 - tMaxPotencia) + PulsoMinPotencia;
  // =============== Using linear equation for each channel
  
  // =============== Filter
  PotenciaFilt = PotenciaFilt * 0.9 + pulsoPotencia * 0.1;
  wPitchFilt = wPitchFilt * 0.9 + wPitch * 0.1;
  wRollFilt = wRollFilt * 0.9 + wRoll * 0.1;
  wYawlFilt = wYawlFilt * 0.9 + wYaw * 0.1;
  
  wPitchConsigna = wPitchFilt;
  wRollConsigna = wRollFilt;
  wYawConsigna = wYawlFilt;
  
  // ====== Reduce unnecessary oscillations if inputs are near to 0
  if (wPitchFilt < 3  && wPitchFilt > -3)wPitchConsigna = 0; 
  if (wRollFilt < 3 && wRollFilt > -3)wRollConsigna = 0;
  if (wYawlFilt < 3  && wYawlFilt > -3)wYawConsigna = 0;
  
  // ====== In case our inputs remain constant during 150 loops, we will turn on our warning LED to avoid controller connection losses.
  if (pulsoPotenciaAnt == pulsoPotencia)cont_seguridad2++;
  else cont_seguridad2 = 0;
  if (cont_seguridad2 > 150) { 
    digitalWrite(11,HIGH);
  }
  pulsoPotenciaAnt = pulsoPotencia;
  }
