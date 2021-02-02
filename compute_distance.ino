void computeDistance(){
	hoverDistance = 40; //In cm
	if(hover!=true){
		digitalWrite(trigPin,LOW);
		
		currentMillis = micros();
		if(currentMillis - previousMillis > 4){
			digitalWrite(trigPin,HIGH);
			previousMillis = currentMillis; //previousMillis is defined in void setup, therefore will only change if we call the variable.
		}	
		if(currentMillis - previousMillis > 12){
			digitalWrite(trigPin,LOW);
			previousMillis = currentMillis; 
		}	
		duration = pulseIn(echoPin,HIGH);
		
		distance = duration * 0.034/2; //Speed of sound/ go and back
		
		if (distance > hoverDistance){
			hover = true;
		}
	}
}
