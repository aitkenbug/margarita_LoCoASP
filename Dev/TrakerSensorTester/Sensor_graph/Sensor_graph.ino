//Grafica el sensor 
void setup(){
    Serial.begin(115200);

}
void loop(){
    int sensor0 = analogRead(A0);
    delay(1);
    int sensor1 = analogRead(A1);
    delay(1);
    int sensor2 = analogRead(A2);
    delay(1);
    int sensor3 = analogRead(A3);
    delay(1);
    Serial.print("Sensor1:");
    Serial.print(sensor0);
    Serial.print(",Sensor2:");
    Serial.print(sensor1);
    Serial.print(",Sensor3:");
    Serial.print(sensor2);
    Serial.print(",Sensor4:");
    Serial.println(sensor3);
    delay(100);
}
