int value=0;

void setup() {
  Serial.begin(9600);
  value=0;

}

void loop() {
  // put your main code here, to run repeatedly:
   value=analogRead(A0);
   Serial.println(value);
   if(value==0)
    Serial.println("knock!");
   delay(50);
   
}