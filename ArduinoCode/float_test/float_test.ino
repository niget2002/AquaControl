void setup() {
  // put your setup code here, to run once:
  pinMode(15, INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int pin;
  int i;
  int average;
  for(i=0;i<=10;i++){
    pin = pin+digitalRead(15);
    delay(10);
  }
  average=pin/10;
  if(average==52){
    Serial.println("Up");
  }
  if(average==51){
    Serial.println("Down");
  }
  delay(100);
}
