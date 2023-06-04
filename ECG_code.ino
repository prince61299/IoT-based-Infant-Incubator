const int ecgPin = A0;
int ecgValue = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(A0, INPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  ecgValue = analogRead(ecgPin);
  Serial.print("ECG:");
  Serial.println(ecgValue);
  // delay(1000);
}
