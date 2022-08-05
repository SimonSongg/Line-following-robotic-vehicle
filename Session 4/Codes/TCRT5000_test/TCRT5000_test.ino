void setup() {
  pinMode(2, INPUT);
  Serial.begin(9600);
}

void loop() {
  Serial.print("L:");
  Serial.print(analogRead(A0));
  Serial.print("\t");
  Serial.print("R:");
  Serial.println(analogRead(A1));
}
