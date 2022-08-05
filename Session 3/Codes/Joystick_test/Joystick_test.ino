void setup()
{
  Serial.begin(9600);
    pinMode(2, INPUT_PULLUP);
}

void loop()
{
    delay(100);
    Serial.print("X:");
    Serial.print(analogRead(A6));
    Serial.print("\t");
    Serial.print("Y:");
    Serial.println(analogRead(A7));
}
