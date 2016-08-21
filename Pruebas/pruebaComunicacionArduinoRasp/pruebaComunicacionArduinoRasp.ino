void setup()
{
	Serial.begin(9600);
}

void loop()
{
	Serial.println("Sebas sigue sin mandar nada...");
  delay(2000);
}

void serialEvent()
{
	if (Serial.available() > 0)
	{
		int valor = Serial.read();
		Serial.print("El valor recibido es ");
		Serial.println(valor);
		Serial.write(valor);
	}
}
