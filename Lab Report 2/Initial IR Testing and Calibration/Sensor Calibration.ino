//Far Left = 1, Far right = 6

int sensor1 = 32;
int sensor2 = 35;
int sensor3 = 33;
int sensor4 = 2;
int sensor5 = 0;
int sensor6 = 4;

int reading1 = 0;
int reading2 = 0;
int reading3 = 0;
int reading4 = 0;
int reading5 = 0;
int reading6 = 0;

void setup() {

  Serial.begin(115200);
}

void loop() {

  int reading1 = 0;
  int reading2 = 0;
  int reading3 = 0;
  int reading4 = 0;
  int reading5 = 0;
  int reading6 = 0;

  reading1 = analogRead(sensor1);
  reading2 = analogRead(sensor2);
  reading3 = analogRead(sensor3);
  reading4 = analogRead(sensor4);
  reading5 = analogRead(sensor5);
  reading6 = analogRead(sensor6);


  Serial.print("Sensor 1: ");
  Serial.println(reading1);
  Serial.print("Sensor 2: ");
  Serial.println(reading2);
  Serial.print("Sensor 3: ");
  Serial.println(reading3);
  Serial.print("Sensor 4: ");
  Serial.println(reading4);
  Serial.print("Sensor 5: ");
  Serial.println(reading5);
  Serial.print("Sensor 6: ");
  Serial.println(reading6);

  delay(1500);
}
