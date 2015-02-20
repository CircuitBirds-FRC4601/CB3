#include <Wire.h>

/*Variables*/
byte payloadReturn = 0x0;
boolean States[6];
int binary[8] = {1, 2, 4, 8, 16, 32, 64, 128};

/*
Map
3    lift T 0
4    lift B 1
5  R. Arm R 2
6  R. Arm L 3
7  L. Arm R 4
8  L. Arm L 5
*/

void setup()
{
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event

  Serial.begin(115200);           // start serial for output

  for (int i = 3; i <= 8; i++)  //Setup limit Pins
  {
    Serial.print("Setting Pin ");
    Serial.print(i);
    Serial.print(" to \'INPUT_PULLUP\'\n");
    pinMode(i, INPUT_PULLUP);
  }
  pinMode(2,OUTPUT);
  digitalWrite(2,LOW);

}


void loop()
{
 for (int i = 3; i<=8; i++)
  {
    digitalWrite(13, HIGH);
    if (!digitalRead(i)) payloadReturn |= binary[i - 3]; //The magic happens here
  }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()

void receiveEvent(int howMany)
{
  Serial.println("-------START------");
  
  int regAddr = Wire.read();
  int counter = 0 ;
  Serial.print("\n\nReg Arddress: ");
  Serial.println(regAddr, HEX);
  
  while (0 < Wire.available()) // loop through all but the last
  {
    int c = Wire.read(); // receive byte as a character
    Serial.print("Byte# ");
    Serial.print(counter);
    Serial.print(", Incomming Payload: ");
    Serial.println(c, HEX); // print the character
    switch (c)
    {
      case 0xA:
        Serial.println("A was pressed");
        break;
      case 0xB:
        Serial.println("B was pressed");
        break;
      case 0xC:
        Serial.print("Transmitting Data:");
        Serial.println(payloadReturn,BIN);
        Wire.write(payloadReturn);
        break;
    }
    counter++;
    payloadReturn = 0x0;
  }
  Serial.print("counter: ");
  Serial.println(counter);
  Serial.println("-------STOP-------");
}
