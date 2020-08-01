#include <Wire.h>
#include <HSCDTD008A.h>
#include <mimirHEAD.h>

#define addrCompas 0X0C

HSCDTD008A compas(Wire, addrCompas);
mimirHEAD mimir(115200);

int16_t compasX;
int16_t compasY;
int16_t compasZ;
float bearing;

String calcCardinalPoint(float bearing)
{
  String cardinalPoint;

  if (bearing < 90)
  {
    cardinalPoint += "N";
    cardinalPoint += (int)bearing;
    cardinalPoint += "ºE";
  }
  else if (bearing < 180)
  {
    cardinalPoint += "S";
    cardinalPoint += -(int)(bearing - 180);
    cardinalPoint += "ºE";
  }
  else if (bearing < 270)
  {
    cardinalPoint += "S";
    cardinalPoint += (int)(bearing - 180);
    cardinalPoint += "ºW";
  }
  else
  {
    cardinalPoint += "N";
    cardinalPoint += -(int)(bearing - 360);
    cardinalPoint += "ºW";
  }
  return cardinalPoint;
}

void setup(void)
{

  mimir.i2cScanner();
  Serial.println("HSCDTD008A test");
  if (!compas.begin())
  {
    Serial.println("Couldn't find HSCDTD008A");
    while (1)
      delay(1);
  };
  compas.calibrate();
}

void loop()
{
  if (compas.measure())
  {
    compasX = compas.x();
    compasY = compas.y();
    compasZ = compas.z();
    bearing = ((atan2(compasY, compasX)) * 180) / PI;
    if (bearing < 0)
    {
      bearing = 360 + bearing;
    }
  }
  Serial.print("X: ");
  Serial.println(compasX);
  Serial.print("Y: ");
  Serial.println(compasY);
  Serial.print("Z: ");
  Serial.println(compasZ);
  Serial.print("Bearing: ");
  Serial.println(bearing);
  Serial.print("Cardinal Point: ");
  Serial.println(calcCardinalPoint(bearing));
  delay(5000);
}