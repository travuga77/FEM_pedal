
#include <mcp_can.h>
#include <SPI.h>

#define CAN0_INT 2                              // Set INT to pin 2
MCP_CAN CAN0(10);                               // Set CS to pin 10

#define lSensorPin 5
#define rSensorPin 6

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
int lSensorData=0, rSensorData=0, lSensorConv=0, rSensorConv=0; 
unsigned int count=0;
unsigned long lSensorData_collect=0, rSensorData_collect=0;
byte data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};

unsigned long timer = 0;
int errCode;

void setup() {
  // put your setup code here, to run once:
   Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  if (CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT);  // Configuring pin for /INT input
  pinMode(A6, INPUT);
  digitalWrite(A6, LOW);
  pinMode(A1, INPUT);
  digitalWrite(A1, LOW);
}



void loop() {
 // put your main code here, to run repeatedly:
  
 lSensorData_collect += analogRead(lSensorPin);
 rSensorData_collect += analogRead(rSensorPin);
 count++;
 if (!digitalRead(CAN0_INT)) {                       // If CAN0_INT pin is low, read receive buffer
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s) 
  }
  switch (rxId) {
    case 0x80:
      // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
      lSensorData = lSensorData_collect/count;
      rSensorData = rSensorData_collect/count;
      lSensorConv = map(lSensorData, 695, 1010, 0, 4096);
      rSensorConv = map(rSensorData, 30, 310, 0, 4096);
      lSensorConv = constrain(lSensorConv, 0, 4095);
      rSensorConv = constrain(rSensorConv, 0, 4095);
      int minim; byte sndStat;
      if (lSensorConv<rSensorConv) minim=lSensorConv; else minim=rSensorConv;

      data[0] = minim%256;
      data[1] = minim>>8;
      sndStat = CAN0.sendMsgBuf(0x150, 0, 2, data);
      if (millis()-timer>1000) {
        data[0] = errCode%256;
        data[1] = errCode>>8;
        sndStat = CAN0.sendMsgBuf(0x151, 0, 2, data);
        timer = millis();
        Serial.println(errCode);
      }
      Serial.print(minim); 
      Serial.print(" "); 
      Serial.print(lSensorData); 
      Serial.print(" "); 
      Serial.print(rSensorData); 
      Serial.print(" ");
      Serial.print(lSensorConv); 
      Serial.print(" ");
      Serial.println(rSensorConv);
      rxId=count=lSensorData_collect=rSensorData_collect=0;
      break;  
    default:
      break;
  }
}
