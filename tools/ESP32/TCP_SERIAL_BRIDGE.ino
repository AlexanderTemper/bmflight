#include <WiFi.h>
#include <LoopbackStream.h>

const char* ssid     = "";
const char* password = "";

LoopbackStream serialInBuffer(1024);

WiFiServer server(8800);

void setup()
{
    Serial.begin(115200);
    Serial2.begin(500000,SERIAL_8N1,18,19);
    delay(10);

    // We start by connecting to a WiFi network
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    
    server.begin();

}

WiFiClient client;
unsigned long current,diff,old = 0;

void loop() {
 
 client = server.available();
  if (client) {
    while (client.connected()) {
      while (client.available()>0) {
        char c = client.read();
        Serial2.write(c);
      }
      current = micros();
      diff = current - old;
      // Copy Serial Data
      while (Serial2.available()) {
        char c = (char)Serial2.read();
        if(serialInBuffer.available() > 1024){
          Serial.println("Buffer overflow SerialInBuffer!!");
        } else {
          serialInBuffer.write(c);
        }
      }
      if (serialInBuffer.available() >= 1000 || (serialInBuffer.available() > 0  && ((diff) >= 1000) )) {
        sendTcpBuffer();
      }
      
      
    }
    client.stop();
    Serial.println("Client disconnected");
  }
}


char tcpOutBuffer[1024];
char debugbuffer[1024];
void sendTcpBuffer() {
  int i = 0;
  while(serialInBuffer.available()){
    tcpOutBuffer[i] = serialInBuffer.read();
    if(i>1000){
      Serial.write("overflow udpOutBuffer");
      break;
    }
    i++;
  }
  client.write((uint8_t *)&tcpOutBuffer[0],i);
  old = current;
}
