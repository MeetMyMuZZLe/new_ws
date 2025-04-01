#ifndef __CC3200R1M1RGC__
#include <SPI.h>
#endif
#include <WiFi.h>
char ssid[] = "Pranav's iPhone";
char password[] = "12378956";
int keyIndex = 0;
void setup()
{
Serial.begin(115200);
Serial.println("**Simplelink IP by name demo**");
Serial.print("Attempting to connect to Network named: ");
Serial.println(ssid);
WiFi.begin(ssid, password);
while ( WiFi.status() != WL_CONNECTED)
{
Serial.print(".");
delay(300);
}
Serial.println("\nYou're connected to the network");
Serial.println("Waiting for an ip address");
while (WiFi.localIP() == INADDR_NONE)
{
Serial.print(".");
delay(300);
}
Serial.println("\nIP Address obtained");
printWifiStatus();
}
#define STRLEN 32
char buffer[STRLEN];
int bufferIndex = 0;
void loop()
{
while(Serial.available())
{
buffer[bufferIndex] = Serial.read();
if( buffer[bufferIndex] != '\r')
{
bufferIndex++;
continue;
}
buffer[ bufferIndex ] = 0;
bufferIndex = 0;
Serial.print("Getting IP for ");
Serial.println(buffer);
IPAddress IP(0,0,0,0);
int iRet = WiFi.hostByName(buffer, IP);
if(iRet < 0)
{
Serial.println("Host name lookup failed");
}
else
{
Serial.println(IP);
Serial.flush();
}
}
}
void printWifiStatus()
{
Serial.print("Network Name: ");
Serial.println(WiFi.SSID());
IPAddress ip = WiFi.localIP();
Serial.print("IP Address: ");
Serial.println(ip);
long rssi = WiFi.RSSI();
Serial.print("signal strength (RSSI):");
Serial.print(rssi);
Serial.println(" dBm");
}
