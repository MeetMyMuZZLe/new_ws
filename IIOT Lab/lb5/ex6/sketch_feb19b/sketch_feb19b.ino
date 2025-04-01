#include <SPI.h>
#include <WiFi.h>
IPAddress ip(192, 168, 248, 108);
char ssid[] = "Meet's OnePlus";
char password[] ="123456789";
char b[100];
int k;
WiFiServer server(5555);
void setup()
{
Serial.begin(9600);
Serial.print("Attempting to connect to Network named: ");
Serial.println(ssid);
WiFi.config(ip);
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
server.begin();
}
void loop()
{
WiFiClient client = server.available();
if((Serial.available()))
{
memset(b,0,100);
Serial.readBytes(b,100);
Serial.println(b);
server.println(b);
delay(500);
}
if(client.available())
{
char c=client.read();
Serial.print(c);
}
}
void printWifiStatus()
{
Serial.print("SSID: ");
Serial.println(WiFi.SSID());
IPAddress IP = WiFi.localIP();
Serial.print("IP Address: ");
Serial.println(IP);
}
