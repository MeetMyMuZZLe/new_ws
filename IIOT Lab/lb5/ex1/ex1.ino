#include <SPI.h>
#include <AIR430BoostETSI.h>

unsigned char txEntry[] = {'E', 'N', 'T', 'R', 'Y', '\0'};
unsigned char txExit[] = {'E', 'X', 'I', 'T', '\0'};
int a;
int b;

void printTxEntry()
{
    Serial.print("TX (DATA): ");
    Serial.println((char*)txEntry);
}

void printTxExit()
{
    Serial.print("TX (DATA): ");
    Serial.println((char*)txExit);
}

void setup()
{
    Radio.begin(0x01, CHANNEL_1, POWER_MAX);
    Serial.begin(9600);
    
    pinMode(GREEN_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);
    pinMode(PUSH1, INPUT_PULLUP);
    pinMode(PUSH2, INPUT_PULLUP);
    
    attachInterrupt(PUSH1, entry, FALLING);
    attachInterrupt(PUSH2, exit, FALLING);
}

void entry()
{
    a=1;
}

void exit()
{
    b=1;
}

void loop()
{
    if (a==1)
    {
        a=0;
        Radio.transmit(ADDRESS_BROADCAST, txEntry, 6);
        printTxEntry();
        digitalWrite(RED_LED, HIGH);
        delay(500);
        digitalWrite(RED_LED, LOW);
    }
    
    if (b==1)
    {
        b=0;
        Radio.transmit(ADDRESS_BROADCAST, txExit, 6);
        printTxExit();
        digitalWrite(GREEN_LED, HIGH);
        delay(500);
        digitalWrite(GREEN_LED, LOW);
    }
}
