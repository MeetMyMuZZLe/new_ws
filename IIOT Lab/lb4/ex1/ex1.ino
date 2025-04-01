#define LED2 BLUE_LED
int launchpadButtonState = 0; //variable
void setup()
{
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT); //setting state of LED as OUTPUT
  pinMode(PUSH1, INPUT_PULLUP); //Setting the of onboard Push button to PULLUP
  pinMode(PUSH2, INPUT_PULLUP); //Setting the of onboard Push button to PULLUP

} 
void loop()
{
  bool choice1 = digitalRead(PUSH1) == LOW;//president
  bool choice2 = digitalRead(PUSH2) == LOW;//counselor1 or counselor2 or counselor3 can press this button
  bool missilelaunch = choice1 && choice2;
  digitalWrite(LED2, missilelaunch);
}
