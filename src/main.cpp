#include <Arduino.h>


#define BUTTON0_PIN 3
#define BUTTON1_PIN 4
#define JBUTTON0_PIN 5
#define JBUTTON1_PIN 6

#define STICK0_X_PIN A0
#define STICK0_Y_PIN A1
#define STICK1_X_PIN A2
#define STICK1_Y_PIN A3

#define SYNC_MSG "\nSYNC"

#define MAX_BUTTONS 16
#define MAX_STICKS 2

typedef struct 
{
  uint8_t pin;
  bool state;
  uint16_t debounceCycles;
  uint16_t debounceCounter;
} Button;

typedef struct 
{
  uint16_t xAxisVal;
  uint16_t yAxisVal;
  Button jButton;
  uint8_t id;
  uint8_t xPin;
  uint8_t yPin;
} Joystick;



Joystick stick0 = {
  .xAxisVal = 0,
  .yAxisVal = 0,
  .jButton = {
    .pin = JBUTTON0_PIN,
    .state = HIGH,
    .debounceCycles = 50,
    .debounceCounter = 50
  },
  .id = 0,
  .xPin = STICK0_X_PIN,
  .yPin = STICK0_Y_PIN
};


Joystick stick1 = {
  .xAxisVal = 0,
  .yAxisVal = 0,
  .jButton = {
    .pin = JBUTTON1_PIN,
    .state = HIGH,
    .debounceCycles = 50,
    .debounceCounter = 50
  },
  .id = 1,
  .xPin = STICK1_X_PIN,
  .yPin = STICK1_Y_PIN
};


Button but0 = {
  .pin = BUTTON0_PIN,
  .state = HIGH,
  .debounceCycles = 50,
  .debounceCounter = 50
};

Button but1 = {
  .pin = BUTTON1_PIN,
  .state = HIGH,
  .debounceCycles = 50,
  .debounceCounter = 50
};


Button* buttons[] = {&but0, &but1, &stick0.jButton, &stick1.jButton, NULL};
Joystick* sticks[] = {&stick0, &stick1, NULL};


void updateStickAxes(Joystick* stickPtr)
{
  stickPtr->xAxisVal = analogRead(stickPtr->xPin);
  stickPtr->yAxisVal = analogRead(stickPtr->yPin);
}


bool readButtonState(Button* butPtr)
{
  return digitalRead(butPtr->pin);
}


void updateButtonState(Button* butPtr)
{
  butPtr->state = !butPtr->state;
}


bool buttonValChanged(Button* butPtr, bool state)
{
  if (butPtr->state != state)
  {
    return true;
  }
  return false;
}



void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(BUTTON0_PIN, INPUT_PULLUP);
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(JBUTTON0_PIN, INPUT_PULLUP);
  pinMode(JBUTTON1_PIN, INPUT_PULLUP);

  Serial.println("CrudePlayPad started...");
}


void loop() 
{
  // Serial.write(SYNC_MSG);
  Serial1.write(SYNC_MSG);
  uint16_t buttonsOut = 0;
  delay(10);

  /* Read button states */
  Serial.println("-----------------------------------");
  for (int i = 0; i < MAX_BUTTONS && buttons[i] != NULL; i++)
  {
    Button* butPtr = buttons[i];
    if (butPtr->debounceCounter == 0)
    {
      butPtr->debounceCounter = butPtr->debounceCycles;
    }

    /* We are not within a debounce cycle */
    if (butPtr->debounceCounter >= butPtr->debounceCycles)
    {
      /* Check if something has changed*/
      if (buttonValChanged(butPtr, readButtonState(butPtr)))
      {
        updateButtonState(butPtr);
        butPtr->debounceCounter--;
      }
    } 
    else // Debouncing
    {
      butPtr->debounceCounter--;
    }

    /* Update the outStr */
    buttonsOut |= (butPtr->state << i);  
    // Serial.print("Button on Pin: ");
    // Serial.print(butPtr->pin);
    // Serial.print("\tState:");
    // Serial.println(butPtr->state);
  }

  // Serial.write(buttonsOut);
  Serial1.write(buttonsOut);


  /* Read stick values */
  for (int i = 0; i < MAX_STICKS && sticks[i] != NULL; i++)
  {
    Joystick* stickPtr = sticks[i];
    updateStickAxes(stickPtr);
    // Serial.write(stickPtr->id);
    // Serial.write(stickPtr->xAxisVal);
    // Serial.write(stickPtr->yAxisVal);
    Serial1.write(stickPtr->id);
    Serial1.write(stickPtr->xAxisVal);
    Serial1.write(stickPtr->yAxisVal);
  }
}