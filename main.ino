#include <Arduino.h>
#include <string>
#include <functional>
#include <iostream>

// int incomingByte = 0;

class Controller
{
public:
    int pin;
    uint8_t mode;
    bool state;

    typedef std::function<void()> Callback;

    inline void setPinMode()
    {
        pinMode(pin, mode);
    }

    inline void setInitialValues()
    {
        state = digitalRead(pin);
    }

    Controller(int pinValue, uint8_t pinMode, bool stateValue = LOW)
    {
        pin = pinValue;
        mode = pinMode;
        state = stateValue;

        setPinMode();
        setInitialValues();
    }
};

// Button
class Button : public Controller
{

    // Time
    unsigned long time, prevTime = 0;

    // isPressed State
    bool isPressed, prevIsPressed;

public:
    int debounceTime = 30;

    inline void pressed(Callback callback = []() {})
    {
        time = millis();
        isPressed = digitalRead(pin);

        // Remove jitter
        if (time > prevTime + debounceTime)
        {
            prevTime = time;

            if (isPressed != prevIsPressed)
            {

                // isPressed:
                if (mode == INPUT_PULLDOWN && !isPressed)
                    callback();
                if (mode == INPUT_PULLUP && isPressed)
                    callback();

                // Toggled State:
                if (isPressed)
                    state = !state;
            }

            prevIsPressed = isPressed;
        }
    }

    Button(int pin, uint8_t mode) : Controller(pin, mode) {}
};

// Switch
class Switch : public Controller
{

public:
    inline void changed(Callback callback)
    {
        callback();
    }

    Switch(int pinValue, uint8_t modeValue) : Controller(pinValue, modeValue) {}
};

class Led : Controller
{
public:
    typedef std::function<void()> Callback;

    inline void on()
    {
        digitalWrite(pin, HIGH);
    }
    inline void off()
    {
        digitalWrite(pin, LOW);
    }

    inline void turn(
        bool stateValue, Callback callback = []() {})
    {
        state = stateValue;
        state ? on() : off();
        callback();
    }

    Led(int pin, bool initState = LOW) : Controller(pin, OUTPUT, initState)
    {
        turn(initState);
    }
};

// Init
Button navButton(5, INPUT_PULLDOWN);
Led navButtonLed(4, HIGH);

Button launchButton(10, INPUT_PULLDOWN);
Led launchButtonLed(9, HIGH);

Switch lidSwitch(8, INPUT_PULLDOWN);

void setup()
{
    Serial.begin(115200);
}

void loop()
{

    launchButton.pressed([]()
                         { launchButtonLed.turn(launchButton.state, []()
                                                {
      Serial.println("Launch Led Changed");
      Serial.println(launchButton.pin); }); });

    navButton.pressed([]()
                      { navButtonLed.turn(navButton.state, []()
                                          {
      Serial.println("Navigation Led Changed");
      Serial.println(navButton.pin); }); });

    // lidSwitch.changed([]() {
    //   Serial.println("Switch changed");
    // });
}
