#ifndef MOMENTARY_BUTTON_INCLUDED
#define MOMENTARY_BUTTON_INCLUDED

#ifdef ARDUINO
  #include "Arduino.h"
  #if (0)
    #include <../PinChangeInt/PinChangeInt.h>   //Still need to #include in the main .ino code too
    #define ACTIVE_PINS 20
    int8_t (*aInterrupt)(uint8_t, PCIntvoidFuncPtr, int) = PCintPort::attachInterrupt;
  #endif
#else
  #include "arduPi.h"
  #if (0)
    #define ACTIVE_PINS 28
    void (*aInterrupt)(int, void (*f)(), Digivalue) = attachInterrupt;
    #define CHANGE BOTH
  #endif
#endif

/**
 * This button library allows your design to receive input from push buttons and 
 * bump switches. 
 */
class MomentaryButton {

    int isDown;
    int pin;


    private:
    uint8_t _pin;           //arduino pin number
    uint8_t _puEnable;      //internal pullup resistor enabled
    uint8_t _invert;        //if 0, interpret high state as pressed, else interpret low state as pressed
    uint8_t _state;         //current button state
    uint8_t _lastState;     //previous button state
    uint8_t _changed;       //state changed since last read
    uint8_t _wasPress;       //keeps track if button has been pressed since last check
    uint8_t _wasRelease;     //keeps track if button has been released since last check
    long _time;         //time of current state (all times are in ms)
    uint32_t _lastChange;   //time of last state change
    uint32_t _dbTime;       //debounce time

    /**
     * This function returns the state of the button, 1==pressed, 0==released,     
     * does debouncing, captures and maintains times, previous states, etc. 
     */
    uint8_t read()
    {
        static long ms;
        static uint8_t pinVal;

        ms = millis();
        pinVal = digitalRead(_pin);
        if (_invert != 0) pinVal = !pinVal;
        if (ms - _lastChange < _dbTime) {
            _time = ms;
            _changed = 0;
            return _state;
        }
        else {
            _lastState = _state;
            _state = pinVal;
            _time = ms;
            if (_state != _lastState)   {
                _lastChange = ms;
                _changed = 1;
            }
            else {
                _changed = 0;
            }
            return _state;
        }
    }


    public:
    /**
     * \brief Constructor
     *
     * Creates a variable of type MomentaryButton. The pin parameter is the 
     * hardware pin connecting the MomentaryButton to the microcontroller.
     *
     * If you use the sketch that came with your robot, you won't need call this.
     */
    MomentaryButton(int pin): pin(pin) {
        _pin = pin;
        _puEnable = 1;
        _invert = 1;
        _dbTime = 200;
        pinMode(_pin, INPUT);
        if (_puEnable != 0)
            digitalWrite(_pin, HIGH);       //enable pullup resistor
        _state = digitalRead(_pin);
        if (_invert != 0) _state = !_state;
        _time = millis();
        _lastState = _state;
        _changed = 0;
        _lastChange = _time;

        #if (0)
        if(pin >= 0 && pin < ACTIVE_PINS)
            ptr[pin] = this;
        #endif
    }

   /**
    *
     * \brief Checks if the button is pressed. 
     *
     * Returns true if the button is pressed, otherwise false. 
     */
    bool isPressed()
    {
        read();
        _wasPress = 0; //resets counter for if the button was pressed
        return _state == 1 ;
    }

#ifndef GTRON_ARDUINO_SKIP
    /**
     * Checks if the button is not pressed. Returns true if the 
     * button is not pressed, otherwise false. 
     */
    bool isReleased()
    {
        read();
        _wasRelease = 0; //resets counter for if the button was released
        return _state == 0;
    }
#endif
    
   /**
    * \brief Wait for a press
    *
    * Pauses program execution until the button or bump 
    * switch is pressed.
    */
    void waitUntilPressed ()
    {
        while (read() != 1);
    }

    /**
    * \brief Wait for release
    *
    * Pauses program execution until the button or bump 
    * switch is released.
    */
    void waitUntilReleased ()
    {
        while (read() == 1);
    }

#if (0)
   /**
     * This function does not cause the button to be read.                  
     * Do not use
     */
    bool wasPressed()
    {
        bool rv = (bool)_wasPress;
        _wasPress = 0;  //reset counter
        return rv;
    }

   /**
     * This function does not cause the button to be read. 
     * Do not use
     */
    bool wasReleased()
    {
        bool rv = (bool)_wasRelease;
        _wasRelease = 0;    //reset counter
        return rv;
    }

   /**
     * This function checks if the button has been pressed for t milliseconds.
     * If the button has been pressed for t milliseconds, then this function 
     * returns true. Otherwise, it returns false.
     */
    long pressedFor(long t)
    {
        read();
        return (_state == 1 && _time - _lastChange >= t) ? 1 : 0;
    }

   /**
     * This function checks if the button has not been pressed for t 
     * milliseconds since the last it was pressed. If the button the previous 
     * condition, is true, then this function returns true. Otherwise, it 
     * returns false.
     */
    long releasedFor(long t)
    {
        read();
        return (_state == 0 && _time - _lastChange >= t) ? 1 : 0;
    }
    /** Sets the debounce time for when reading the button */
    void setDB(uint32_t ms)
    {
        _dbTime = ms;
    }


   /**
     * This function returns the time button ellapsed from when the button was
     * last pressed or released.
     */
    long lastChange()
    {
        return _lastChange;
    }
#endif

#ifndef GTRON_ARDUINO_SKIP
   /**
     * This function prepares the MomentaryButton to operate. This function 
     * should be called within the setup function of an Arduino program. 
     * Otherwise, unpredictable errors may occur while using this library.
     */
    void setup() {
#if (0)
        switch (_pin)
        {
            case 3:
                aInterrupt(_pin, pin3rupt, CHANGE); break;
            case 4:
                aInterrupt(_pin, pin4rupt, CHANGE); break;
            case 5:
                aInterrupt(_pin, pin5rupt, CHANGE); break;
            case 6:
                aInterrupt(_pin, pin6rupt, CHANGE); break;
            case 8:
                aInterrupt(_pin, pin8rupt, CHANGE); break;
            case 9:
                aInterrupt(_pin, pin9rupt, CHANGE); break;
            case 10:
                aInterrupt(_pin, pin10rupt, CHANGE); break;
            case 11:
                aInterrupt(_pin, pin11rupt, CHANGE); break;
            case 12:
                aInterrupt(_pin, pin12rupt, CHANGE); break;
            case 13:
                aInterrupt(_pin, pin13rupt, CHANGE); break;
#ifdef ARDUINO
            case A0:
                aInterrupt(_pin, pinA0rupt, CHANGE); break;
            case A1:
                aInterrupt(_pin, pinA1rupt, CHANGE); break;
            case A2:
                aInterrupt(_pin, pinA2rupt, CHANGE); break;
            case A3:
                aInterrupt(_pin, pinA3rupt, CHANGE); break;
            case A4:
                aInterrupt(_pin, pinA4rupt, CHANGE); break;
            case A5:
                aInterrupt(_pin, pinA5rupt, CHANGE); break;
            #else
            case 0: aInterrupt(_pin, pin0rupt, CHANGE); break;
            case 1: aInterrupt(_pin, pin1rupt, CHANGE); break;
            case 2: aInterrupt(_pin, pin2rupt, CHANGE); break;
            case 14: aInterrupt(_pin, pin14rupt, CHANGE); break;
            case 15: aInterrupt(_pin, pin15rupt, CHANGE); break;
            case 16: aInterrupt(_pin, pin16rupt, CHANGE); break;
            case 17: aInterrupt(_pin, pin17rupt, CHANGE); break;
            case 18: aInterrupt(_pin, pin18rupt, CHANGE); break;
            case 19: aInterrupt(_pin, pin19rupt, CHANGE); break;
            case 20: aInterrupt(_pin, pin20rupt, CHANGE); break;
            case 21: aInterrupt(_pin, pin21rupt, CHANGE); break;
            case 22: aInterrupt(_pin, pin22rupt, CHANGE); break;
            case 23: aInterrupt(_pin, pin23rupt, CHANGE); break;
            case 24: aInterrupt(_pin, pin24rupt, CHANGE); break;
            case 25: aInterrupt(_pin, pin25rupt, CHANGE); break;
            case 26: aInterrupt(_pin, pin26rupt, CHANGE); break;
            case 27: aInterrupt(_pin, pin27rupt, CHANGE); break;
#endif
            default:
                break;
        }
#endif
    }
#endif
    

#if (0)
    private:
    //Called when there is an interrupt, the button makes a note whether it
    //was pressed or released.
    void myrupt()
    {
        if (millis() - _lastChange > _dbTime) {
            read();

            if(_state == 0)
            {
                _wasRelease = 1;
            }
            else
            {
                _wasPress = 1;
            }
        }
    }

    /*----------------------------------------------------------------------*
     * pin<number>rupt calls the corresponding pin's interrupt method       *
     *----------------------------------------------------------------------*/
    static void pin3rupt()
    {
        ptr[3]->myrupt();
    }

    static void pin4rupt()
    {
        ptr[4]->myrupt();
    }

    static void pin5rupt()
    {
        ptr[5]->myrupt();
    }

    static void pin6rupt()
    {
        ptr[6]->myrupt();
    }

    static void pin8rupt()
    {
        ptr[8]->myrupt();
    }

    static void pin9rupt()
    {
        ptr[9]->myrupt();
    }

    static void pin10rupt()
    {
        ptr[10]->myrupt();
    }

    static void pin11rupt()
    {
        ptr[11]->myrupt();
    }

    static void pin12rupt()
    {
        ptr[12]->myrupt();
    }

    static void pin13rupt()
    {
        ptr[13]->myrupt();
    }

#ifdef ARDUINO
    static void pinA0rupt() {
        ptr[A0]->myrupt();
    }

    static void pinA1rupt() {
        ptr[A1]->myrupt();
    }

    static void pinA2rupt() {
        ptr[A2]->myrupt();
    }

    static void pinA3rupt() {
        ptr[A3]->myrupt();
    }

    static void pinA4rupt() {
        ptr[A4]->myrupt();
    }

    static void pinA5rupt() {
        ptr[A5]->myrupt();
    }
#else
    static void pin0rupt() { ptr[0]->myrupt(); }
    static void pin1rupt() { ptr[1]->myrupt(); }
    static void pin2rupt() { ptr[2]->myrupt(); }
    static void pin3rupt() { ptr[3]->myrupt(); }
    static void pin14rupt() { ptr[14]->myrupt(); }
    static void pin15rupt() { ptr[15]->myrupt(); }
    static void pin16rupt() { ptr[16]->myrupt(); }
    static void pin17rupt() { ptr[17]->myrupt(); }
    static void pin18rupt() { ptr[18]->myrupt(); }
    static void pin19rupt() { ptr[19]->myrupt(); }
    static void pin20rupt() { ptr[20]->myrupt(); }
    static void pin21rupt() { ptr[21]->myrupt(); }
    static void pin22rupt() { ptr[22]->myrupt(); }
    static void pin23rupt() { ptr[23]->myrupt(); }
    static void pin24rupt() { ptr[24]->myrupt(); }
    static void pin25rupt() { ptr[25]->myrupt(); }
    static void pin26rupt() { ptr[26]->myrupt(); }
    static void pin27rupt() { ptr[27]->myrupt(); }
    static MomentaryButton *ptr[ACTIVE_PINS];
#endif

#endif
};
//MomentaryButton *MomentaryButton::ptr[ACTIVE_PINS];  //Outside declaration needed for ptr array to work
#endif


