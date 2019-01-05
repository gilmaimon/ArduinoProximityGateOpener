//#define __INTERVALS_BEACON_IS_MAIN__
#ifdef __INTERVALS_BEACON_IS_MAIN__

#include <ArduinoComponents.h>
#include <ArduinoComponents/Components/RF/RC_Switch.h>
#include <ArduinoComponents/Helpers/TimedDispatch.h>

using namespace components;

class IntervalsBeacon : public Component {
  public:
    IntervalsBeacon(PinNumber ledPin, PinNumber txPin, int code,
                    unsigned intervalInMillis) : _led(ledPin),
                                                 _transmitter(txPin, RC_Switch::MODE::Mode_transmitter),
                                                 _code(code),
                                                 _intervalInMillis(intervalInMillis),
                                                 _dispatch(this, [&]() {
                                                     transmit();
                                                     startBeaconTimedDisptach();
                                                 }){

        RegisterChild(_dispatch);
        startBeaconTimedDisptach();
    }

  private:
    LED _led;
    TimedDispatch _dispatch;
    const unsigned _intervalInMillis;
    const int _code;
    RC_Switch _transmitter;

    void privateLoop() {}

    void startBeaconTimedDisptach(){
        _dispatch.disptach_delayed(_intervalInMillis);
    }

    void transmit() {
        indicate_transmittion();
        for (int i = 0; i < 1; i++) {
            _transmitter.send(_code);
        }
        _led.off();
    }

    void indicate_transmittion() {
        for (int i = 0; i < 3; i++) {
            _led.on();
            delay(20);
            _led.off();
            delay(20);
        }
        _led.on();
    }
};

void setup() {
    IntervalsBeacon bcn(13, 12, 11111, 1 * 1000);
    while (true) bcn.loop();
}

void loop() {}

#endif