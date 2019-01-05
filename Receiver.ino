//#define __RECEIVER_IS_MAIN__
#ifdef __RECEIVER_IS_MAIN__

#include <ArduinoComponents.h>
#include <ArduinoComponents/Components/RF/RC_Switch.h>
#include <ArduinoComponents/Helpers/TimedDispatch.h>
#include <ArduinoComponents/Helpers/UniquePtr.h>
#include <EEPROM.h>

using namespace components;

#define BeaconsCode 11111
static const unsigned TIMEOUT_IN_MILLIS = 1000 * 9;
static const unsigned WAIT_BETWEEN_GATE_TOGGLES_IN_MILLIS = 1000 * 14;
static const unsigned RELAY_TRIGGER_DURATION_MILLIS = 360;
static const unsigned EXTRA_DELAY_AFTER_GATE_CLOSED_MILLIS = 1000 * 4;

class Receiver;
class ReciverState {
  public:
    ReciverState(Receiver *parent) : parent(parent) {}
    virtual void gotCode(int code) = 0;
    virtual void loop() {}
    virtual void printlnIdentity() = 0;
    virtual ~ReciverState() {}

  protected:
    Receiver *parent;
};

class Receiver : public Component {
  public:
    Receiver(PinNumber ledPin, PinNumber rxPin) : rcSwitch(rxPin, RC_Switch::MODE::MODE_Reciever),
                                                  indicateGateOpenLED(9),
                                                  inProgrammingMode(10),
                                                  relayLED(12),
                                                  inRemoteMode(11),
                                                  auxLed(8),
                                                  programSwitch(7, InputPull::Up),
                                                  state(nullptr) {

        // register rcSwitch so it's loop is called and incoming
        // codes will get recived and callbacked
        RegisterChild(rcSwitch);

        // load saved codes from persistent storage into `codes`
        loadCodesFromStorage();

        // Handle incoming codes, forward incoming codes to `state`
        rcSwitch.onCode([&](int code) {
            delay(100);
            Serial.print("Got code: ");
            Serial.println(code);
            indicateGotCode();
            state->gotCode(code);
        });
    }

    // switch to a new state, states will be deleted automatically
    // because of the use of UniquePtr
    void setState(UniquePtr<ReciverState> newState) {
        this->state = newState;
        Serial.print("Changing to ");
        this->state->printlnIdentity();
    }

    void closeGate() {
        Serial.println("Gate Closed");
        gateOpen = false;
        triggerRelay();
        for (int i = 0; i < (WAIT_BETWEEN_GATE_TOGGLES_IN_MILLIS + EXTRA_DELAY_AFTER_GATE_CLOSED_MILLIS) / 200; i++) {
            indicateGateOpenLED.toggle();
            delay(200);
        }

        //delay(WAIT_BETWEEN_GATE_TOGGLES_IN_MILLIS);
    }

    void openGate() {
        Serial.println("Gate Open");
        gateOpen = true;
        triggerRelay();
        for (int i = 0; i < WAIT_BETWEEN_GATE_TOGGLES_IN_MILLIS / 500; i++) {
            indicateGateOpenLED.toggle();
            delay(500);
        }
        //delay(WAIT_BETWEEN_GATE_TOGGLES_IN_MILLIS);
    }

    // is code known ( = contained in `codes`)
    bool isCodeKnown(int code) {
        for (int i = 0; i < codes.nextCodeIdx; i++) {
            if (code == codes.remoteCodes[i])
                return true;
        }
        return false;
    }

    // locally register a new code and set it as a known code
    // then, backup those local changes to storage
    void registerCode(int code) {
        Serial.print("Registering ");
        Serial.println(code);
        codes.remoteCodes[codes.nextCodeIdx++] = code;
        backupCodesToStorage();
    }

    // store `codes` to persistent storage, usually EEPROM.
    void backupCodesToStorage() {
        EEPROM.put(EEPROM_ADREESS_PROGRAMMED_SIGNALS, codes);
    }

    // load codes from persistent storage, usually EEPROM.
    // codes are loaded into `codes`
    void loadCodesFromStorage() {
        EEPROM.get(EEPROM_ADREESS_PROGRAMMED_SIGNALS, codes);
    }

    bool isInProgrammingMode() {
        return programSwitch.isLow();
    }

    // a method for other classes to indicate is in remote mode or not.
    // mode is dependent on the inner state, so state must call this function
    void indicateRemoteMode(bool remoteMode) {
        inRemoteMode.set(remoteMode ? State_High : State_Low);
    }

    // Trigger the relay led (which should be connected to relay itself)
    // for a moment so a signal will be sent from the remote
    void triggerRelay() {
        relayLED.on();
        delay(RELAY_TRIGGER_DURATION_MILLIS);
        relayLED.off();
    }

  private:
    void indicateGotCode() {
        auxLed.on();
        delay(100);
        auxLed.off();
    }
    static const int EEPROM_ADREESS_PROGRAMMED_SIGNALS = 0;

    UniquePtr<ReciverState> state;

    LED indicateGateOpenLED;
    LED relayLED;
    LED inProgrammingMode;
    LED inRemoteMode;
    LED auxLed;

    RC_Switch rcSwitch;
    DigitalInput programSwitch;

    bool gateOpen = false;

    // A struct to store programmed codes, so they could be used instead of a remote
    struct Codes {
        int remoteCodes[20];
        int nextCodeIdx = 0;
    } codes;

    void privateLoop() {
        // proccess loop on inner state
        state->loop();

        // indicate if the gate is open (according to program's state)
        if (gateOpen)
            indicateGateOpenLED.on();
        else
            indicateGateOpenLED.off();

        // indicate if in programming mode (using a phsical toggle/button)
        inProgrammingMode.set(isInProgrammingMode() ? State_High : State_Low);
    }
};

// This class represents a state in which the gate is idle
// a *known* remote code will make the gate go to OpenUntilRemoteMessage - wich means
// it will stay open until the next remote code (not beacon).
// a beacon code will make the gate go to OpenedAndWaitingForTimeout, opening the gate
// and closing it only after the next message or timeout
// in case of an *unkown* code, if the gate is in programming mode, it will program
// the new code into the system
class IdleClosed : public ReciverState {
  public:
    IdleClosed(Receiver *parent) : ReciverState(parent) {
        parent->indicateRemoteMode(false);
    }

    virtual void gotCode(int code);
    void printlnIdentity() {
        Serial.println("IdleClosed");
    }
};

// This class represents a state in which the gate can only be opened using a remote.
// beacons will not trigger anything and only a remote will make it move to IdleState
class ClosedUntilRemoteMessage : public ReciverState {
  public:
    ClosedUntilRemoteMessage(Receiver *parent) : ReciverState(parent) {
        parent->indicateRemoteMode(true);
    }

    // when a code is recived, if it is a known (programmed code), move to
    virtual void gotCode(int code) {
        Serial.println("- Inside: ClosedUntilRemoteMessage gotCode");
        if (parent->isCodeKnown(code)) {
            Serial.println("Message is remote, moving to Idle");
            parent->setState(UniquePtr<ReciverState>(new IdleClosed(parent)));
        }
    }
    void printlnIdentity() {
        Serial.println("ClosedUntilRemoteMessage");
    }
};

// This mode represent a state in which the gate was opened using a beacon.
// in this state, the gate will be closed either by a timeout or by a known remote code
class OpenedAndWaitingForTimeout : public ReciverState {
  public:
    OpenedAndWaitingForTimeout(Receiver *parent) : ReciverState(parent) {
        parent->indicateRemoteMode(false);
        timeLeft = TIMEOUT_IN_MILLIS;
        lastTime = millis();
    }
    virtual void gotCode(int code) {
        Serial.println("- Inside: OpenedAndWaitingForTimeout gotCode");
        if (code == BeaconsCode) {
            Serial.println("Code is beacon. Reseting timeout.");
            timeLeft = TIMEOUT_IN_MILLIS;
            lastTime = millis();
        }
        else if (parent->isCodeKnown(code)) {
            Serial.println("Code is Remote! closing.");
            parent->closeGate();
            parent->setState(UniquePtr<ReciverState>(new ClosedUntilRemoteMessage(parent)));
        }
    }
    virtual void loop() {
        timeLeft -= millis() - lastTime;
        lastTime = millis();
        if (timeLeft <= 0) {
            Serial.println("Timed-Out");
            parent->closeGate();
            parent->setState(UniquePtr<ReciverState>(new IdleClosed(parent)));
        }
    }
    void printlnIdentity() {
        Serial.println("OpenedAndWaitingForTimeout");
    }

  private:
    short timeLeft;
    short lastTime;
};

// This class represent a state in which the gate was opened by a remote, so
// it will be closed only by another remote trigger, beacons wont do anything
class OpenUntilRemoteMessage : public ReciverState {
  public:
    OpenUntilRemoteMessage(Receiver *parent) : ReciverState(parent) {
        parent->indicateRemoteMode(true);
    }
    virtual void gotCode(int code) {
        Serial.println("- Inside: OpenUntilRemoteMessage gotCode");
        if (parent->isCodeKnown(code)) {
            parent->closeGate();
            parent->setState(UniquePtr<ReciverState>(new IdleClosed(parent)));
        }
    }

    void printlnIdentity() {
        Serial.println("OpenUntilRemoteMessage");
    }
};

void IdleClosed::gotCode(int code) {
    Serial.println("- Inside: IdleClosed gotCode");
    if (code == BeaconsCode) {
        parent->openGate();
        parent->setState(UniquePtr<ReciverState>(new OpenedAndWaitingForTimeout(parent)));
    }
    else if (parent->isCodeKnown(code)) {
        Serial.println("Code is known");
        parent->openGate();
        parent->setState(UniquePtr<ReciverState>(new OpenUntilRemoteMessage(parent)));
    }
    else if (parent->isInProgrammingMode()) {
        Serial.println("In programming, registering");
        parent->registerCode(code);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Start");
    Receiver rcv(13, 0);
    rcv.setState(UniquePtr<ReciverState>(new IdleClosed(&rcv)));
    while (true)
        rcv.loop();
}

void loop() {}

#endif