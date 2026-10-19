/*
  ============================================================
  MTA Prototype Firmware v0.3.10
  ============================================================

  Based on v0.3.5.
  NEW v0.3.10: per-device HTTP authorization via X-MTA-Key; local secrets header.
  NEW v0.3.7: persistent NVS event journal + USB serial test commands.
  NEW v0.3.8: FIFO HTTP event sender, NTP timestamps, explicit TEST archival.
  NEW v0.3.6: Wi-Fi heartbeat in a separate FreeRTOS task.
  Network work never runs in the mechanical control loop.

  NEW v0.3.5:
  - SERVICE_DISPENSING state
  - Short GPIO17 press: normal service dispense after checks
  - GPIO17 hold >= 3 s: forced service dispense
  - 7 s timeout starts at actual motor start
  - GPIO19/GPIO21 stop on a new valid event
  - Already-active sensors must return normal before re-arming
  - Re-press GPIO17 after release: manual STOP in forced mode
  - Normal service stop returns directly to SERVICE
  - Closing service door during SERVICE_DISPENSING -> ERROR

  NOT IMPLEMENTED YET:
  - Operator notifications
  - Real PaymentManager
*/

#include <time.h>
#include <cstring>

// Forward declarations for Arduino IDE automatic function prototypes.
struct StoredEvent;
struct EventSendRequest;
struct EventSendResult;
struct HeartbeatSnapshot;

// ============================================================
// MACHINE STATES
// ============================================================

enum MachineState {
  IDLE,
  WAIT_PAYMENT,
  DISPENSING,
  SUCCESS,
  EMPTY,
  SERVICE,
  SERVICE_DISPENSING,
  WAIT_CLEAR_DISPENSER,
  WAIT_SHUTTER_CLOSE,
  ERROR_STATE
};


// ============================================================
// PIN MAP
// ============================================================

const int PIN_WANT_WINDOW      = 14;
const int PIN_ARROW            = 22;
const int PIN_WANT_BUTTON_LED  = 23;

const int PIN_LED_READY        = 25;
const int PIN_LED_DISPENSE     = 26;
const int PIN_LED_TAKE         = 27;
const int PIN_LED_EMPTY        = 32;
const int PIN_LED_ERROR        = 33;

const int PIN_WANT_BUTTON      = 18;

const int PIN_TRAY_SENSOR      = 34;
const int PIN_NEXT_CAN         = 35;
const int PIN_CAN_EXIT         = 21;
const int PIN_SHUTTER          = 19;
const int PIN_SERVICE_DOOR     = 16;
const int PIN_MANUAL_DISPENSE  = 17;

const int PIN_MOTOR_RELAY      = 13;

const int PIN_PAY_OK           = 39;
const int PIN_PAY_FAIL         = 15;
const int PIN_REFUND_LED       = 5;

const int PIN_BUZZER           = 4;


// ============================================================
// CONFIGURATION
// ============================================================

const bool MOTOR_ENABLED = true;

const unsigned long DISPENSE_TIMEOUT       = 7000;
const unsigned long EMPTY_CHECK_TIMEOUT    = 15000;

// NEW v0.3.4
const unsigned long SHUTTER_RECOVERY_TIMEOUT = 15000;

// NEW v0.3.5
const unsigned long MANUAL_HOLD_TIMEOUT = 3000;


// ============================================================
// VERIFIED HARDWARE LEVELS
// ============================================================

const int TRAY_OCCUPIED_LEVEL    = LOW;
const int NEXT_CAN_PRESENT_LEVEL = LOW;

const int SERVICE_DOOR_OPEN_LEVEL = HIGH;

const int SHUTTER_DOWN_LEVEL   = HIGH;
const int SHUTTER_RAISED_LEVEL = LOW;

const int CAN_EXIT_NORMAL_LEVEL  = LOW;
const int CAN_EXIT_TRIGGER_LEVEL = HIGH;

const int WANT_BUTTON_RELEASED_LEVEL = HIGH;
const int WANT_BUTTON_PRESSED_LEVEL  = LOW;

const int MANUAL_BUTTON_RELEASED_LEVEL = HIGH;
const int MANUAL_BUTTON_PRESSED_LEVEL  = LOW;


// ============================================================
// DEBOUNCED INPUT
// ============================================================

struct DebouncedInput {
  int pin;
  int stableState;
  int lastRawState;
  unsigned long lastChangeTime;
};

const unsigned long DEBOUNCE_TIME = 40;

DebouncedInput wantButton;
DebouncedInput manualDispense;
DebouncedInput shutter;
DebouncedInput serviceDoor;
DebouncedInput canExit;
DebouncedInput traySensor;
DebouncedInput payOK;
DebouncedInput payFail;


// ============================================================
// GLOBAL STATE
// ============================================================

MachineState currentState = ERROR_STATE;


// Context for WAIT_CLEAR_DISPENSER

MachineState returnStateAfterClear = IDLE;


// NEW v0.3.4
// Context for WAIT_SHUTTER_CLOSE

MachineState returnStateAfterShutter = IDLE;


// ============================================================
// TIMERS
// ============================================================

unsigned long dispenseStartTime = 0;
unsigned long emptyCheckStartTime = 0;

// NEW v0.3.4
unsigned long shutterRecoveryStartTime = 0;

// NEW v0.3.5
unsigned long manualPressStartTime = 0;


// ============================================================
// DISPENSE DATA
// ============================================================

bool exitSeen    = false;
bool shutterSeen = false;
bool traySeen    = false;

bool motorRunning = false;

bool refundRequested = false;


// ============================================================
// SERVICE DISPENSE DATA - NEW v0.3.5
// ============================================================

bool serviceDispenseForced = false;
bool serviceExitArmed = false;
bool serviceShutterArmed = false;
bool serviceStopArmed = false;
bool manualPressPending = false;


// ============================================================
// FORWARD DECLARATIONS
// ============================================================

void runReadinessCheck(MachineState requestedState);
void checkDispenseSuccess();
void recordStateTransition(MachineState oldState, MachineState newState);
bool appendEvent(const char* type, const char* data);


// ============================================================
// SENSOR SEMANTICS
// ============================================================

bool trayOccupied() {
  return digitalRead(PIN_TRAY_SENSOR)
         == TRAY_OCCUPIED_LEVEL;
}


bool nextCanPresent() {
  return digitalRead(PIN_NEXT_CAN)
         == NEXT_CAN_PRESENT_LEVEL;
}


bool serviceDoorOpen() {
  return digitalRead(PIN_SERVICE_DOOR)
         == SERVICE_DOOR_OPEN_LEVEL;
}


bool shutterRaised() {
  return digitalRead(PIN_SHUTTER)
         == SHUTTER_RAISED_LEVEL;
}


bool canExitAbnormal() {
  return digitalRead(PIN_CAN_EXIT)
         != CAN_EXIT_NORMAL_LEVEL;
}


// ============================================================
// OUTPUT MANAGER
// ============================================================

void motorOff() {
  digitalWrite(PIN_MOTOR_RELAY, LOW);
  motorRunning = false;
}


void motorOn() {

  if (!MOTOR_ENABLED) {
    motorOff();
    return;
  }

  digitalWrite(PIN_MOTOR_RELAY, HIGH);
  motorRunning = true;
}


void refundOff() {
  refundRequested = false;
  digitalWrite(PIN_REFUND_LED, LOW);
}


void requestRefund() {
  refundRequested = true;
  digitalWrite(PIN_REFUND_LED, HIGH);
}


void allCustomerIndicatorsOff() {

  digitalWrite(PIN_WANT_WINDOW, LOW);
  digitalWrite(PIN_ARROW, LOW);
  digitalWrite(PIN_WANT_BUTTON_LED, LOW);

  digitalWrite(PIN_LED_READY, LOW);
  digitalWrite(PIN_LED_DISPENSE, LOW);
  digitalWrite(PIN_LED_TAKE, LOW);
  digitalWrite(PIN_LED_EMPTY, LOW);
  digitalWrite(PIN_LED_ERROR, LOW);
}


void applyStateOutputs(MachineState state) {

  // Safe defaults

  motorOff();
  digitalWrite(PIN_BUZZER, LOW);

  allCustomerIndicatorsOff();


  switch (state) {

    case IDLE:

      refundOff();

      digitalWrite(PIN_WANT_WINDOW, HIGH);
      digitalWrite(PIN_WANT_BUTTON_LED, HIGH);
      digitalWrite(PIN_LED_READY, HIGH);

      break;


    case WAIT_PAYMENT:

      refundOff();

      digitalWrite(PIN_ARROW, HIGH);

      break;


    case DISPENSING:

      digitalWrite(PIN_LED_DISPENSE, HIGH);
      motorOn();

      break;


    case SUCCESS:

      digitalWrite(PIN_LED_TAKE, HIGH);

      break;


    case EMPTY:

      digitalWrite(PIN_LED_EMPTY, HIGH);

      break;


    case SERVICE:

      // Customer indicators OFF.
      // Shutter manipulation allowed.

      break;


    case SERVICE_DISPENSING:

      // Customer indicators OFF.
      // Motor starts explicitly in startServiceDispense().

      break;


    case WAIT_CLEAR_DISPENSER:

      // White "Take can"

      digitalWrite(PIN_LED_TAKE, HIGH);

      break;


    case WAIT_SHUTTER_CLOSE:

      // Protective warning

      digitalWrite(PIN_LED_ERROR, HIGH);
      digitalWrite(PIN_BUZZER, HIGH);

      break;


    case ERROR_STATE:

      digitalWrite(PIN_LED_ERROR, HIGH);

      break;
  }
}


// ============================================================
// STATE MANAGER
// ============================================================

void changeState(MachineState newState) {

  if (currentState == newState) {
    return;
  }

  MachineState previousState = currentState;
  currentState = newState;
  recordStateTransition(previousState, newState);


  if (newState == DISPENSING) {

    exitSeen    = false;
    shutterSeen = false;
    traySeen    = false;

    dispenseStartTime = millis();
  }


  if (newState == EMPTY) {

    emptyCheckStartTime = millis();
  }


  // NEW v0.3.4
  //
  // 15-second timer starts ONLY when entering
  // WAIT_SHUTTER_CLOSE.
  //
  // It is not restarted while remaining in that state.

  if (newState == WAIT_SHUTTER_CLOSE) {

    shutterRecoveryStartTime = millis();
  }


  applyStateOutputs(currentState);
}


// ============================================================
// WAIT_CLEAR_DISPENSER ENTRY
// ============================================================

void enterWaitClearDispenser(
  MachineState returnState
) {

  returnStateAfterClear = returnState;

  changeState(WAIT_CLEAR_DISPENSER);
}


// ============================================================
// WAIT_SHUTTER_CLOSE ENTRY
// ============================================================

void enterWaitShutterClose(
  MachineState returnState
) {

  returnStateAfterShutter = returnState;

  changeState(WAIT_SHUTTER_CLOSE);
}


// ============================================================
// SERVICE DISPENSE MANAGER - NEW v0.3.5
// ============================================================

void finishServiceDispense() {

  motorOff();

  serviceDispenseForced = false;
  serviceExitArmed = false;
  serviceShutterArmed = false;
  serviceStopArmed = false;

  changeState(SERVICE);
}


void startServiceDispense(bool forced) {

  // Service dispense is legal only with service door open.
  if (!serviceDoorOpen()) {
    changeState(ERROR_STATE);
    return;
  }

  // Normal service dispense requires the four agreed
  // pre-start conditions. Forced mode bypasses them.
  if (!forced) {

    if (!nextCanPresent() ||
        shutterRaised() ||
        canExitAbnormal() ||
        trayOccupied()) {

      return;
    }
  }

  serviceDispenseForced = forced;

  // If a sensor is already active at forced start, it is not
  // armed. It must first return to normal; only its NEXT
  // activation may stop the motor.
  serviceExitArmed =
    !forced || !canExitAbnormal();

  serviceShutterArmed =
    !forced || !shutterRaised();

  // In forced mode the starting long press is still held.
  // Manual STOP becomes armed only after that press is released.
  serviceStopArmed = !forced;

  changeState(SERVICE_DISPENSING);

  // Seven seconds are counted from actual motor start.
  dispenseStartTime = millis();
  motorOn();
}


void processManualHoldTimer() {

  if (currentState != SERVICE) {
    return;
  }

  if (!manualPressPending) {
    return;
  }

  if (manualDispense.stableState
      != MANUAL_BUTTON_PRESSED_LEVEL) {
    return;
  }

  if ((millis() - manualPressStartTime)
      < MANUAL_HOLD_TIMEOUT) {
    return;
  }

  // Long press recognized. Its later release must not be
  // interpreted as a short press.
  manualPressPending = false;

  startServiceDispense(true);
}


void processServiceDispenseTimeout() {

  if (currentState != SERVICE_DISPENSING) {
    return;
  }

  if ((millis() - dispenseStartTime)
      < DISPENSE_TIMEOUT) {
    return;
  }

  // Service timeout is deliberately simple:
  // MOTOR OFF -> SERVICE. No ERROR.
  finishServiceDispense();
}


// ============================================================
// DISPENSE MANAGER
// ============================================================

void checkDispenseSuccess() {

  if (currentState != DISPENSING) {
    return;
  }


  if (shutterSeen && traySeen) {

    motorOff();

    changeState(SUCCESS);
  }
}


// ============================================================
// DISPENSE TIMEOUT
// ============================================================

void processDispenseTimeout() {

  if (currentState != DISPENSING) {
    return;
  }


  if ((millis() - dispenseStartTime)
      < DISPENSE_TIMEOUT) {
    return;
  }


  motorOff();


  // Final physical check

  if (shutterRaised()) {
    shutterSeen = true;
  }

  if (trayOccupied()) {
    traySeen = true;
  }


  // Product confirmed

  if (shutterSeen && traySeen) {

    changeState(SUCCESS);

    return;
  }


  // Product passed mechanism,
  // but tray not confirmed.
  // No refund.

  if (shutterSeen && !traySeen) {

    changeState(ERROR_STATE);

    return;
  }


  // Product passage not confirmed.
  // Refund.

  if (!shutterSeen) {

    requestRefund();

    changeState(ERROR_STATE);

    return;
  }
}


// ============================================================
// EMPTY TIMER
// ============================================================

void processEmptyTimer() {

  if (currentState != EMPTY) {
    return;
  }


  if ((millis() - emptyCheckStartTime)
      < EMPTY_CHECK_TIMEOUT) {
    return;
  }


  emptyCheckStartTime = millis();


  if (!nextCanPresent()) {
    return;
  }


  runReadinessCheck(IDLE);
}


// ============================================================
// NEW v0.3.4
// SHUTTER RECOVERY TIMER
// ============================================================

void processShutterRecoveryTimer() {

  if (currentState != WAIT_SHUTTER_CLOSE) {
    return;
  }


  if ((millis() - shutterRecoveryStartTime)
      < SHUTTER_RECOVERY_TIMEOUT) {
    return;
  }


  // 15 seconds elapsed.
  //
  // If shutter has physically returned to normal,
  // do not create an ERROR merely because a debounced
  // edge has not yet been processed.

  if (!shutterRaised()) {

    MachineState targetState =
      returnStateAfterShutter;

    runReadinessCheck(targetState);

    return;
  }


  // Shutter still raised after 15 seconds.
  //
  // applyStateOutputs(ERROR_STATE) will switch buzzer OFF
  // and leave red ERROR LED ON.
  //
  // Future:
  // JournalManager -> one journal event
  // Communications -> one operator notification

  changeState(ERROR_STATE);
}


// ============================================================
// READINESS CHECK
// ============================================================

void runReadinessCheck(
  MachineState requestedState
) {

  /*
      PRIORITY:

      1. Service door
      2. Shutter
      3. Can exit limit switch
      4. Next can
      5. Tray
      6. Requested state
  */


  // ----------------------------------------------------------
  // 1. SERVICE DOOR
  // ----------------------------------------------------------

  if (serviceDoorOpen()) {

    changeState(SERVICE);

    return;
  }


  // ----------------------------------------------------------
  // 2. SHUTTER
  // ----------------------------------------------------------

  if (shutterRaised()) {

    /*
       If readiness discovers an open shutter,
       enter protective recovery state.

       IMPORTANT:
       requestedState becomes the context to which
       we will try to return after recovery.
    */

    enterWaitShutterClose(
      requestedState
    );

    return;
  }


  // ----------------------------------------------------------
  // 3. CAN EXIT
  // ----------------------------------------------------------

  if (canExitAbnormal()) {

    changeState(ERROR_STATE);

    return;
  }


  // ----------------------------------------------------------
  // 4. NEXT CAN
  // ----------------------------------------------------------

  if (!nextCanPresent()) {

    changeState(EMPTY);

    return;
  }


  // ----------------------------------------------------------
  // 5. TRAY
  // ----------------------------------------------------------

  if (trayOccupied()) {

    enterWaitClearDispenser(
      requestedState
    );

    return;
  }


  // ----------------------------------------------------------
  // 6. EVERYTHING OK
  // ----------------------------------------------------------

  changeState(requestedState);
}


// ============================================================
// DEBOUNCE
// ============================================================

void initDebouncedInput(
  DebouncedInput &input,
  int pin
) {

  input.pin = pin;

  int value = digitalRead(pin);

  input.stableState = value;
  input.lastRawState = value;

  input.lastChangeTime = millis();
}


bool updateDebouncedInput(
  DebouncedInput &input,
  int &oldStableState
) {

  int rawState = digitalRead(input.pin);


  if (rawState != input.lastRawState) {

    input.lastRawState = rawState;
    input.lastChangeTime = millis();
  }


  if ((millis() - input.lastChangeTime)
      >= DEBOUNCE_TIME) {

    if (input.stableState != rawState) {

      oldStableState = input.stableState;
      input.stableState = rawState;

      return true;
    }
  }


  return false;
}


// ============================================================
// INPUT PROCESSING
// ============================================================

void processInputs() {

  int oldState;


  // ==========================================================
  // SERVICE DOOR
  // ==========================================================

  if (updateDebouncedInput(
        serviceDoor,
        oldState
      )) {

    if (serviceDoor.stableState
        == SERVICE_DOOR_OPEN_LEVEL) {


      // Service door has highest priority.
      //
      // Opening it from WAIT_SHUTTER_CLOSE
      // immediately ends the alarm because SERVICE
      // outputs have buzzer OFF.

      if (currentState == IDLE ||
          currentState == EMPTY ||
          currentState == ERROR_STATE ||
          currentState == WAIT_CLEAR_DISPENSER ||
          currentState == WAIT_SHUTTER_CLOSE) {

        changeState(SERVICE);

      }

      else if (currentState == SERVICE_DISPENSING) {

        // A service cycle already requires the door to be open.
        // If an OPEN edge is nevertheless received, stop safely
        // and return to SERVICE.
        motorOff();
        changeState(SERVICE);
      }

      else if (currentState != SERVICE) {

        changeState(ERROR_STATE);
      }

    }

    else {

      // Door closed after servicing.
      //
      // Start a completely new readiness check.

      if (currentState == SERVICE_DISPENSING) {

        // Closing the service door during active service
        // dispensing is an abnormal termination.
        motorOff();
        changeState(ERROR_STATE);
      }

      else if (currentState == SERVICE) {

        runReadinessCheck(IDLE);
      }
    }
  }


  // ==========================================================
  // SHUTTER
  // ==========================================================

  if (updateDebouncedInput(
        shutter,
        oldState
      )) {


    // ---------------------------------------------------------
    // SHUTTER RAISED
    // ---------------------------------------------------------

    if (shutter.stableState
        == SHUTTER_RAISED_LEVEL) {


      // =======================================================
      // DISPENSING
      //
      // Expected event.
      // =======================================================

      if (currentState == DISPENSING) {

        shutterSeen = true;

        motorOff();

        checkDispenseSuccess();
      }


      // =======================================================
      // SERVICE
      //
      // Allowed.
      // Technician may manipulate shutter.
      // =======================================================

      else if (currentState == SERVICE_DISPENSING) {

        if (serviceShutterArmed) {
          finishServiceDispense();
        }
      }


      else if (currentState == SERVICE) {

        // No action.
      }


      // =======================================================
      // ERROR
      //
      // ERROR is already blocking.
      // Do not replace it.
      // =======================================================

      else if (currentState == ERROR_STATE) {

        // No action.
      }


      // =======================================================
      // WAIT_SHUTTER_CLOSE
      //
      // Already handling this event.
      // Do not restart timer.
      // =======================================================

      else if (currentState
               == WAIT_SHUTTER_CLOSE) {

        // No action.
      }


      // =======================================================
      // WAIT_CLEAR_DISPENSER
      //
      // Shutter protection has higher priority.
      //
      // IMPORTANT:
      // Do NOT overwrite returnStateAfterClear.
      //
      // returnStateAfterShutter is set to the ORIGINAL
      // destination stored by WAIT_CLEAR_DISPENSER.
      // =======================================================

      else if (currentState
               == WAIT_CLEAR_DISPENSER) {

        enterWaitShutterClose(
          returnStateAfterClear
        );
      }


      // =======================================================
      // ALL OTHER STATES
      //
      // IDLE
      // WAIT_PAYMENT
      // SUCCESS
      // EMPTY
      //
      // Shutter opening is abnormal.
      // =======================================================

      else {

        enterWaitShutterClose(
          currentState
        );
      }
    }


    // ---------------------------------------------------------
    // SHUTTER LOWERED
    // ---------------------------------------------------------

    else {


      if (currentState == SERVICE_DISPENSING &&
          serviceDispenseForced) {

        // DOWN only re-arms a shutter that was already raised
        // at forced start. DOWN itself never stops the motor.
        serviceShutterArmed = true;
      }


      else if (currentState
          == WAIT_SHUTTER_CLOSE) {

        MachineState targetState =
          returnStateAfterShutter;


        // Do NOT blindly restore state.
        //
        // Full physical readiness check decides
        // what is actually possible now.

        runReadinessCheck(
          targetState
        );
      }
    }
  }


  // ==========================================================
  // CAN EXIT
  // ==========================================================

  if (updateDebouncedInput(
        canExit,
        oldState
      )) {

    if (canExit.stableState
        == CAN_EXIT_TRIGGER_LEVEL) {

      if (currentState == DISPENSING) {

        exitSeen = true;

        motorOff();
      }


      else if (currentState == SERVICE_DISPENSING) {

        if (serviceExitArmed) {
          finishServiceDispense();
        }
      }
    }

    else {

      if (currentState == SERVICE_DISPENSING &&
          serviceDispenseForced) {

        // NORMAL only re-arms GPIO21 if it was already active
        // at forced start. NORMAL itself does not stop motor.
        serviceExitArmed = true;
      }
    }
  }


  // ==========================================================
  // TRAY SENSOR
  // ==========================================================

  if (updateDebouncedInput(
        traySensor,
        oldState
      )) {


    // ---------------------------------------------------------
    // TRAY OCCUPIED
    // ---------------------------------------------------------

    if (traySensor.stableState
        == TRAY_OCCUPIED_LEVEL) {


      if (currentState == DISPENSING) {

        traySeen = true;

        checkDispenseSuccess();
      }


      else if (currentState == IDLE) {

        enterWaitClearDispenser(
          IDLE
        );
      }


      else if (currentState
               == WAIT_PAYMENT) {

        enterWaitClearDispenser(
          WAIT_PAYMENT
        );
      }


      /*
         SUCCESS:
             occupied tray expected

         SERVICE:
             allowed

         EMPTY:
             EMPTY retained

         WAIT_CLEAR_DISPENSER:
             already waiting

         WAIT_SHUTTER_CLOSE:
             higher-priority shutter condition retained

         ERROR:
             retained
      */
    }


    // ---------------------------------------------------------
    // TRAY CLEAR
    // ---------------------------------------------------------

    else {


      if (currentState == SUCCESS) {

        runReadinessCheck(IDLE);
      }


      else if (currentState
               == WAIT_CLEAR_DISPENSER) {

        MachineState targetState =
          returnStateAfterClear;

        returnStateAfterClear = IDLE;

        runReadinessCheck(
          targetState
        );
      }


      // If tray clears while WAIT_SHUTTER_CLOSE is active,
      // do nothing.
      //
      // Shutter protection remains dominant.
      // When shutter closes, full readiness check will see
      // the actual tray state.
    }
  }


  // ==========================================================
  // MANUAL SERVICE DISPENSE BUTTON - NEW v0.3.5
  // ==========================================================

  if (updateDebouncedInput(
        manualDispense,
        oldState
      )) {

    // ---------------------------------------------------------
    // BUTTON PRESSED
    // ---------------------------------------------------------

    if (manualDispense.stableState
        == MANUAL_BUTTON_PRESSED_LEVEL) {

      if (currentState == SERVICE) {

        // Do not start motor yet. First determine whether this
        // is a short press or a >=3 s forced command.
        manualPressPending = true;
        manualPressStartTime = millis();
      }

      else if (currentState == SERVICE_DISPENSING &&
               serviceDispenseForced &&
               serviceStopArmed) {

        // New press after release = immediate manual STOP.
        finishServiceDispense();
      }
    }


    // ---------------------------------------------------------
    // BUTTON RELEASED
    // ---------------------------------------------------------

    else {

      if (currentState == SERVICE &&
          manualPressPending) {

        unsigned long pressDuration =
          millis() - manualPressStartTime;

        manualPressPending = false;

        if (pressDuration < MANUAL_HOLD_TIMEOUT) {

          // Short press -> normal service dispense.
          startServiceDispense(false);
        }

        else {

          // Defensive fallback: normally the hold timer starts
          // forced dispense exactly at 3 s while still pressed.
          startServiceDispense(true);
        }
      }

      else if (currentState == SERVICE_DISPENSING &&
               serviceDispenseForced) {

        // Starting long press has now been released.
        // The NEXT new press may act as manual STOP.
        serviceStopArmed = true;
      }
    }
  }


  // ==========================================================
  // WANT BUTTON
  // ==========================================================

  if (updateDebouncedInput(
        wantButton,
        oldState
      )) {

    if (wantButton.stableState
        == WANT_BUTTON_PRESSED_LEVEL) {

      if (currentState == IDLE) {

        runReadinessCheck(
          WAIT_PAYMENT
        );
      }
    }
  }


  // ==========================================================
  // PAYMENT OK
  // ==========================================================

  if (updateDebouncedInput(
        payOK,
        oldState
      )) {

    if (payOK.stableState == HIGH) {

      if (currentState
          == WAIT_PAYMENT) {

        runReadinessCheck(
          DISPENSING
        );
      }


      /*
         Payment events received while
         WAIT_CLEAR_DISPENSER or WAIT_SHUTTER_CLOSE
         are intentionally NOT handled yet.

         This belongs to future PaymentManager
         with the real Vendista terminal.
      */
    }
  }


  // ==========================================================
  // PAYMENT FAIL
  // ==========================================================

  if (updateDebouncedInput(
        payFail,
        oldState
      )) {

    if (payFail.stableState == HIGH) {

      if (currentState
          == WAIT_PAYMENT) {

        runReadinessCheck(IDLE);
      }


      // Existing diagnostic behavior retained.

      else if (currentState
               == DISPENSING) {

        motorOff();

        runReadinessCheck(IDLE);
      }
    }
  }
}




// ============================================================
// EVENT MANAGER (v0.3.7, phase 1)
// NVS journal. Fixed 64-record capacity. No overwrite on full.
// Records are retained after reboot; delivery only after HTTP 200 confirmation.
// Serial Monitor commands: T = add test event, L = list events.
// ============================================================
#include <Preferences.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <esp_partition.h>
#include <time.h>
#include <cstring>
#include <cstdlib>

const uint16_t MTA_EVENT_CAPACITY = 64;
const uint32_t MTA_EVENT_MAGIC = 0x4D544137UL;

struct StoredEvent {
  uint32_t magic;
  uint32_t sequence;
  char eventId[40];
  char eventType[20];
  char data[80];
};

Preferences eventPrefs;
Preferences archivePrefs;
QueueHandle_t eventSendQueue = nullptr;
QueueHandle_t eventResultQueue = nullptr;
bool eventSendPending = false;
uint32_t eventLastAttempt = 0;
const uint32_t EVENT_RETRY_MS = 5000;
struct EventSendRequest { char payload[400]; char eventId[40]; };
struct EventSendResult { char eventId[40]; int httpCode; bool accepted; };
bool eventJournalReady = false;
uint16_t eventHead = 0;
uint16_t eventCount = 0;
uint32_t eventSequence = 0;

// v0.3.9 MVP: local count is persisted separately from the event queue.
// The initial 20 cans are a LOCAL test baseline, not an automatic server LOAD.
const uint16_t MTA_INITIAL_STOCK = 20;
const int MTA_TEST_PRICE = 1; // Test value in minor currency units; not a real tariff.
uint16_t localStock = MTA_INITIAL_STOCK;
const char* activeFault = "NONE";

void initializeLocalStock() {
  if (!eventJournalReady) return;
  if (!eventPrefs.isKey("stock")) {
    if (eventPrefs.putUShort("stock", MTA_INITIAL_STOCK) != sizeof(uint16_t)) {
      Serial.println("[MTA STOCK] ERROR: cannot initialize stock");
      return;
    }
  }
  localStock = eventPrefs.getUShort("stock", MTA_INITIAL_STOCK);
  Serial.printf("[MTA STOCK] Local stock=%u (server must be synchronized separately)\n", localStock);
}

void recordStateTransition(MachineState oldState, MachineState newState) {
  // Only normal paid dispensing counts as a sale. Service dispense is excluded.
  if (oldState == DISPENSING && newState == SUCCESS) {
    if (localStock == 0) {
      Serial.println("[MTA STOCK] CRITICAL: physical sale but local stock is zero");
      return;
    }
    time_t now = time(nullptr);
    char data[80];
    snprintf(data, sizeof(data), "{\"ts\":%lu}", (unsigned long)(now >= 1700000000 ? now : 0));
    if (appendEvent("SALE", data)) {
      // Pending-sale recovery on reboot uses an idempotent event sequence.
      // If power fails between these writes, recovery below reconciles stock.
      if (eventPrefs.putUShort("stock", localStock - 1) == sizeof(uint16_t)) {
        --localStock;
        Serial.printf("[MTA STOCK] SALE saved; local stock=%u\n", localStock);
      } else Serial.println("[MTA STOCK] CRITICAL: sale saved, stock write failed");
    } else Serial.println("[MTA EVT] CRITICAL: sale occurred but journal could not save it");
  }
  if (newState == ERROR_STATE && oldState != ERROR_STATE) {
    const char* fault = (oldState == DISPENSING) ? "DISPENSE_FAILURE" : "UNCLASSIFIED_ERROR";
    activeFault = fault;
    time_t now = time(nullptr);
    char data[80];
    snprintf(data, sizeof(data), "{\"ts\":%lu,\"code\":\"%s\"}",
             (unsigned long)(now >= 1700000000 ? now : 0), fault);
    if (!appendEvent("ERROR_SET", data)) Serial.println("[MTA EVT] CRITICAL: error not journaled");
  } else if (oldState == ERROR_STATE && newState != ERROR_STATE && strcmp(activeFault, "NONE") != 0) {
    time_t now = time(nullptr);
    char data[80];
    snprintf(data, sizeof(data), "{\"ts\":%lu,\"code\":\"%s\"}",
             (unsigned long)(now >= 1700000000 ? now : 0), activeFault);
    if (appendEvent("ERROR_CLEAR", data)) activeFault = "NONE";
    else Serial.println("[MTA EVT] CRITICAL: error clear not journaled");
  }
}


String eventSlotKey(uint16_t slot) {
  char key[8];
  snprintf(key, sizeof(key), "ev%02u", slot);
  return String(key);
}

bool readEventAt(uint16_t slot, StoredEvent &ev) {
  String key = eventSlotKey(slot);
  return eventPrefs.getBytesLength(key.c_str()) == sizeof(ev) &&
         eventPrefs.getBytes(key.c_str(), &ev, sizeof(ev)) == sizeof(ev) &&
         ev.magic == MTA_EVENT_MAGIC;
}

// Crash-safe removal: the durable marker is written first. Recovery finishes it.
bool finishAcknowledgedHead() {
  while (eventCount > 0) {
    String ackKey = String("ack") + String(eventHead);
    if (!eventPrefs.getBool(ackKey.c_str(), false)) break;
    uint16_t newHead = (eventHead + 1) % MTA_EVENT_CAPACITY;
    if (eventPrefs.putUShort("head", newHead) != sizeof(uint16_t)) return false;
    eventHead = newHead;
    if (eventPrefs.putUShort("count", eventCount - 1) != sizeof(uint16_t)) return false;
    --eventCount;
    eventPrefs.remove(ackKey.c_str());
  }
  return true;
}

bool acknowledgeHead(const char* id) {
  if (!eventJournalReady || eventCount == 0) return false;
  StoredEvent ev = {};
  if (!readEventAt(eventHead, ev) || strcmp(ev.eventId, id) != 0) return false;
  String key = String("ack") + String(eventHead);
  if (eventPrefs.putBool(key.c_str(), true) == 0) return false;
  return finishAcknowledgedHead();
}

void setupEventManager() {
  if (!eventPrefs.begin("mta-events", false)) {
    Serial.println("[MTA EVT] ERROR: cannot open NVS");
    return;
  }
  eventHead = eventPrefs.getUShort("head", 0);
  eventCount = eventPrefs.getUShort("count", 0);
  eventSequence = eventPrefs.getUInt("seq", 0);
  if (eventHead >= MTA_EVENT_CAPACITY || eventCount > MTA_EVENT_CAPACITY) {
    Serial.println("[MTA EVT] ERROR: invalid journal metadata; journal disabled");
    return;
  }
  eventJournalReady = true;
  if (!finishAcknowledgedHead()) {
    eventJournalReady = false;
    Serial.println("[MTA EVT] ERROR: journal recovery failed");
    return;
  }
  Serial.printf("[MTA EVT] NVS journal ready: %u/%u stored, next sequence=%lu\n",
                eventCount, MTA_EVENT_CAPACITY, (unsigned long)(eventSequence + 1));
  initializeLocalStock();
}

bool appendEvent(const char* type, const char* data) {
  if (!eventJournalReady) {
    Serial.println("[MTA EVT] ERROR: journal unavailable");
    return false;
  }
  if (eventCount >= MTA_EVENT_CAPACITY) {
    Serial.println("[MTA EVT] FULL: event NOT recorded (no overwrite)");
    return false;
  }
  // Persist sequence before writing the record: no ID reuse after reboot.
  const uint32_t nextSeq = eventSequence + 1;
  if (eventPrefs.putUInt("seq", nextSeq) != sizeof(uint32_t)) {
    Serial.println("[MTA EVT] ERROR: cannot persist sequence");
    return false;
  }
  eventSequence = nextSeq;
  StoredEvent ev = {};
  ev.magic = MTA_EVENT_MAGIC;
  ev.sequence = nextSeq;
  snprintf(ev.eventId, sizeof(ev.eventId), "MTA-001-E%010lu", (unsigned long)nextSeq);
  snprintf(ev.eventType, sizeof(ev.eventType), "%s", type);
  snprintf(ev.data, sizeof(ev.data), "%s", data);
  const uint16_t slot = (eventHead + eventCount) % MTA_EVENT_CAPACITY;
  const String key = eventSlotKey(slot);
  // Clear stale acknowledgement before writing a new record in this slot.
  String ackKey = String("ack") + String(slot);
  if (eventPrefs.getBool(ackKey.c_str(), false)) {
    Serial.println("[MTA EVT] ERROR: stale acknowledgement, refusing write");
    return false;
  }
  if (eventPrefs.putBytes(key.c_str(), &ev, sizeof(ev)) != sizeof(ev)) {
    Serial.println("[MTA EVT] ERROR: cannot persist record");
    return false;
  }
  if (eventPrefs.putUShort("count", eventCount + 1) != sizeof(uint16_t)) {
    Serial.println("[MTA EVT] ERROR: cannot persist count; record needs recovery");
    eventJournalReady = false;
    return false;
  }
  ++eventCount;
  Serial.printf("[MTA EVT] SAVED id=%s type=%s queued=%u\n",
                ev.eventId, ev.eventType, eventCount);
  return true;
}

void listEvents() {
  if (!eventJournalReady) {
    Serial.println("[MTA EVT] Journal unavailable");
    return;
  }
  Serial.printf("[MTA EVT] STORED EVENTS: %u/%u\n", eventCount, MTA_EVENT_CAPACITY);
  for (uint16_t i = 0; i < eventCount; ++i) {
    const uint16_t slot = (eventHead + i) % MTA_EVENT_CAPACITY;
    StoredEvent ev = {};
    const String key = eventSlotKey(slot);
    if (eventPrefs.getBytesLength(key.c_str()) != sizeof(ev) ||
        eventPrefs.getBytes(key.c_str(), &ev, sizeof(ev)) != sizeof(ev) ||
        ev.magic != MTA_EVENT_MAGIC) {
      Serial.printf("[MTA EVT] ERROR: corrupt slot %u\n", slot);
      continue;
    }
    Serial.printf("[MTA EVT] %u: id=%s type=%s data=%s\n",
                  i + 1, ev.eventId, ev.eventType, ev.data);
  }
}

// TEST records are moved to a separate NVS namespace; never sent to server.
// Archive capacity is 16. The archive is not erased on normal reboot.
void archiveLegacyTests() {
  if (!eventJournalReady) return;
  if (!archivePrefs.begin("mta-test-arc", false)) {
    Serial.println("[MTA EVT] ERROR: cannot open archive"); return;
  }
  while (eventCount > 0) {
    StoredEvent ev = {};
    if (!readEventAt(eventHead, ev)) break;
    if (strcmp(ev.eventType, "TEST") != 0) break;
    uint16_t archived = archivePrefs.getUShort("count", 0);
    bool found = false;
    for (uint16_t i = 0; i < archived; ++i) {
      char key[9]; snprintf(key, sizeof(key), "t%02u", i);
      StoredEvent old = {};
      if (archivePrefs.getBytes(key, &old, sizeof(old)) == sizeof(old) &&
          strcmp(old.eventId, ev.eventId) == 0) { found = true; break; }
    }
    if (!found) {
      if (archived >= 16) { Serial.println("[MTA EVT] Archive full"); break; }
      char key[9]; snprintf(key, sizeof(key), "t%02u", archived);
      if (archivePrefs.putBytes(key, &ev, sizeof(ev)) != sizeof(ev) ||
          archivePrefs.putUShort("count", archived + 1) != sizeof(uint16_t)) {
        Serial.println("[MTA EVT] Archive write failed"); break;
      }
    }
    if (!acknowledgeHead(ev.eventId)) {
      Serial.println("[MTA EVT] Cannot advance archived TEST"); break;
    }
    Serial.printf("[MTA EVT] ARCHIVED %s (not sent)\n", ev.eventId);
  }
  archivePrefs.end();
}

// A real LOAD event for controlled testing. Quantity +1 affects server stock.
void saveDemoLoad() {
  time_t now = time(nullptr);
  if (now < 1700000000) {
    Serial.println("[MTA EVT] Clock not synchronized; LOAD not saved"); return;
  }
  char data[80];
  snprintf(data, sizeof(data), "{\"product_id\":\"WATER_05\",\"quantity\":1,\"ts\":%lu}",
           (unsigned long)now);
  appendEvent("LOAD", data);
}

// Read-only NVS diagnostics. No changes to journal or partition table.
void printNvsDiagnostics() {
  const esp_partition_t* part = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, nullptr);
  Serial.println("[MTA NVS] --- Diagnostics ---");
  Serial.printf("[MTA NVS] sizeof(StoredEvent) = %u bytes\n", (unsigned)sizeof(StoredEvent));
  Serial.printf("[MTA NVS] Journal = %u/%u records\n", eventCount, MTA_EVENT_CAPACITY);
  if (part) {
    Serial.printf("[MTA NVS] Partition label=%s, size=%lu bytes (%lu KiB)\n",
                  part->label, (unsigned long)part->size,
                  (unsigned long)(part->size / 1024));
  } else {
    Serial.println("[MTA NVS] NVS partition not found");
  }
  nvs_stats_t stats = {};
  esp_err_t err = nvs_get_stats(part ? part->label : nullptr, &stats);
  if (err == ESP_OK) {
    Serial.printf("[MTA NVS] Entries: used=%lu, free=%lu, total=%lu, namespaces=%lu\n",
                  (unsigned long)stats.used_entries, (unsigned long)stats.free_entries,
                  (unsigned long)stats.total_entries, (unsigned long)stats.namespace_count);
    Serial.println("[MTA NVS] Note: entries are NVS slots, not bytes; blobs use multiple slots.");
  } else {
    Serial.printf("[MTA NVS] nvs_get_stats error=0x%X\n", (unsigned)err);
  }
}

// One-time recovery after intentional full Flash erase and NVS repartition.
// Last accepted event was E0000000012; next new event must be E0000000013.
// This command is deliberately NOT a general-purpose counter setter.
void restoreEventSequence13() {
  const uint32_t LAST_SEQUENCE_BEFORE_MIGRATION = 12;
  if (!eventJournalReady || eventCount != 0 || eventHead != 0 || eventSequence != 0) {
    Serial.printf("[MTA EVT] RESTORE REFUSED: ready=%u, count=%u, head=%u, seq=%lu\n",
                  (unsigned)eventJournalReady, (unsigned)eventCount,
                  (unsigned)eventHead, (unsigned long)eventSequence);
    return;
  }
  // A fresh NVS must not contain any pre-existing journal slot or ACK flag.
  for (uint16_t slot = 0; slot < MTA_EVENT_CAPACITY; ++slot) {
    String key = eventSlotKey(slot);
    String ackKey = String("ack") + String(slot);
    if (eventPrefs.isKey(key.c_str()) || eventPrefs.isKey(ackKey.c_str())) {
      Serial.println("[MTA EVT] RESTORE REFUSED: stale journal keys found");
      return;
    }
  }
  if (eventPrefs.putUInt("seq", LAST_SEQUENCE_BEFORE_MIGRATION) != sizeof(uint32_t)) {
    Serial.println("[MTA EVT] RESTORE FAILED: NVS write failed");
    return;
  }
  const uint32_t confirmed = eventPrefs.getUInt("seq", 0);
  if (confirmed != LAST_SEQUENCE_BEFORE_MIGRATION) {
    Serial.println("[MTA EVT] RESTORE FAILED: readback mismatch");
    eventJournalReady = false;
    return;
  }
  eventSequence = confirmed;
  Serial.println("[MTA EVT] RESTORED: last sequence=12, next sequence=13");
}


// ============================================================
// DIAGNOSTICS v0.3.8b-DIAG: read-only GPIO and readiness snapshot
// Send P in Serial Monitor (115200 baud). No GPIO writes here.
// ============================================================
void printReadinessDiagnostics() {
  const char* stateNames[] = {"IDLE", "WAIT_PAYMENT", "DISPENSING", "SUCCESS", "EMPTY", "SERVICE", "SERVICE_DISPENSING", "WAIT_CLEAR_DISPENSER", "WAIT_SHUTTER_CLOSE", "ERROR_STATE"};
  int st = (int)currentState;
  Serial.println("\n[MTA DIAG] ===== INPUT SNAPSHOT =====");
  Serial.printf("[MTA DIAG] state=%s (%d)\n", st >= 0 && st < 10 ? stateNames[st] : "UNKNOWN", st);
  Serial.printf("[MTA DIAG] GPIO16 door=%d (OPEN when HIGH) => %s\n", digitalRead(PIN_SERVICE_DOOR), serviceDoorOpen() ? "OPEN" : "CLOSED");
  Serial.printf("[MTA DIAG] GPIO19 shutter=%d (RAISED when LOW) => %s\n", digitalRead(PIN_SHUTTER), shutterRaised() ? "RAISED" : "DOWN");
  Serial.printf("[MTA DIAG] GPIO21 exit=%d (NORMAL when LOW) => %s\n", digitalRead(PIN_CAN_EXIT), canExitAbnormal() ? "ABNORMAL" : "NORMAL");
  Serial.printf("[MTA DIAG] GPIO35 next_can=%d (PRESENT when LOW) => %s\n", digitalRead(PIN_NEXT_CAN), nextCanPresent() ? "PRESENT" : "ABSENT");
  Serial.printf("[MTA DIAG] GPIO34 tray=%d (OCCUPIED when LOW) => %s\n", digitalRead(PIN_TRAY_SENSOR), trayOccupied() ? "OCCUPIED" : "CLEAR");
  const char* reason = serviceDoorOpen() ? "SERVICE: door open" : shutterRaised() ? "WAIT_SHUTTER_CLOSE: shutter raised" : canExitAbnormal() ? "ERROR_STATE: exit switch abnormal" : !nextCanPresent() ? "EMPTY: next can absent" : trayOccupied() ? "WAIT_CLEAR_DISPENSER: tray occupied" : "IDLE: all readiness inputs OK";
  Serial.printf("[MTA DIAG] Readiness result IF checked now: %s\n", reason);
  Serial.println("[MTA DIAG] ==========================\n");
}

void processEventTestCommands() {
  while (Serial.available() > 0) {
    const char command = (char)Serial.read();
    if (command == 'P' || command == 'p') printReadinessDiagnostics();
    else if (command == 'T' || command == 't') appendEvent("TEST", "serial-test");
    else if (command == 'L' || command == 'l') listEvents();
    else if (command == 'I' || command == 'i') Serial.printf("[MTA STOCK] Local stock=%u\n", localStock);
    else if (command == 'A' || command == 'a') archiveLegacyTests();
    else if (command == 'D' || command == 'd') saveDemoLoad();
    else if (command == 'N' || command == 'n') printNvsDiagnostics();
    else if (command == 'R' || command == 'r') restoreEventSequence13();
  }
}

// Only the main loop reads/writes Preferences; network task uses FreeRTOS queues.
void processEventDelivery() {
  if (!eventJournalReady || !eventSendQueue || !eventResultQueue) return;
  EventSendResult result = {};
  if (xQueueReceive(eventResultQueue, &result, 0) == pdTRUE) {
    eventSendPending = false;
    if (result.accepted) {
      if (acknowledgeHead(result.eventId))
        Serial.printf("[MTA EVT] CONFIRMED %s, remaining=%u\n", result.eventId, eventCount);
      else Serial.println("[MTA EVT] ERROR: confirmation could not be persisted");
    } else {
      Serial.printf("[MTA EVT] NOT confirmed id=%s HTTP=%d; retained\n",
                    result.eventId, result.httpCode);
    }
  }
  if (eventSendPending || eventCount == 0 ||
      (eventLastAttempt && millis() - eventLastAttempt < EVENT_RETRY_MS)) return;
  StoredEvent ev = {};
  if (!readEventAt(eventHead, ev)) {
    Serial.println("[MTA EVT] ERROR: corrupt head; delivery stopped"); return;
  }
  if (strcmp(ev.eventType, "TEST") == 0) return; // Archive explicitly with A.
  const bool isLoad = strcmp(ev.eventType, "LOAD") == 0;
  const bool isSale = strcmp(ev.eventType, "SALE") == 0;
  const bool isErrorSet = strcmp(ev.eventType, "ERROR_SET") == 0;
  const bool isErrorClear = strcmp(ev.eventType, "ERROR_CLEAR") == 0;
  if (!isLoad && !isSale && !isErrorSet && !isErrorClear) {
    Serial.println("[MTA EVT] Unsupported event at head; retained"); return;
  }
  const char* tsField = strstr(ev.data, "\"ts\":");
  if (!tsField) { Serial.println("[MTA EVT] Missing timestamp; retained"); return; }
  unsigned long unixSeconds = strtoul(tsField + 5, nullptr, 10);
  if (unixSeconds < 1700000000) {
    time_t now = time(nullptr);
    if (now < 1700000000) return;
    unixSeconds = (unsigned long)now; // estimated time when offline clock was unset
  }
  struct tm utc = {};
  time_t stamp = (time_t)unixSeconds;
  gmtime_r(&stamp, &utc);
  char iso[32]; strftime(iso, sizeof(iso), "%Y-%m-%dT%H:%M:%SZ", &utc);
  EventSendRequest req = {};
  snprintf(req.eventId, sizeof(req.eventId), "%s", ev.eventId);
  if (isLoad) {
    snprintf(req.payload, sizeof(req.payload),
             "{\"event_id\":\"%s\",\"device_id\":\"MTA-001\","
             "\"event_type\":\"LOAD\",\"created_at\":\"%s\","
             "\"data\":{\"product_id\":\"WATER_05\",\"quantity\":1}}",
             ev.eventId, iso);
  } else if (isSale) {
    snprintf(req.payload, sizeof(req.payload),
             "{\"event_id\":\"%s\",\"device_id\":\"MTA-001\","
             "\"event_type\":\"SALE\",\"session_id\":\"%s\","
             "\"created_at\":\"%s\",\"data\":{\"product_id\":\"WATER_05\","
             "\"price\":%d,\"payment\":\"SUCCESS\",\"dispense\":\"SUCCESS\"}}",
             ev.eventId, ev.eventId, iso, MTA_TEST_PRICE);
  } else {
    const char* code = strstr(ev.data, "\"code\":\"");
    if (!code) { Serial.println("[MTA EVT] Missing error code; retained"); return; }
    code += 8;
    char fault[32] = {};
    size_t n = 0;
    while (code[n] && code[n] != '"' && n < sizeof(fault) - 1) { fault[n] = code[n]; ++n; }
    snprintf(req.payload, sizeof(req.payload),
             "{\"event_id\":\"%s\",\"device_id\":\"MTA-001\","
             "\"event_type\":\"%s\",\"created_at\":\"%s\","
             "\"data\":{\"error_code\":\"%s\"}}",
             ev.eventId, isErrorSet ? "ERROR_SET" : "ERROR_CLEAR", iso, fault);
  }
  if (xQueueSend(eventSendQueue, &req, 0) == pdTRUE) {
    eventSendPending = true;
    eventLastAttempt = millis();
  }
}

// ============================================================
// COMMUNICATION MANAGER (v0.3.6)
// Wi-Fi and HTTP run on a separate FreeRTOS task.
// Never perform network operations in the mechanical loop.
// ============================================================
#include <WiFi.h>
#include <HTTPClient.h>
#include "mta_secrets.h"  // Local only; never commit real credentials.

// Wi-Fi credentials and MTA_DEVICE_KEY come from mta_secrets.h.
const char* MTA_HEARTBEAT_URL =
  "http://192.168.1.106:8000/api/v1/device/heartbeat";
const char* MTA_DEVICE_ID = "MTA-001";
const char* MTA_EVENTS_URL = "http://192.168.1.106:8000/api/v1/device/events";

const unsigned long MTA_HEARTBEAT_INTERVAL_MS = 10000;
const unsigned long MTA_WIFI_RETRY_MS = 10000;

struct HeartbeatSnapshot {
  char state[32];
  char activeError[32];
};

QueueHandle_t heartbeatQueue = nullptr;
unsigned long lastHeartbeatSnapshotAt = 0;

const char* stateNameForServer(MachineState state) {
  switch (state) {
    case IDLE: return "IDLE";
    case WAIT_PAYMENT: return "WAIT_PAYMENT";
    case DISPENSING: return "DISPENSING";
    case SUCCESS: return "SUCCESS";
    case EMPTY: return "EMPTY";
    case SERVICE: return "SERVICE";
    case SERVICE_DISPENSING: return "SERVICE_DISPENSING";
    case WAIT_CLEAR_DISPENSER: return "WAIT_CLEAR_DISPENSER";
    case WAIT_SHUTTER_CLOSE: return "WAIT_SHUTTER_CLOSE";
    case ERROR_STATE: return "ERROR";
    default: return "ERROR";
  }
}

void queueHeartbeatSnapshot() {
  if (heartbeatQueue == nullptr) return;
  HeartbeatSnapshot snapshot = {};
  snprintf(snapshot.state, sizeof(snapshot.state), "%s", stateNameForServer(currentState));
  // v0.3.5 has no specific fault-code tracking yet.
  // Generic marker until ErrorManager/EventManager integration.
  snprintf(snapshot.activeError, sizeof(snapshot.activeError), "%s",
           currentState == ERROR_STATE ? "UNCLASSIFIED_ERROR" : "NONE");
  xQueueOverwrite(heartbeatQueue, &snapshot);
}

void communicationTask(void* parameter) {
  HeartbeatSnapshot snapshot;
  unsigned long lastWiFiAttempt = 0;
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  configTime(0, 0, "pool.ntp.org", "time.google.com");
  Serial.println("[MTA NET] Task started");

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      const unsigned long now = millis();
      if (lastWiFiAttempt == 0 || now - lastWiFiAttempt >= MTA_WIFI_RETRY_MS) {
        lastWiFiAttempt = now;
        WiFi.begin(MTA_WIFI_SSID, MTA_WIFI_PASSWORD);
        Serial.println("[MTA NET] Connecting to Wi-Fi...");
      }
      // Drop stale snapshots while offline; heartbeat is current state, not a journal.
      if (heartbeatQueue != nullptr) xQueueReceive(heartbeatQueue, &snapshot, 0);
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    // One FIFO event request at a time. On failure the main loop retains it.
    EventSendRequest request = {};
    if (eventSendQueue && xQueueReceive(eventSendQueue, &request, 0) == pdTRUE) {
      EventSendResult result = {};
      snprintf(result.eventId, sizeof(result.eventId), "%s", request.eventId);
      HTTPClient eventHttp;
      eventHttp.setConnectTimeout(1500);
      eventHttp.setTimeout(1500);
      if (eventHttp.begin(MTA_EVENTS_URL)) {
        eventHttp.addHeader("Content-Type", "application/json");
        eventHttp.addHeader("X-MTA-Key", MTA_DEVICE_KEY);
        result.httpCode = eventHttp.POST(String(request.payload));
        if (result.httpCode == 200) {
          String response = eventHttp.getString();
          result.accepted = response.indexOf("\"accepted\"") >= 0 ||
                            response.indexOf("\"already_accepted\"") >= 0;
        }
        eventHttp.end();
      } else result.httpCode = -1;
      Serial.printf("[MTA NET] Event id=%s HTTP=%d confirmed=%d\n",
                    result.eventId, result.httpCode, result.accepted);
      if (eventResultQueue) xQueueSend(eventResultQueue, &result, portMAX_DELAY);
    }
    if (xQueueReceive(heartbeatQueue, &snapshot, pdMS_TO_TICKS(250)) == pdTRUE) {
      Serial.printf("[MTA NET] Wi-Fi connected, IP=%s\n", WiFi.localIP().toString().c_str());
      HTTPClient http;
      http.setConnectTimeout(1500);
      http.setTimeout(1500);
      if (http.begin(MTA_HEARTBEAT_URL)) {
        http.addHeader("Content-Type", "application/json");
        http.addHeader("X-MTA-Key", MTA_DEVICE_KEY);
        String payload = String("{\"device_id\":\"") + MTA_DEVICE_ID +
          "\",\"state\":\"" + snapshot.state +
          "\",\"active_error\":\"" + snapshot.activeError + "\"}";
        int code = http.POST(payload);
        Serial.printf("[MTA NET] Heartbeat state=%s HTTP=%d\n", snapshot.state, code);
        http.end();
      } else {
        Serial.println("[MTA NET] HTTP begin failed");
      }
    }
  }
}

void setupCommunication() {
  eventSendQueue = xQueueCreate(1, sizeof(EventSendRequest));
  eventResultQueue = xQueueCreate(1, sizeof(EventSendResult));
  heartbeatQueue = xQueueCreate(1, sizeof(HeartbeatSnapshot));
  if (heartbeatQueue == nullptr) {
    Serial.println("[MTA NET] Cannot allocate heartbeat queue");
    return;
  }
  BaseType_t started = xTaskCreatePinnedToCore(
    communicationTask, "MTA_Network", 8192, nullptr, 1, nullptr, 0
  );
  if (started != pdPASS) {
    Serial.println("[MTA NET] Cannot start network task");
    vQueueDelete(heartbeatQueue);
    heartbeatQueue = nullptr;
  }
}

void processCommunication() {
  const unsigned long now = millis();
  if (lastHeartbeatSnapshotAt == 0 ||
      now - lastHeartbeatSnapshotAt >= MTA_HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatSnapshotAt = now;
    queueHeartbeatSnapshot();
  }
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  Serial.println("[MTA] Firmware v0.3.10-MVP starting");
  setupEventManager();
  Serial.println("[MTA EVT] Commands: T=test, L=list, A=archive TEST, D=LOAD +1, I=stock, N=NVS stats, R=restore seq13 (fresh NVS only), P=GPIO diagnostics");

  // ----------------------------------------------------------
  // OUTPUTS
  // ----------------------------------------------------------

  pinMode(PIN_WANT_WINDOW, OUTPUT);
  pinMode(PIN_ARROW, OUTPUT);
  pinMode(PIN_WANT_BUTTON_LED, OUTPUT);

  pinMode(PIN_LED_READY, OUTPUT);
  pinMode(PIN_LED_DISPENSE, OUTPUT);
  pinMode(PIN_LED_TAKE, OUTPUT);
  pinMode(PIN_LED_EMPTY, OUTPUT);
  pinMode(PIN_LED_ERROR, OUTPUT);

  pinMode(PIN_MOTOR_RELAY, OUTPUT);
  pinMode(PIN_REFUND_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);


  // ----------------------------------------------------------
  // SAFE OUTPUT STATE
  // ----------------------------------------------------------

  allCustomerIndicatorsOff();

  digitalWrite(PIN_MOTOR_RELAY, LOW);
  digitalWrite(PIN_REFUND_LED, LOW);
  digitalWrite(PIN_BUZZER, LOW);

  motorRunning = false;
  refundRequested = false;


  // ----------------------------------------------------------
  // INPUTS WITH INTERNAL PULL-UP
  // ----------------------------------------------------------

  pinMode(PIN_WANT_BUTTON, INPUT_PULLUP);
  pinMode(PIN_MANUAL_DISPENSE, INPUT_PULLUP);
  pinMode(PIN_SERVICE_DOOR, INPUT_PULLUP);
  pinMode(PIN_SHUTTER, INPUT_PULLUP);
  pinMode(PIN_CAN_EXIT, INPUT_PULLUP);


  // ----------------------------------------------------------
  // IR SENSORS
  // ----------------------------------------------------------

  pinMode(PIN_TRAY_SENSOR, INPUT);
  pinMode(PIN_NEXT_CAN, INPUT);


  // ----------------------------------------------------------
  // PAYMENT SIMULATOR
  // ----------------------------------------------------------

  pinMode(PIN_PAY_OK, INPUT);
  pinMode(PIN_PAY_FAIL, INPUT);


  // ----------------------------------------------------------
  // DEBOUNCED INPUT INITIALIZATION
  // ----------------------------------------------------------

  initDebouncedInput(
    wantButton,
    PIN_WANT_BUTTON
  );

  initDebouncedInput(
    manualDispense,
    PIN_MANUAL_DISPENSE
  );

  initDebouncedInput(
    shutter,
    PIN_SHUTTER
  );

  initDebouncedInput(
    serviceDoor,
    PIN_SERVICE_DOOR
  );

  initDebouncedInput(
    canExit,
    PIN_CAN_EXIT
  );

  initDebouncedInput(
    traySensor,
    PIN_TRAY_SENSOR
  );

  initDebouncedInput(
    payOK,
    PIN_PAY_OK
  );

  initDebouncedInput(
    payFail,
    PIN_PAY_FAIL
  );


  delay(500);


  // ----------------------------------------------------------
  // STARTUP READINESS
  // ----------------------------------------------------------

  runReadinessCheck(IDLE);
  printReadinessDiagnostics();
  Serial.println("[MTA] Setup complete, starting communications");
  setupCommunication();
}


// ============================================================
// LOOP
// ============================================================

void loop() {

  processEventTestCommands();
  processEventDelivery();
  processInputs();


  // ----------------------------------------------------------
  // TIMERS
  // ----------------------------------------------------------

  processDispenseTimeout();

  // NEW v0.3.5
  processManualHoldTimer();
  processServiceDispenseTimeout();

  processEmptyTimer();

  // NEW v0.3.4
  processShutterRecoveryTimer();


  // ----------------------------------------------------------
  // HARD MOTOR SAFETY
  // ----------------------------------------------------------

  if (currentState != DISPENSING &&
      currentState != SERVICE_DISPENSING) {
    motorOff();
  }


  if (!MOTOR_ENABLED) {
    motorOff();
  }

  processCommunication();
}

