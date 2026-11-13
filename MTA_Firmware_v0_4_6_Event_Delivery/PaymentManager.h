#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>

// Vendista Slave+GSM adapter. No motor/indicator GPIO. No bank logic.
// Nonblocking; TimerManager (or a test harness) invokes onTimeout().
class PaymentManager {
public:
  enum class State : uint8_t { IDLE, STARTING, WAIT_RESULT, WAIT_DISPENSE, FINISHING };
  enum class Event : uint8_t {
    PAYMENT_STARTED, PAYMENT_APPROVED, PAYMENT_NO_CARD, PAYMENT_DECLINED,
    SALE_REPORTED, REFUND_REQUEST_ACCEPTED, PAYMENT_CYCLE_FINISHED,
    PAYMENT_ERROR
  };
  enum class Error : uint8_t {
    NONE, BAD_ARGUMENT, BUSY, ACK_TIMEOUT, RESULT_TIMEOUT,
    AUTH_TIMEOUT, DISPLAY_ACK_TIMEOUT, PROTOCOL, REBOOT, UNKNOWN_RESULT
  };
  enum class Timer : uint8_t { NONE, ACK, CARD_RESULT, AUTH_RESULT };
  struct Config {
    uint32_t baud = 115200;
    int8_t rxPin = 39; // ESP32 input only, external signal-level matching required
    int8_t txPin = 15;
    uint32_t ackMs = 3000;
    uint32_t cardMs = 45000; // safeguard; terminal handles normal 30s card timer
    uint32_t authMs = 90000;
    bool gsm = true;
  };
  using EventCallback = void (*)(Event event, Error error, void *ctx);
  using TimerCallback = void (*)(Timer timer, uint32_t durationMs, void *ctx);
  // Timer::NONE cancels the outstanding TimerManager timer.

  explicit PaymentManager(HardwareSerial &uart);
  void begin(const Config &cfg, EventCallback onEvent, TimerCallback onTimer, void *ctx = nullptr);
  void update();
  bool startPayment(uint32_t priceKopecks, uint16_t humanItemNumber);
  bool confirmDispensed();
  bool reportNotDispensed();
  bool cancelCardReading(); // Only while waiting for a card, before approval
  void onTimeout(Timer timer); // Call ONLY for the currently armed timer
  State state() const { return state_; }
  bool isIdle() const { return state_ == State::IDLE; }
  bool uncertainFinancialResult() const { return uncertain_; }
  uint32_t goodFrames() const { return goodFrames_; }
  uint32_t badCrcFrames() const { return badCrcFrames_; }
  uint32_t discardedBytes() const { return discardedBytes_; }
  // Diagnostics only; no influence on payment flow.
  uint32_t rawRxBytes() const { return rawRxBytes_; }
  uint32_t ackFrames() const { return ackFrames_; }
  uint32_t cardFrames() const { return cardFrames_; }
  uint32_t authFrames() const { return authFrames_; }
  uint32_t otherFrames() const { return otherFrames_; }
  uint32_t invalidLengthBytes() const { return invalidLengthBytes_; }
  uint32_t partialFrameTimeouts() const { return partialFrameTimeouts_; }
  bool hasHardwareErrorMonitor() const { return hwErrorMonitorAvailable_; }
  uint32_t hwBreak() const { return hwBreak_; }
  uint32_t hwBufferFull() const { return hwBufferFull_; }
  uint32_t hwFifoOverflow() const { return hwFifoOverflow_; }
  uint32_t hwFrame() const { return hwFrame_; }
  uint32_t hwParity() const { return hwParity_; }
  uint32_t peakRxPending() const { return peakRxPending_; }

  static const char *stateName(State v);
  static const char *eventName(Event v);
  static const char *errorName(Error v);

private:
  enum class Await : uint8_t { NONE, START_PICTURE, VEND_REQUEST, READ_CARD, VEND_REPORT, REFUND, CANCEL_READ, FINAL_PICTURE };
  enum class Finish : uint8_t { SUCCESS, NO_CARD, DECLINED, REFUND_REQUESTED, FAILURE };
  HardwareSerial &uart_;
  Config cfg_;
  EventCallback cb_ = nullptr;
  TimerCallback tcb_ = nullptr;
  void *ctx_ = nullptr;
  State state_ = State::IDLE;
  Await awaiting_ = Await::NONE;
  Timer timer_ = Timer::NONE;
  Finish finish_ = Finish::FAILURE;
  uint16_t expectedAck_ = 0;
  uint32_t price_ = 0;
  uint16_t item_ = 0;
  bool uncertain_ = false;
  bool cardReadSeen_ = false;
  uint8_t buf_[260] = {};
  size_t used_ = 0;
  uint32_t rxStarted_ = 0;
  uint32_t goodFrames_ = 0, badCrcFrames_ = 0, discardedBytes_ = 0;
  uint32_t rawRxBytes_ = 0, ackFrames_ = 0, cardFrames_ = 0;
  uint32_t authFrames_ = 0, otherFrames_ = 0;
  uint32_t invalidLengthBytes_ = 0, partialFrameTimeouts_ = 0;
  uint8_t crcTraceThisSession_ = 0;
  bool hwErrorMonitorAvailable_ = false;
  volatile uint32_t hwBreak_ = 0, hwBufferFull_ = 0, hwFifoOverflow_ = 0;
  volatile uint32_t hwFrame_ = 0, hwParity_ = 0;
  uint32_t peakRxPending_ = 0;

  void event(Event e, Error err = Error::NONE);
  void timer(Timer which, uint32_t durationMs = 0);
  void enter(State s);
  void send(const uint8_t *body, size_t length, Await ack);
  void sendPicture(bool initial);
  void sendRequest();
  void sendReadCard();
  void sendVendReport();
  void sendCancelLast();
  void beginFinish(Finish finish);
  void finished(bool displayAck);
  void fail(Error reason, bool financialUncertain = false);
  void frame(const uint8_t *body, size_t len);
  void ack(uint16_t crc);
  void receiveByte(uint8_t b);
  void dropByte();
  static uint16_t crc(const uint8_t *p, size_t n);
  static const char *awaitName(Await a);
  static const char *timerName(Timer t);
};
