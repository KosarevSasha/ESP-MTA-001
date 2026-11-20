#include "PaymentManager.h"
#include <string.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

PaymentManager::PaymentManager(HardwareSerial &uart) : uart_(uart) {}

void PaymentManager::begin(const Config &cfg, EventCallback onEvent, TimerCallback onTimer, void *ctx) {
  cfg_ = cfg; cb_ = onEvent; tcb_ = onTimer; ctx_ = ctx;
  state_ = State::IDLE; awaiting_ = Await::NONE;
  timer_ = Timer::NONE; uncertain_ = false; used_ = 0;
  diagPending_ = false; windowAt_ = millis();
  uart_.setRxBufferSize(4096);
  uart_.begin(cfg_.baud, SERIAL_8N1, cfg_.rxPin, cfg_.txPin);
  // Diagnostic only. Callback runs in UART event task; do not log or touch
  // payment state there. May not be provided by older Arduino-ESP32 cores.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  uart_.onReceiveError([this](hardwareSerial_error_t e) {
    switch (e) {
      case UART_BREAK_ERROR:       ++hwBreak_; break;
      case UART_BUFFER_FULL_ERROR: ++hwBufferFull_; break;
      case UART_FIFO_OVF_ERROR:    ++hwFifoOverflow_; break;
      case UART_FRAME_ERROR:       ++hwFrame_; break;
      case UART_PARITY_ERROR:      ++hwParity_; break;
      default: break;
    }
  });
  hwErrorMonitorAvailable_ = true;
#else
  hwErrorMonitorAvailable_ = false;
#endif
  Serial.printf("[PM DIAG] UART hardware error callback %s (no payment behavior changes)\n",
                hwErrorMonitorAvailable_ ? "ENABLED" : "UNAVAILABLE_ON_THIS_CORE");
}

const char *PaymentManager::stateName(State s) {
  switch (s) { case State::IDLE:return "IDLE"; case State::STARTING:return "STARTING";
    case State::WAIT_RESULT:return "WAIT_RESULT"; case State::WAIT_DISPENSE:return "WAIT_DISPENSE";
    case State::FINISHING:return "FINISHING"; }
  return "?";
}
const char *PaymentManager::eventName(Event e) {
  switch (e) {
    case Event::PAYMENT_STARTED:return "PAYMENT_STARTED";
    case Event::PAYMENT_APPROVED:return "PAYMENT_APPROVED";
    case Event::PAYMENT_NO_CARD:return "PAYMENT_NO_CARD";
    case Event::PAYMENT_DECLINED:return "PAYMENT_DECLINED";
    case Event::SALE_REPORTED:return "SALE_REPORTED";
    case Event::REFUND_REQUEST_ACCEPTED:return "REFUND_REQUEST_ACCEPTED";
    case Event::PAYMENT_CYCLE_FINISHED:return "PAYMENT_CYCLE_FINISHED";
    case Event::PAYMENT_ERROR:return "PAYMENT_ERROR";
  }
  return "?";
}
const char *PaymentManager::errorName(Error e) {
  switch (e) {
    case Error::NONE:return "NONE"; case Error::BAD_ARGUMENT:return "BAD_ARGUMENT";
    case Error::BUSY:return "BUSY"; case Error::ACK_TIMEOUT:return "ACK_TIMEOUT";
    case Error::RESULT_TIMEOUT:return "RESULT_TIMEOUT"; case Error::AUTH_TIMEOUT:return "AUTH_TIMEOUT";
    case Error::DISPLAY_ACK_TIMEOUT:return "DISPLAY_ACK_TIMEOUT";
    case Error::PROTOCOL:return "PROTOCOL"; case Error::REBOOT:return "REBOOT";
    case Error::UNKNOWN_RESULT:return "UNKNOWN_RESULT";
  }
  return "?";
}
// v0.4.1 ACK Trace: only protocol metadata. NEVER print card/PAN payload.
const char *PaymentManager::awaitName(Await a) {
  switch (a) {
    case Await::NONE: return "NONE";
    case Await::START_PICTURE: return "START_PICTURE";
    case Await::VEND_REQUEST: return "VEND_REQUEST";
    case Await::READ_CARD: return "READ_CARD";
    case Await::VEND_REPORT: return "VEND_REPORT";
    case Await::REFUND: return "CANCEL_LAST_TRANSACTION";
    case Await::CANCEL_READ: return "CANCEL_READ_CARD";
    case Await::FINAL_PICTURE: return "FINAL_PICTURE";
  }
  return "?";
}
const char *PaymentManager::timerName(Timer t) {
  switch (t) {
    case Timer::NONE: return "NONE";
    case Timer::ACK: return "ACK";
    case Timer::CARD_RESULT: return "CARD_RESULT";
    case Timer::AUTH_RESULT: return "AUTH_RESULT";
  }
  return "?";
}
void PaymentManager::event(Event e, Error error) { if (cb_) cb_(e,error,ctx_); }
void PaymentManager::timer(Timer which, uint32_t durationMs) {
  timer_ = which;
  if (tcb_) tcb_(which,durationMs,ctx_);
}
void PaymentManager::enter(State s) { state_ = s; }
uint16_t PaymentManager::crc(const uint8_t *p, size_t n) {
  uint16_t value = 0xFFFF;
  for (size_t i=0;i<n;++i) {
    value ^= uint16_t(p[i]) << 8;
    for (uint8_t j=0;j<8;++j)
      value = (value & 0x8000) ? uint16_t((value << 1) ^ 0x8005) : uint16_t(value << 1);
  }
  return value ^ 0xFFFF;
}
void PaymentManager::send(const uint8_t *body, size_t length, Await ackFor) {
  if (!length || length > 16) return;
  uint8_t packet[20];
  const uint16_t c = crc(body,length);
  packet[0] = uint8_t(length); packet[1] = uint8_t(length >> 8);
  packet[2] = uint8_t(c); packet[3] = uint8_t(c >> 8);
  memcpy(packet+4,body,length);
  awaiting_ = ackFor; expectedAck_ = c;
  // Arm first; a fast response will not be mistaken for an unsolicited ACK.
  timer(Timer::ACK,cfg_.ackMs);
  const size_t written = uart_.write(packet,length+4);
  Serial.printf("[PM TRACE] TX command=%s bytes=%u written=%u ackCRC=%04X ackWaitMs=%lu PM=%s\n",
                awaitName(ackFor), (unsigned)(length+4), (unsigned)written,
                (unsigned)c, (unsigned long)cfg_.ackMs, stateName(state_));
}
void PaymentManager::sendPicture(bool initial) {
  const uint8_t b[] = {0x03,0x02};
  send(b,sizeof(b),initial ? Await::START_PICTURE : Await::FINAL_PICTURE);
}
void PaymentManager::sendRequest() {
  const uint16_t zeroBased = uint16_t(item_-1);
  const uint8_t b[] = {0x31,uint8_t(price_),uint8_t(price_>>8),uint8_t(price_>>16),uint8_t(price_>>24),
                      uint8_t(zeroBased),uint8_t(zeroBased>>8),uint8_t(cfg_.gsm ? 1 : 0)};
  send(b,sizeof(b),Await::VEND_REQUEST);
}
void PaymentManager::sendReadCard() {
  // ReadCard: price 4 bytes; RUB currency 0x4306; time=0 -> terminal clock; GSM=1.
  const uint8_t b[] = {0x01,uint8_t(price_),uint8_t(price_>>8),uint8_t(price_>>16),uint8_t(price_>>24),
                      0x06,0x43,0,0,0,0,uint8_t(cfg_.gsm ? 1 : 0)};
  send(b,sizeof(b),Await::READ_CARD);
}
void PaymentManager::sendVendReport() {
  // Vendor protocol: cashless, price LE uint16, human product number uint8, GSM flag.
  const uint8_t b[] = {0x0E,0x02,uint8_t(price_),uint8_t(price_>>8),uint8_t(item_),uint8_t(cfg_.gsm ? 1 : 0)};
  send(b,sizeof(b),Await::VEND_REPORT);
}
void PaymentManager::sendCancelLast() {
  // Slave section 3.7: one-byte CancelLastTransaction (0x07).
  // Terminal may additionally emit PacketToServer. ACK != funds refunded.
  const uint8_t b[] = {0x07};
  send(b,sizeof(b),Await::REFUND);
}
bool PaymentManager::startPayment(uint32_t priceKopecks,uint16_t humanItemNumber) {
  if (state_ != State::IDLE || uncertain_ || diagPending_) return false;
  if (priceKopecks == 0 || priceKopecks > 65535 || humanItemNumber == 0 || humanItemNumber > 255) {
    event(Event::PAYMENT_ERROR,Error::BAD_ARGUMENT); return false;
  }
  price_ = priceKopecks; item_ = humanItemNumber;
  cardReadSeen_ = false;
  crcTraceThisSession_ = 0; // bound diagnostics; do not leak card payloads
  enter(State::STARTING);
  // Same initial display as verified Cycle Test (no 5-second delay in production).
  sendPicture(true);
  return true;
}
bool PaymentManager::confirmDispensed() {
  if (state_ != State::WAIT_DISPENSE) return false;
  enter(State::FINISHING);
  sendVendReport();
  return true;
}
bool PaymentManager::reportNotDispensed() {
  if (state_ != State::WAIT_DISPENSE) return false;
  enter(State::FINISHING);
  sendCancelLast();
  return true;
}
bool PaymentManager::cancelCardReading() {
  if (state_ != State::WAIT_RESULT || cardReadSeen_ || awaiting_ != Await::NONE) return false;
  enter(State::FINISHING);
  const uint8_t b[] = {0x08};
  send(b,sizeof(b),Await::CANCEL_READ);
  return true;
}
void PaymentManager::beginFinish(Finish finish) {
  finish_ = finish;
  enter(State::FINISHING);
  sendPicture(false);
}
void PaymentManager::finished(bool displayAck) {
  timer(Timer::NONE);
  awaiting_ = Await::NONE;
  enter(State::IDLE);
  if (displayAck && !uncertain_) event(Event::PAYMENT_CYCLE_FINISHED);
  // If command outcome is uncertain, StateManager receives PAYMENT_ERROR instead
  // and must not assume this adapter is cleared for a new real payment.
}
void PaymentManager::fail(Error reason,bool financialUncertain) {
  if (financialUncertain) uncertain_ = true;
  event(Event::PAYMENT_ERROR,reason);
  if (awaiting_ == Await::FINAL_PICTURE) {
    finished(false);
  } else {
    // One attempt to restore display; never retry financial packet.
    beginFinish(Finish::FAILURE);
  }
}
void PaymentManager::ack(uint16_t c) {
  if (diagPending_) {
    if (c == diagExpectedAck_) {
      ++diagAckMatches_;
      diagPending_ = false;
      Serial.printf("[HYBRID] DIAG_ACK_OK crc=%04X latencyMs=%lu matched=%lu\n",
                    (unsigned)c, (unsigned long)(millis()-diagSentAt_),
                    (unsigned long)diagAckMatches_);
    } else {
      Serial.printf("[HYBRID] DIAG_ACK_OTHER crc=%04X expected=%04X\n",
                    (unsigned)c, (unsigned)diagExpectedAck_);
    }
    return;
  }
  if (awaiting_ == Await::NONE || c != expectedAck_) {
    Serial.printf("[PM TRACE] RX ACK_UNMATCHED crc=%04X expected=%04X pending=%s PM=%s\n",
                  (unsigned)c, (unsigned)expectedAck_, awaitName(awaiting_), stateName(state_));
    return;
  }
  Serial.printf("[PM TRACE] RX ACK_OK crc=%04X for=%s PM=%s\n",
                (unsigned)c, awaitName(awaiting_), stateName(state_));
  const Await accepted = awaiting_;
  awaiting_ = Await::NONE;
  timer(Timer::NONE);
  switch (accepted) {
    case Await::START_PICTURE: sendRequest(); break;
    case Await::VEND_REQUEST: sendReadCard(); break;
    case Await::READ_CARD:
      enter(State::WAIT_RESULT);
      timer(Timer::CARD_RESULT,cfg_.cardMs);
      event(Event::PAYMENT_STARTED);
      break;
    case Await::VEND_REPORT:
      event(Event::SALE_REPORTED);
      beginFinish(Finish::SUCCESS);
      break;
    case Await::REFUND:
      event(Event::REFUND_REQUEST_ACCEPTED);
      beginFinish(Finish::REFUND_REQUESTED);
      break;
    case Await::CANCEL_READ:
      beginFinish(Finish::NO_CARD);
      break;
    case Await::FINAL_PICTURE:
      finished(true);
      break;
    default: break;
  }
}
void PaymentManager::frame(const uint8_t *body,size_t len) {
  if (!len) return;
  const uint8_t type = body[0];
  if (type == 0x15 && len >= 3) {
    ++ackFrames_;
    ack(uint16_t(body[1]) | (uint16_t(body[2])<<8)); return;
  }
  // Process card/auth only after ReadCard ACK. CardReadResult=2 is useful but
  // may be missed; an explicit positive authorization is the authoritative event.
  if (type == 0x13 && len >= 2) {
    ++cardFrames_;
    Serial.printf("[PM TRACE] RX CARD_RESULT value=%u PM=%s pending=%s %s\n",
                  (unsigned)body[1], stateName(state_), awaitName(awaiting_),
                  state_ == State::WAIT_RESULT ? "HANDLED" : "IGNORED_STAGE");
  }
  if (type == 0x14 && len >= 2) {
    ++authFrames_;
    Serial.printf("[PM TRACE] RX AUTH_RESULT value=%u PM=%s pending=%s %s\n",
                  (unsigned)body[1], stateName(state_), awaitName(awaiting_),
                  state_ == State::WAIT_RESULT ? "HANDLED" : "IGNORED_STAGE");
  }
  if (type != 0x13 && type != 0x14 && type != 0x15) {
    ++otherFrames_;
    Serial.printf("[PM TRACE] RX OTHER type=%02X len=%u PM=%s pending=%s (payload hidden)\n",
                  (unsigned)type, (unsigned)len, stateName(state_), awaitName(awaiting_));
  }
  if (type == 0x13 && len >= 2 && state_ == State::WAIT_RESULT) {
    if (body[1] == 0) {
      timer(Timer::NONE);
      event(Event::PAYMENT_NO_CARD);
      beginFinish(Finish::NO_CARD);
    } else if (body[1] == 2) {
      cardReadSeen_ = true;
      timer(Timer::AUTH_RESULT,cfg_.authMs);
    } else if (body[1] != 1) {
      fail(Error::PROTOCOL,true);
    }
    return;
  }
  if (type == 0x14 && len >= 2 && state_ == State::WAIT_RESULT) {
    timer(Timer::NONE);
    if (body[1] == 1) {
      enter(State::WAIT_DISPENSE);
      event(Event::PAYMENT_APPROVED);
    } else if (body[1] == 0) {
      event(Event::PAYMENT_DECLINED);
      beginFinish(Finish::DECLINED);
    } else {
      fail(Error::PROTOCOL,true);
    }
    return;
  }
  if (type == 0x16 && state_ != State::IDLE) {
    fail(Error::REBOOT,true); return;
  }
  // 0x12 PacketToServer must NOT be forwarded by Master in Slave+GSM;
  // never log sensitive card data. Other messages ignored by adapter.
}
void PaymentManager::dropByte() {
  if (!used_) return;
  --used_; ++discardedBytes_;
  if (used_) memmove(buf_,buf_+1,used_);
  rxStarted_ = millis();
}
void PaymentManager::receiveByte(uint8_t b) {
  ++rawRxBytes_;
  if (!used_) rxStarted_ = millis();
  if (used_ == sizeof(buf_)) { used_ = 0; ++discardedBytes_; rxStarted_ = millis(); }
  buf_[used_++] = b;
  while (used_ >= 2) {
    const size_t len = size_t(buf_[0]) | (size_t(buf_[1])<<8);
    if (len == 0 || len > 255) { ++invalidLengthBytes_; dropByte(); continue; }
    if (used_ < len+4) return;
    const uint16_t got = uint16_t(buf_[2]) | (uint16_t(buf_[3])<<8);
    if (got != crc(buf_+4,len)) {
      ++badCrcFrames_;
      // Metadata only: packet header (length+received CRC) and calculated CRC.
      // Do NOT dump body bytes: a shifted/corrupt frame can include card data.
      // Bound printing to eight events per payment session to keep UART serviced.
      if (crcTraceThisSession_ < 8) {
        ++crcTraceThisSession_;
        const uint16_t calculated = crc(buf_+4,len);
        Serial.printf("[PM TRACE] RX BAD_CRC n=%lu bodyLen=%u header=%02X-%02X-%02X-%02X "
                      "gotCRC=%04X calcCRC=%04X PM=%s pending=%s (body hidden)\n",
                      (unsigned long)badCrcFrames_, (unsigned)len,
                      (unsigned)buf_[0], (unsigned)buf_[1],
                      (unsigned)buf_[2], (unsigned)buf_[3],
                      (unsigned)got, (unsigned)calculated,
                      stateName(state_), awaitName(awaiting_));
      }
      dropByte(); continue;
    }
    ++goodFrames_;
    uint8_t packet[255]; memcpy(packet,buf_+4,len);
    const size_t tail = used_-(len+4);
    if (tail) memmove(buf_,buf_+len+4,tail);
    used_ = tail; rxStarted_ = millis();
    frame(packet,len);
  }
}
void PaymentManager::setIdleRxMode(IdleRxMode mode) {
  if (state_ != State::IDLE || awaiting_ != Await::NONE || diagPending_) {
    Serial.println("[HYBRID] Mode change refused: busy");
    return;
  }
  idleRxMode_ = mode;
  used_ = 0;
  Serial.printf("[HYBRID] IDLE_RX=%s (payment packets ALWAYS parsed)\n",
                mode == IdleRxMode::NORMAL ? "NORMAL" : "DISCARD_IDLE");
}

bool PaymentManager::diagnosticShowPicture(uint8_t index) {
  if ((index != 1 && index != 2) || state_ != State::IDLE ||
      awaiting_ != Await::NONE || diagPending_ || uncertain_) return false;
  // Flush old incoming background bytes, not financial data.
  while (uart_.available()) {
    const int b = uart_.read();
    if (b < 0) break;
    ++rawRxBytes_; ++windowRx_; ++discardedBytes_; ++windowDropped_;
    if (b == 0xFF) ++windowFF_;
    else if (b == 0x00) ++windowZero_;
  }
  used_ = 0;
  const uint8_t body[] = {0x03, index};
  const uint16_t c = crc(body, sizeof(body));
  const uint8_t packet[] = {2, 0, uint8_t(c), uint8_t(c >> 8), body[0], body[1]};
  diagExpectedAck_ = c;
  diagPending_ = true;
  diagSentAt_ = millis();
  const size_t written = uart_.write(packet, sizeof(packet));
  uart_.flush();
  Serial.printf("[HYBRID] DIAG_TX picture=%u bytes=%u expectedACK=%04X\n",
                (unsigned)index, (unsigned)written, (unsigned)c);
  if (written != sizeof(packet)) {
    diagPending_ = false;
    Serial.println("[HYBRID] DIAG_TX_FAILED");
    return false;
  }
  return true;
}

void PaymentManager::printUartWindow() {
  Serial.printf("[HYBRID] RX_WINDOW ms=%lu bytes=%lu FF=%lu ZERO=%lu OTHER=%lu "
                "discardedIdle=%lu mode=%s diagPending=%u matches=%lu timeouts=%lu\n",
                (unsigned long)(millis()-windowAt_),
                (unsigned long)windowRx_, (unsigned long)windowFF_,
                (unsigned long)windowZero_,
                (unsigned long)(windowRx_-windowFF_-windowZero_),
                (unsigned long)windowDropped_,
                idleRxMode_ == IdleRxMode::NORMAL ? "NORMAL" : "DISCARD_IDLE",
                (unsigned)diagPending_, (unsigned long)diagAckMatches_,
                (unsigned long)diagAckTimeouts_);
  windowAt_ = millis();
  windowRx_ = windowFF_ = windowZero_ = windowDropped_ = 0;
}

void PaymentManager::update() {
  const int pending = uart_.available();
  if (pending > 0 && (uint32_t)pending > peakRxPending_)
    peakRxPending_ = (uint32_t)pending;
  const bool dropIdle = idleRxMode_ == IdleRxMode::DISCARD_IDLE &&
      state_ == State::IDLE && awaiting_ == Await::NONE && !diagPending_;
  size_t budget = 2048;
  while (uart_.available() && budget--) {
    const int b = uart_.read();
    if (b < 0) continue;
    ++windowRx_;
    if (b == 0xFF) ++windowFF_;
    else if (b == 0x00) ++windowZero_;
    if (dropIdle) {
      ++rawRxBytes_; ++discardedBytes_; ++windowDropped_; used_ = 0;
    } else receiveByte(uint8_t(b));
  }
  if (diagPending_ && uint32_t(millis()-diagSentAt_) >= cfg_.ackMs) {
    diagPending_ = false; ++diagAckTimeouts_;
    Serial.printf("[HYBRID] DIAG_ACK_TIMEOUT expected=%04X count=%lu\n",
                  (unsigned)diagExpectedAck_, (unsigned long)diagAckTimeouts_);
  }
  if (used_ && uint32_t(millis()-rxStarted_) >= 1000) {
    ++partialFrameTimeouts_;
    discardedBytes_ += used_;
    used_ = 0;
  }
}
void PaymentManager::onTimeout(Timer fired) {
  if (fired == Timer::NONE || fired != timer_) return;
  // Report BEFORE cancellation: the pending command is otherwise lost in diagnostics.
  Serial.printf("[PM TRACE] TIMEOUT timer=%s pending=%s expectedACK=%04X PM=%s rxGood=%lu rxCRC=%lu rxDiscard=%lu\n",
                timerName(fired), awaitName(awaiting_), (unsigned)expectedAck_,
                stateName(state_), (unsigned long)goodFrames_,
                (unsigned long)badCrcFrames_, (unsigned long)discardedBytes_);
  timer(Timer::NONE);
  if (fired == Timer::ACK) {
    const Await phase = awaiting_;
    if (phase == Await::FINAL_PICTURE) {
      event(Event::PAYMENT_ERROR,Error::DISPLAY_ACK_TIMEOUT);
      finished(false);
    } else {
      const bool financial = phase == Await::VEND_REPORT || phase == Await::REFUND ||
                            phase == Await::READ_CARD || phase == Await::CANCEL_READ;
      fail(Error::ACK_TIMEOUT,financial);
    }
  } else if (fired == Timer::CARD_RESULT) {
    // Terminal did not deliver a no-card event; possible payment status unknown.
    fail(Error::RESULT_TIMEOUT,true);
  } else if (fired == Timer::AUTH_RESULT) {
    fail(Error::AUTH_TIMEOUT,true);
  }
}
