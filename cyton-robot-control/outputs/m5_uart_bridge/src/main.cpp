// M5StickC Plus (gen 1) bridge between the host USB serial port and the Uno
// gateway — version 2: line-paced.
//
// The Uno receives from us on SoftwareSerial, which cannot receive while it
// transmits. Any host command that arrives while the Uno is still printing a
// reply (a full STATUS report takes ~300 ms) is lost or garbled. So this
// bridge no longer copies bytes blindly: host lines are queued and the next
// one is forwarded only after the Uno's terminating reply line (OK…, ERR…,
// PONG, READY) or a timeout. STOP is sent immediately, retried until the Uno
// acknowledges it, and discards any
// pending motion commands. Bytes from the Uno are passed through unchanged, so
// every existing host tool keeps working and a plain serial terminal is safe.
//
// Wiring: G26 = TX to Uno D7, G36 = RX from Uno D8 via 1k/2k divider, GND.

#include <Arduino.h>

namespace {
constexpr uint32_t USB_BAUD = 115200;
constexpr uint32_t UNO_BAUD = 57600;
constexpr int UNO_RX_PIN = 36;
constexpr int UNO_TX_PIN = 26;
constexpr int LED_PIN = 10;                 // active low
constexpr size_t LINE_SIZE = 128;
constexpr size_t QUEUE_SIZE = 16;
constexpr uint32_t REPLY_TIMEOUT_MS = 1500;  // Uno never answered: move on
constexpr uint32_t STOP_RETRY_MS = 120;      // resend STOP while un-acknowledged

HardwareSerial unoLink(2);

struct Line {
  char text[LINE_SIZE];
  size_t length = 0;
};

Line hostLine;                 // line being received from the host
Line unoLine;                  // line being received from the Uno (for terminator detection)
Line queueBuf[QUEUE_SIZE];     // FIFO of complete host lines waiting for the Uno
size_t queueHead = 0, queueCount = 0;
bool inFlight = false;          // a normal command is awaiting its terminator
bool stopPending = false;       // a STOP has been sent and not yet acknowledged with "OK STOP"
uint8_t stopRetries = 0;
uint8_t stopCopies = 0;         // STOP lines written in the current episode
uint8_t extraStopAcks = 0;      // "OK STOP" replies still expected for retried copies
uint32_t settleUntil = 0;       // stop waiting for those extras after this time
uint32_t sentAt = 0;
uint32_t stopSentAt = 0;
constexpr uint8_t STOP_MAX_RETRIES = 25;

bool startsWith(const char *text, const char *prefix) {
  while (*prefix) {
    if (*text++ != *prefix++) return false;
  }
  return true;
}

bool isTerminator(const char *text) {
  return startsWith(text, "OK") || startsWith(text, "ERR") || startsWith(text, "PONG") ||
         startsWith(text, "READY");
}

bool isMotion(const char *text) {
  return startsWith(text, "MOVE") || startsWith(text, "GROUP") || startsWith(text, "ENABLE");
}

bool isStop(const char *text) { return startsWith(text, "STOP"); }

void upper(char *text) {
  for (; *text; ++text) *text = static_cast<char>(toupper(static_cast<unsigned char>(*text)));
}

Line &slot(size_t index) { return queueBuf[(queueHead + index) % QUEUE_SIZE]; }

void queueClearMotion() {
  size_t kept = 0;
  for (size_t i = 0; i < queueCount; ++i) {
    Line &item = slot(i);
    if (!isMotion(item.text)) {
      if (kept != i) slot(kept) = item;
      ++kept;
    }
  }
  queueCount = kept;
}

void queuePushFront(const Line &line) {
  if (queueCount == QUEUE_SIZE) queueCount--;  // drop the newest to make room
  queueHead = (queueHead + QUEUE_SIZE - 1) % QUEUE_SIZE;
  queueBuf[queueHead] = line;
  queueCount++;
}

void queuePushBack(const Line &line) {
  if (queueCount == QUEUE_SIZE) {
    Serial.println(F("M5 ERR queue_full_dropped"));
    return;
  }
  slot(queueCount) = line;
  queueCount++;
}

bool queuePopFront(Line &out) {
  if (queueCount == 0) return false;
  out = queueBuf[queueHead];
  queueHead = (queueHead + 1) % QUEUE_SIZE;
  queueCount--;
  return true;
}

void forward(const Line &line) {
  unoLink.write(reinterpret_cast<const uint8_t *>(line.text), line.length);
  unoLink.write('\n');
  inFlight = true;
  sentAt = millis();
  digitalWrite(LED_PIN, LOW);
}

// STOP bypasses the queue: it is written immediately even if a command is in
// flight (the V2 Uno checks for it between servos and aborts), and resent
// every STOP_RETRY_MS until the Uno answers "OK STOP". Between DYNAMIXEL
// transactions the Uno can receive, so a retry lands within a few hundred ms
// even if the first copy was garbled by a reply in progress.
void sendStop() {
  unoLink.println(F("STOP"));
  stopPending = true;
  stopCopies++;
  stopSentAt = millis();
  digitalWrite(LED_PIN, LOW);
}

void dispatch() {
  if (inFlight || stopPending) return;
  if (extraStopAcks) {
    // Retried STOP copies each earn their own "OK STOP" from the Uno; wait
    // for them (bounded) so they are not mistaken for the next reply.
    if (static_cast<int32_t>(millis() - settleUntil) < 0) return;
    extraStopAcks = 0;
  }
  Line next;
  if (queuePopFront(next)) forward(next);
}

void acceptHostLine() {
  hostLine.text[hostLine.length] = '\0';
  if (hostLine.length == 0) {
    // Bare newline from the host: pass straight through so it can flush the
    // Uno line buffer; it produces no reply worth waiting for.
    unoLink.write('\n');
    return;
  }
  char verb[8] = {0};
  strncpy(verb, hostLine.text, sizeof(verb) - 1);
  upper(verb);
  if (startsWith(verb, "M5")) {           // bridge-local query, never forwarded
    Serial.print(F("M5 BRIDGE_V2 queued="));
    Serial.print(queueCount);
    Serial.print(F(" in_flight="));
    Serial.print(inFlight ? 1 : 0);
    Serial.print(F(" stop_pending="));
    Serial.print(stopPending ? 1 : 0);
    Serial.print(F(" extra_acks="));
    Serial.println(extraStopAcks);
    return;
  }
  Line copy = hostLine;
  memcpy(copy.text, verb, strlen(verb));  // normalise verb case for matching
  if (isStop(copy.text)) {
    queueClearMotion();
    stopRetries = 0;
    stopCopies = 0;
    extraStopAcks = 0;
    sendStop();
    return;
  }
  queuePushBack(copy);
  dispatch();
}

void pollHost() {
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      acceptHostLine();
      hostLine.length = 0;
    } else if (hostLine.length + 1 < LINE_SIZE) {
      hostLine.text[hostLine.length++] = c;
    } else {
      hostLine.length = 0;
      Serial.println(F("M5 ERR line_too_long"));
    }
  }
}

void pollUno() {
  while (unoLink.available()) {
    const char c = static_cast<char>(unoLink.read());
    Serial.write(c);                       // transparent pass-through
    if (c == '\r') continue;
    if (c == '\n') {
      unoLine.text[unoLine.length] = '\0';
      if (unoLine.length && extraStopAcks && startsWith(unoLine.text, "ERR garbled")) {
        // A retried STOP copy that was mangled by a reply in progress: the
        // V2 Uno reports it instead of answering, so it counts as consumed.
        extraStopAcks--;
        dispatch();
      } else if (unoLine.length && isTerminator(unoLine.text)) {
        if (startsWith(unoLine.text, "OK STOP")) {
          if (stopPending) {
            // Acknowledged; an aborted command never sends its own terminator.
            stopPending = false;
            inFlight = false;
            extraStopAcks = stopCopies - 1;
            settleUntil = millis() + 200;
          } else if (extraStopAcks) {
            extraStopAcks--;       // a retried copy answering late: not a terminator
          } else {
            inFlight = false;
          }
        } else {
          inFlight = false;
        }
        digitalWrite(LED_PIN, HIGH);
        dispatch();
      }
      unoLine.length = 0;
    } else if (unoLine.length + 1 < LINE_SIZE) {
      unoLine.text[unoLine.length++] = c;
    } else {
      unoLine.length = 0;
    }
  }
}

void pollTimeouts() {
  const uint32_t now = millis();
  if (stopPending && now - stopSentAt > STOP_RETRY_MS) {
    if (stopRetries++ < STOP_MAX_RETRIES) {
      sendStop();
    } else {
      Serial.println(F("M5 ERR stop_unacknowledged"));
      stopPending = false;
      inFlight = false;
      extraStopAcks = 0;
      dispatch();
    }
  }
  if (inFlight && now - sentAt > REPLY_TIMEOUT_MS) {
    Serial.println(F("M5 ERR no_reply_timeout"));
    inFlight = false;
    digitalWrite(LED_PIN, HIGH);
    dispatch();
  }
}
}  // namespace

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  Serial.begin(USB_BAUD);
  unoLink.begin(UNO_BAUD, SERIAL_8N1, UNO_RX_PIN, UNO_TX_PIN);
  delay(250);
  Serial.println();
  Serial.println(F("M5STICKC_PLUS_UART_BRIDGE_V2 line-paced"));
  Serial.println(F("USB 115200 <-> UNO 57600; G36=RX, G26=TX; STOP has priority"));
  // Our reset (the host opening the port) may have left junk in the Uno's
  // line buffer: terminate it, then make sure the arm is de-energised.
  unoLink.write('\n');
  stopRetries = 0;
  stopCopies = 0;
  sendStop();
}

void loop() {
  pollUno();
  pollHost();
  pollTimeouts();
  dispatch();   // also releases the queue when a settle window expires with no new event
}
