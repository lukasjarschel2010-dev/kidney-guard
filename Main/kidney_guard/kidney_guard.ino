#include "SparkFun_LIS331.h"
#include <Wire.h>
#include <LittleFS.h>
#include <Preferences.h>

LIS331 xl;

// Direct register access for the accelerometer (H3LIS331DL)
const uint8_t LIS331_ADDR = 0x19;
const uint8_t CTRL_REG4_ADDR = 0x23;
const uint8_t STATUS_REG_ADDR = 0x27;
const uint8_t OUT_X_L_ADDR = 0x28;
const uint8_t AUTO_INC = 0x80;   // MSB of sub-address = auto-increment
const uint8_t BDU_BIT = 0x80;    // CTRL_REG4 bit 7: Block Data Update
const uint8_t ZYXDA_BIT = 0x08;  // STATUS_REG bit 3: new X, Y, Z data ready
const uint8_t ZYXOR_BIT = 0x80;  // STATUS_REG bit 7: X, Y, Z data overrun
const uint8_t INT1_SRC_ADDR = 0x31; // reading it clears the latched INT1

const int ACCEL_RANGE_G = 200;      // must match setFullScale()

// A log row is written for a window with an axis at or over this, seen either
//  in a sample or on INT1.
const float IMPACT_THRESHOLD_G = 100.0;

// Accelerometer INT1 is wired to D0. It goes high when |x|, |y| or |z| exceeds
//  IMPACT_THRESHOLD_G and stays high (latched) until INT1_SRC is read. The
//  sensor checks every sample itself, so this also catches impacts in samples
//  that were never read, e.g. while a flash write paused both cores.
const int INT1_PIN = D0;
// While an axis stays over threshold INT1 fires again on every sample, so
//  an impact only counts as over once INT1 has been quiet for this long.
const unsigned long IMPACT_REARM_MS = 20;

// FSR voltage dividers on D8, D9, D10 (GPIO7-9, analog A8-A10), each with
//  a 4.7k pull-up to 3.3V and the FSR to GND, so a reading is ~4095 at rest
//  and falls as the FSR is pressed. All three are streamed; the log and the
//  "Heavy Squeeze" check use all three too.
const int FSR_COUNT = 3;
const int FSR_ADC_MAX = 4095; // 12-bit ADC, also the reading at rest
const int FSR_PINS[FSR_COUNT] = {A8, A9, A10};

// CSV event log on internal flash (LittleFS on the "spiffs" partition of
//  the default partition scheme, ~1.5 MB). Rows are buffered in RAM and
//  written once an event is over, or when the buffer is full. The writing
//  is done by logTask on core 0 while loop() fills a second buffer, since a
//  flush takes ~0.4 s on a nearly full partition and would stall sampling.
//  Even so, the ESP32 pauses both cores for each flash erase or program,
//  so a flush still costs some samples, just far fewer.
// The log is a ring buffer of numbered segment files (/log/00000001.csv,
//  ...) holding rows only, the header is added when dumping. When the
//  partition is full the oldest segment is deleted, so the newest data is
//  always kept and at most one segment of old rows is lost at a time.
const char *LOG_DIR = "/log";
const char *LEGACY_LOG_PATH = "/log.csv";      // single-file log, pre ring buffer
const char *LOG_HEADER =
  "boot,ms,x_g,y_g,z_g,combined_g,peak_axis_g,peak_axis,"
  "fsr_d8_adc,fsr_d9_adc,fsr_d10_adc,missed_peak\n";
const size_t LOG_BUFFER_SIZE = 4096;
const size_t LOG_SEGMENT_SIZE = 65536;         // ~1100 rows, ~4% of the log
const unsigned long LOG_FLUSH_QUIET_MS = 1000; // flush after 1 s w/o events
const size_t LOG_FREE_MARGIN = 8192;           // keep 2 flash blocks spare
char logBuffers[2][LOG_BUFFER_SIZE];
int logFill = 0;            // buffer loop() is filling
size_t logLen = 0;          // bytes in it
struct LogWrite
{
  int buffer;
  size_t len;
};
QueueHandle_t logQueue;     // full buffers for logTask to write
SemaphoreHandle_t logSpare; // available while the other buffer is free,
                            //  i.e. logTask isn't writing it
uint32_t logRowsDropped = 0; // rows lost because both buffers were full
bool logReady = false;
uint32_t oldestSegment = 1; // segment numbers only grow, so oldest..newest
uint32_t newestSegment = 1; //  is the order of the rows
// Bytes in the newest segment, kept by writeLog() so a flush doesn't have
//  to look it up. SIZE_MAX until read once after boot or erase: that first
//  write also makes sure the rest of the segment fits on the partition.
size_t newestSegmentSize = SIZE_MAX;
uint32_t bootCount = 0; // distinguishes power cycles, since ms restarts at 0

// Stream mode (serial command 's', back to text with 't'): every raw
//  sample goes out as binary packets for Tools/plot_stream.py, which sends
//  's' itself when it connects. The live text rows and diagnostics are
//  not printed then, since they would garble the Serial Monitor view
//  anyway; the CSV log on flash keeps running as normal.
// Packets are queued for streamTask on core 0, so loop() never waits on USB
//  and a short stall or log flush doesn't lose any. If the queue is full
//  the packet is dropped whole (the host sees the gap in seq).
// Command 'r<N>' (e.g. "r10\n") streams only one of every N sensor samples,
//  see streamSample(). The sensor, impact detection and the log keep
//  running at the full 1 kHz.
// Packet layout, little-endian:
//  0xA5 0x5C        sync, the second byte names the layout
//  uint16 seq       increments per packet, so the host can spot drops
//  uint32 micros    micros() when the packet's last sample was read
//  uint16 fullScale +/- g range of the counts below
//  uint8  count     samples in this packet (packetTarget())
//  uint8  overruns  samples the sensor overwrote before we read them
//  count times:
//   int16  x,y,z    raw 12-bit accelerometer counts
//   uint16 fsr[3]   12-bit ADC readings of D8, D9, D10
//  uint8  checksum  sum of all bytes after the sync, mod 256
const uint8_t SYNC0 = 0xA5;
const uint8_t SYNC1 = 0x5C;
const int SAMPLES_PER_PACKET = 10; // at full rate: 10 ms per packet,
                                   //  100 packets/sec
const int SAMPLE_BYTES = 3 * 2 + FSR_COUNT * 2;
const int PACKET_BYTES = 2 + 2 + 4 + 2 + 1 + 1 + SAMPLES_PER_PACKET * SAMPLE_BYTES + 1;
const int STREAM_QUEUE_PACKETS = 32;    // ~320 ms of slack at full rate
const int STREAM_STALL_LIMIT_MS = 50;   // host gone if no byte goes out for this
struct StreamPacket
{
  uint16_t len;
  uint8_t bytes[PACKET_BYTES];
};
QueueHandle_t streamQueue; // packets for streamTask to send
bool streamMode = false;
StreamPacket packet;    // the one being filled
int packetSamples = 0;  // samples in the packet being filled
uint8_t packetOverruns = 0;
uint16_t packetSeq = 0;
const int MAX_STREAM_DECIMATION = 1000; // down to 1 sample/sec
int streamDecimation = 1;     // stream one of every N sensor samples
int groupSamples = 0;         // sensor samples seen for the next streamed one
int32_t groupMag2 = 0;        // largest x^2 + y^2 + z^2 in the group, counts
int16_t groupXyz[3];          // the sample it belongs to
uint16_t groupFsr[FSR_COUNT]; // lowest reading of each FSR in the group

void setup() 
{
  // put your setup code here, to run once:
  for (int i = 0; i < FSR_COUNT; i++)
  {
    pinMode(FSR_PINS[i], INPUT); // FSR analog inputs
  }
  pinMode(INT1_PIN, INPUT); // Accelerometer INT1
  Wire.begin();           // Turns on I2C communication
  Wire.setClock(400000);  // 400 kHz fast-mode I2C, so STATUS_REG can be
                          //  polled several times per 1 ms sample
  xl.setI2CAddr(LIS331_ADDR);    // This MUST be called BEFORE .begin() so 
                          //  .begin() can communicate with the chip
  xl.begin(LIS331::USE_I2C); // Selects the bus to be used and sets
                          //  the power up bit on the accelerometer.
                          //  Also zeroes out all accelerometer
                          //  registers that are user writable.
  // New samples are detected by polling STATUS_REG (ZYXDA). INT1 is only
  //  used to count impacts and log them, see below.
  xl.setFullScale(LIS331::MED_RANGE); // +/-200g
  xl.setODR(LIS331::DR_1000HZ); // Accelerometer output data rate: 1000 Hz
  enableBlockDataUpdate(); // MUST be called AFTER setFullScale(), which
                          //  clears bit 7 of CTRL_REG4
  // This next section configures an interrupt. It will cause pin
  //  INT1 on the accelerometer to go high when the absolute value
  //  of the reading on any axis exceeds IMPACT_THRESHOLD_G.
  xl.intSrcConfig(LIS331::INT_SRC, 1); // Select the source of the
                          //  signal which appears on pin INT1. In
                          //  this case, we want the corresponding
                          //  interrupt's status to appear.
  xl.latchInterrupt(true, 1); // Hold INT1 high until INT1_SRC is
                          //  read, so we count each event once and
                          //  can't miss a short one. Must come AFTER
                          //  intSrcConfig(), which clears this bit.
  xl.setIntDuration(0, 1); // Fire on the first sample over threshold.
  // Threshold register is 7 bits, compared against counts/16. At
  //  +/-200g one register LSB is 16 * 200/2047 = ~1.56g.
  const float gPerThsLsb = 16.0 * ACCEL_RANGE_G / 2047;
  int ths = (int)(IMPACT_THRESHOLD_G / gPerThsLsb + 0.5);
  if (ths > 127) ths = 127;
  xl.setIntThreshold(ths, 1);
  // INT1_CFG's AOI bit is left at 0 (by begin()), so the axes are
  //  OR-combined: any one axis over threshold fires INT1.
  xl.enableInterrupt(LIS331::X_AXIS, LIS331::TRIG_ON_HIGH, 1, true);
  xl.enableInterrupt(LIS331::Y_AXIS, LIS331::TRIG_ON_HIGH, 1, true);
  xl.enableInterrupt(LIS331::Z_AXIS, LIS331::TRIG_ON_HIGH, 1, true);
  Serial.begin(115200);
  unsigned long serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 2000)
  {
    delay(10); // Wait up to 2 s for native USB so the check below is seen
  }

  // Count boots in NVS so rows from different power cycles can be told apart
  Preferences prefs;
  prefs.begin("kidney_guard");
  bootCount = prefs.getUInt("boot", 0) + 1;
  prefs.putUInt("boot", bootCount);
  prefs.end();

  logReady = LittleFS.begin(true); // formats the partition if mount fails
  if (logReady)
  {
    scanLogSegments();
    migrateLegacyLog();
    logQueue = xQueueCreate(1, sizeof(LogWrite));
    logSpare = xSemaphoreCreateBinary();
    xSemaphoreGive(logSpare);
    // loop() runs on core 1, so flash writes don't hold it up
    xTaskCreatePinnedToCore(logTask, "log", 8192, NULL, 1, NULL, 0);
  }
  printLogInfo();
  Serial.println("# Commands: d = dump log, e = erase log, i = log info, "
                 "s = stream to plot_stream.py, t = text output, "
                 "r<N> = stream every Nth sample");
  Serial.print(LOG_HEADER); // header for the live rows

  // Verify accelerometer configuration: expect BDU = 1 and FS = 01 (200g)
  uint8_t ctrlReg4;
  if (readRegister(CTRL_REG4_ADDR, ctrlReg4))
  {
    Serial.print("CTRL_REG4: 0b");
    for (int bit = 7; bit >= 0; bit--)
    {
      Serial.print((ctrlReg4 >> bit) & 1);
    }
    Serial.print(" (0x");
    Serial.print(ctrlReg4, HEX);
    Serial.print(") BDU: ");
    Serial.print((ctrlReg4 & BDU_BIT) ? "ON" : "OFF");
    Serial.print(" FS bits: ");
    Serial.print((ctrlReg4 >> 5) & 1);
    Serial.println((ctrlReg4 >> 4) & 1);
  }
  else
  {
    Serial.println("# I2C ERROR - could not read CTRL_REG4");
  }

  // Clear INT1 if it latched before this boot, the sensor keeps it through
  //  an MCU reset, so it doesn't log a row for an impact long past
  uint8_t staleIntSource;
  readRegister(INT1_SRC_ADDR, staleIntSource);

  Serial.flush();           // let the startup messages above go out first
  Serial.setTxTimeoutMs(0); // Never block on Serial from here on: with USB
                            //  plugged in but no program reading the port,
                            //  prints would otherwise stall the loop for up
                            //  to 100 ms each. Unread output is dropped.

  streamQueue = xQueueCreate(STREAM_QUEUE_PACKETS, sizeof(StreamPacket));
  // loop() runs on core 1, so USB writes don't hold it up
  xTaskCreatePinnedToCore(streamTask, "stream", 4096, NULL, 1, NULL, 0);
}

const unsigned long PRINT_INTERVAL_MS = 100;    // peak window, max 10 prints/sec
const unsigned long DIAG_INTERVAL_MS = 1000;    // diagnostics, 1 print/sec

// A window's peak is only printed if IMPACT_THRESHOLD_G or this is reached
const int FSR_HEAVY_ADC = 2030;              // "Heavy Squeeze", at or below

void loop()
{
  static unsigned long printTimer = 0;
  static unsigned long diagTimer = 0;
  static unsigned long lastEventMs = 0;
  static float maxGx = 0, maxGy = 0, maxGz = 0;
  static float maxCombined = 0;
  static float maxAxisG = 0;            // largest |x|, |y| or |z| in window
  static float peakAxisG = 0;           // that reading with its sign
  static char peakAxis = 'x';           // axis it was measured on
  static bool int1Fired = false;        // INT1 was high during the window
  static int minFsrAdc[FSR_COUNT] = {FSR_ADC_MAX, FSR_ADC_MAX, FSR_ADC_MAX};
                                        // lowest reading = hardest press
  static unsigned int pollCount = 0;    // STATUS_REG polls since diagnostics
  static unsigned int newDataCount = 0; // new sensor samples processed
  static unsigned int overrunCount = 0; // polls where a sample was missed
  static unsigned int intEventCount = 0; // impacts seen on INT1
  static unsigned int i2cErrorCount = 0; // failed accelerometer reads
  static bool inImpact = false;
  static unsigned long lastIntMs = 0;

  if (digitalRead(INT1_PIN) == HIGH)
  {
    uint8_t intSource;
    if (!readRegister(INT1_SRC_ADDR, intSource)) // clears the latch
    {
      i2cErrorCount++; // still latched, so the next pass reads it again
    }
    if (!inImpact)
    {
      intEventCount++; // first INT1 of a new impact
      inImpact = true;
    }
    int1Fired = true;
    lastIntMs = millis();
  }
  else if (inImpact && millis() - lastIntMs >= IMPACT_REARM_MS)
  {
    inImpact = false; // back under threshold, ready for the next impact
  }

  // Software data-ready: poll STATUS_REG every loop pass and only read the
  //  axes when it says a new sample is ready.
  //  Reading STATUS_REG alone does not clear ZYXDA, only reading the output
  //  registers does, so a sample arriving mid-read can't be lost unseen.
  //  A failed read is skipped rather than returning from loop(), so the
  //  timers, log and serial commands keep running if the sensor drops out.
  uint8_t status = 0;
  if (!readRegister(STATUS_REG_ADDR, status))
  {
    i2cErrorCount++;
  }
  pollCount++;
  if (status & ZYXOR_BIT)
  {
    overrunCount++;  // a sample was overwritten before it was read
  }
  int16_t x, y, z;
  bool newSample = false;
  if (status & ZYXDA_BIT)
  {
    newSample = readAxesBurst(x, y, z);
    if (!newSample)
    {
      i2cErrorCount++; // ZYXDA stays set, so the next pass tries again
    }
  }
  if (newSample)
  {
    newDataCount++;
    uint16_t fsr[FSR_COUNT];
    for (int i = 0; i < FSR_COUNT; i++)
    {
      fsr[i] = analogRead(FSR_PINS[i]); // 12-bit ADC (0 to 4095)
    }
    if (streamMode)
    {
      streamSample(x, y, z, fsr, status & ZYXOR_BIT);
    }
    float gx = xl.convertToG(ACCEL_RANGE_G, x); // Convert raw counts to
    float gy = xl.convertToG(ACCEL_RANGE_G, y); //  g-force, using the
    float gz = xl.convertToG(ACCEL_RANGE_G, z); //  current full scale.
    float combined = sqrt(gx * gx + gy * gy + gz * gz);
    if (combined > maxCombined)
    {
      maxCombined = combined;
      maxGx = gx;
      maxGy = gy;
      maxGz = gz;
    }
    float axisG = gx; // largest single-axis reading of this sample, signed
    char axis = 'x';
    if (fabsf(gy) > fabsf(axisG))
    {
      axisG = gy;
      axis = 'y';
    }
    if (fabsf(gz) > fabsf(axisG))
    {
      axisG = gz;
      axis = 'z';
    }
    if (fabsf(axisG) > maxAxisG)
    {
      maxAxisG = fabsf(axisG);
      peakAxisG = axisG;
      peakAxis = axis;
    }

    for (int i = 0; i < FSR_COUNT; i++)
    {
      if (fsr[i] < minFsrAdc[i])
      {
        minFsrAdc[i] = fsr[i];
      }
    }
  }

  if (millis() - printTimer >= PRINT_INTERVAL_MS)
  {
    printTimer = millis();
    bool heavySqueeze = false;
    for (int i = 0; i < FSR_COUNT; i++)
    {
      heavySqueeze = heavySqueeze || minFsrAdc[i] <= FSR_HEAVY_ADC;
    }
    if (maxAxisG >= IMPACT_THRESHOLD_G || int1Fired || heavySqueeze)
    {
      // INT1 saw an impact but no sample did, so the row's peak is too low
      bool missedPeak = int1Fired && maxAxisG < IMPACT_THRESHOLD_G;
      char row[160];
      int len = snprintf(row, sizeof(row),
        "%lu,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%c,%d,%d,%d,%d\n",
        (unsigned long)bootCount, printTimer, maxGx, maxGy, maxGz,
        maxCombined, peakAxisG, peakAxis,
        minFsrAdc[0], minFsrAdc[1], minFsrAdc[2], missedPeak ? 1 : 0);
      len = min(len, (int)sizeof(row) - 1); // snprintf returns untruncated length
      if (!streamMode)
      {
        Serial.print(row);
      }
      appendLog(row, len);
      lastEventMs = printTimer;
    }
    maxGx = maxGy = maxGz = 0;
    maxCombined = 0;
    maxAxisG = 0;
    peakAxisG = 0;
    peakAxis = 'x';
    for (int i = 0; i < FSR_COUNT; i++)
    {
      minFsrAdc[i] = FSR_ADC_MAX;
    }
    int1Fired = false;
  }

  if (millis() - diagTimer >= DIAG_INTERVAL_MS)
  {
    unsigned long now = millis();
    unsigned long elapsedMs = now - diagTimer;
    diagTimer = now;
    if (!streamMode)
    {
      // Sensor sample rate over the actual window (expect ~1000 Hz)
      Serial.print("[diag] samples: ");
      Serial.print(newDataCount);
      Serial.print(" rate: ");
      Serial.print(newDataCount * 1000UL / elapsedMs);
      Serial.print("Hz polls: ");
      Serial.print(pollCount);
      Serial.print(" overrun: ");
      Serial.print(overrunCount);
      Serial.print(" int events: ");
      Serial.print(intEventCount);
      Serial.print(" i2c errs: ");
      Serial.println(i2cErrorCount);
    }
    pollCount = 0;
    newDataCount = 0;
    overrunCount = 0;
    intEventCount = 0;
    i2cErrorCount = 0;
  }

  // Write buffered rows to flash once the event is over (no new rows for
  //  LOG_FLUSH_QUIET_MS), so the write never lands mid-impact. If logTask
  //  is still busy, this tries again on the next pass.
  if (logLen > 0 && millis() - lastEventMs >= LOG_FLUSH_QUIET_MS)
  {
    queueLogFlush();
  }

  if (Serial.available())
  {
    handleCommand(Serial.read());
  }
}

// Reads a single accelerometer register. Returns false on an I2C error,
//  leaving value untouched. Only requestFrom() can report one: with
//  endTransmission(false) nothing is sent until then, so it always returns 0.
bool readRegister(uint8_t reg, uint8_t &value)
{
  Wire.beginTransmission(LIS331_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);    // repeated START, keep the bus
  if (Wire.requestFrom(LIS331_ADDR, (uint8_t)1) != 1)
  {
    return false;
  }
  value = Wire.read();
  return true;
}

// Sets the BDU bit so the output registers are not updated until both
//  the low and high byte of the current sample have been read. This
//  prevents torn readings that mix bytes from two samples.
void enableBlockDataUpdate()
{
  uint8_t reg;
  if (!readRegister(CTRL_REG4_ADDR, reg))
  {
    return; // don't write a guess; the check in setup() reports the failure
  }

  Wire.beginTransmission(LIS331_ADDR);
  Wire.write(CTRL_REG4_ADDR);
  Wire.write(reg | BDU_BIT);
  Wire.endTransmission();
}

// Reads all six output bytes (X_L, X_H, Y_L, Y_H, Z_L, Z_H) in a single
//  I2C transaction using register auto-increment, instead of the six
//  separate transactions done by xl.readAxes(). Returns false on an I2C
//  error, leaving x, y and z untouched.
bool readAxesBurst(int16_t &x, int16_t &y, int16_t &z)
{
  uint8_t data[6];
  Wire.beginTransmission(LIS331_ADDR);
  Wire.write(OUT_X_L_ADDR | AUTO_INC);
  Wire.endTransmission(false);    // repeated START, keep the bus
  if (Wire.requestFrom(LIS331_ADDR, (uint8_t)6) != 6)
  {
    return false;
  }
  for (int i = 0; i < 6; i++)
  {
    data[i] = Wire.read();
  }
  // The data is 12-bit, left justified, so shift right by four after
  //  combining to get a signed value.
  x = (int16_t)(data[0] | data[1] << 8) >> 4;
  y = (int16_t)(data[2] | data[3] << 8) >> 4;
  z = (int16_t)(data[4] | data[5] << 8) >> 4;
  return true;
}

// Collects streamDecimation sensor samples and streams the one with the
//  largest |a|, so decimation doesn't hide a short impact peak. Each FSR
//  sends its lowest reading of the group, its hardest press. Overruns are
//  counted for every sensor sample, streamed or not.
void streamSample(int16_t x, int16_t y, int16_t z,
                  const uint16_t fsr[FSR_COUNT], bool overrun)
{
  if (overrun && packetOverruns < 255)
  {
    packetOverruns++;
  }
  int32_t mag2 = (int32_t)x * x + (int32_t)y * y + (int32_t)z * z;
  if (groupSamples == 0 || mag2 > groupMag2)
  {
    groupMag2 = mag2;
    groupXyz[0] = x;
    groupXyz[1] = y;
    groupXyz[2] = z;
  }
  for (int i = 0; i < FSR_COUNT; i++)
  {
    if (groupSamples == 0 || fsr[i] < groupFsr[i])
    {
      groupFsr[i] = fsr[i];
    }
  }
  if (++groupSamples < streamDecimation)
  {
    return;
  }
  groupSamples = 0;
  addPacketSample(groupXyz, groupFsr);
}

// Samples per packet: SAMPLES_PER_PACKET at full rate, fewer when
//  decimating, so a packet still goes out about every 10 ms (or with every
//  sample, below 100 samples/sec) instead of the plot lagging behind.
int packetTarget()
{
  return max(1, SAMPLES_PER_PACKET / streamDecimation);
}

// Adds a sample to the stream packet and queues the packet when it is full.
void addPacketSample(const int16_t xyz[3], const uint16_t fsr[FSR_COUNT])
{
  uint32_t sampleMicros = micros();
  uint8_t *sample = &packet.bytes[12 + packetSamples * SAMPLE_BYTES];
  memcpy(sample, xyz, 3 * sizeof(int16_t)); // ESP32 is LE
  memcpy(sample + 3 * sizeof(int16_t), fsr, FSR_COUNT * sizeof(uint16_t));
  if (++packetSamples < packetTarget())
  {
    return;
  }
  int packetBytes = 12 + packetSamples * SAMPLE_BYTES + 1;
  packet.bytes[0] = SYNC0;
  packet.bytes[1] = SYNC1;
  packet.bytes[2] = packetSeq & 0xFF;
  packet.bytes[3] = packetSeq >> 8;
  memcpy(&packet.bytes[4], &sampleMicros, 4);
  packet.bytes[8] = ACCEL_RANGE_G & 0xFF;
  packet.bytes[9] = ACCEL_RANGE_G >> 8;
  packet.bytes[10] = packetSamples;
  packet.bytes[11] = packetOverruns;
  uint8_t sum = 0;
  for (int i = 2; i < packetBytes - 1; i++)
  {
    sum += packet.bytes[i];
  }
  packet.bytes[packetBytes - 1] = sum;
  packet.len = packetBytes;
  xQueueSend(streamQueue, &packet, 0); // full: drop it, seq shows the gap
  packetSeq++;
  packetSamples = 0;
  packetOverruns = 0;
}

// Core 0: sends each queued packet, which can take several USB writes since
//  it may be larger than the USB buffer. The packet stays in the queue until
//  it is sent, so an empty queue means nothing is on its way, which is what
//  finishStreamWrites() waits for.
void streamTask(void *)
{
  StreamPacket p;
  for (;;)
  {
    xQueuePeek(streamQueue, &p, portMAX_DELAY);
    size_t sent = 0;
    int stallMs = 0;
    while (sent < p.len && stallMs < STREAM_STALL_LIMIT_MS)
    {
      size_t n = Serial.write(p.bytes + sent, p.len - sent);
      sent += n;
      if (n == 0)
      {
        stallMs++;
        vTaskDelay(pdMS_TO_TICKS(1));
      }
      else
      {
        stallMs = 0;
      }
    }
    if (sent < p.len)
    {
      // Nobody is reading, so drop the backlog rather than send stale
      //  packets later. A packet cut short here fails its checksum.
      xQueueReset(streamQueue);
    }
    else
    {
      xQueueReceive(streamQueue, &p, 0);
    }
  }
}

// Waits until streamTask has sent everything queued, so text printed next
//  can't land in the middle of a packet.
void finishStreamWrites()
{
  while (uxQueueMessagesWaiting(streamQueue) > 0)
  {
    delay(1);
  }
}

void segmentPath(char *path, size_t size, uint32_t segment)
{
  snprintf(path, size, "%s/%08lu.csv", LOG_DIR, (unsigned long)segment);
}

// Finds the oldest and newest segment files left by previous boots.
void scanLogSegments()
{
  if (!LittleFS.exists(LOG_DIR))
  {
    LittleFS.mkdir(LOG_DIR);
  }
  uint32_t oldest = 0, newest = 0;
  File dir = LittleFS.open(LOG_DIR);
  for (File f = dir.openNextFile(); f; f = dir.openNextFile())
  {
    const char *name = strrchr(f.name(), '/'); // name() may include the path
    uint32_t segment = strtoul(name ? name + 1 : f.name(), NULL, 10);
    f.close();
    if (segment == 0)
    {
      continue; // not a segment file
    }
    if (oldest == 0 || segment < oldest)
    {
      oldest = segment;
    }
    if (segment > newest)
    {
      newest = segment;
    }
  }
  dir.close();
  oldestSegment = oldest ? oldest : 1;
  newestSegment = newest ? newest : 1;
}

// Moves the rows of the old single-file log (/log.csv) into the newest
//  segment, so updating the firmware doesn't lose them.
void migrateLegacyLog()
{
  if (!LittleFS.exists(LEGACY_LOG_PATH))
  {
    return;
  }
  File in = LittleFS.open(LEGACY_LOG_PATH, FILE_READ);
  in.readStringUntil('\n'); // segments hold rows only, drop the header
  char path[32];
  segmentPath(path, sizeof(path), newestSegment);
  File out = LittleFS.open(path, FILE_APPEND);
  uint8_t chunk[256];
  while (in.available())
  {
    size_t n = in.read(chunk, sizeof(chunk));
    out.write(chunk, n);
  }
  out.close();
  in.close();
  LittleFS.remove(LEGACY_LOG_PATH);
}

// Adds a CSV row to the RAM buffer. Only flushes here if the buffer is
//  full (a very long event), otherwise the flush waits for a quiet period.
//  If logTask is still writing the other buffer the row is dropped, which
//  takes a buffer filling faster than one flush.
void appendLog(const char *row, int len)
{
  if (!logReady || len <= 0)
  {
    return;
  }
  if (logLen + len > LOG_BUFFER_SIZE && !queueLogFlush())
  {
    logRowsDropped++;
    return;
  }
  memcpy(logBuffers[logFill] + logLen, row, len);
  logLen += len;
}

// Hands the filled buffer to logTask and switches to the other one.
//  Returns false, keeping the rows, while logTask is still writing.
bool queueLogFlush()
{
  if (!logReady || logLen == 0)
  {
    return true;
  }
  if (xSemaphoreTake(logSpare, 0) != pdTRUE)
  {
    return false;
  }
  LogWrite w = {logFill, logLen};
  xQueueSend(logQueue, &w, portMAX_DELAY); // free, logSpare guarantees it
  logFill ^= 1;
  logLen = 0;
  return true;
}

// Hands over any buffered rows and waits until logTask has written them,
//  so the log files can be read or removed.
void finishLogWrites()
{
  if (!logReady)
  {
    return;
  }
  while (!queueLogFlush())
  {
    delay(1); // logTask is busy with the other buffer
  }
  xSemaphoreTake(logSpare, portMAX_DELAY); // free again once it's done
  xSemaphoreGive(logSpare);
}

// Core 0: writes each buffer loop() hands over, then frees it.
void logTask(void *)
{
  LogWrite w;
  for (;;)
  {
    xQueueReceive(logQueue, &w, portMAX_DELAY);
    writeLog(logBuffers[w.buffer], w.len);
    xSemaphoreGive(logSpare);
  }
}

// Appends rows to the newest segment. Starts a new segment when the
//  current one is full, and deletes the oldest segments until the rows
//  fit, so the log never stops and always keeps the newest data.
// Free space is only checked when a segment starts (and on the first
//  write after boot), for the whole rest of the segment: usedBytes() walks
//  the entire filesystem, and every flash read pauses loop() on the other
//  core too, so checking on every flush cost ~200 samples each time.
void writeLog(const char *rows, size_t len)
{
  char path[32];
  segmentPath(path, sizeof(path), newestSegment);
  bool checkRoom = false;
  if (newestSegmentSize == SIZE_MAX)
  {
    newestSegmentSize = 0;
    if (LittleFS.exists(path))
    {
      File f = LittleFS.open(path, FILE_READ);
      newestSegmentSize = f.size();
      f.close();
    }
    checkRoom = true;
  }
  if (newestSegmentSize > 0 && newestSegmentSize + len > LOG_SEGMENT_SIZE)
  {
    newestSegment++;
    segmentPath(path, sizeof(path), newestSegment);
    newestSegmentSize = 0;
    checkRoom = true;
  }
  if (checkRoom)
  {
    size_t needed = LOG_SEGMENT_SIZE - newestSegmentSize + LOG_FREE_MARGIN;
    while (LittleFS.totalBytes() - LittleFS.usedBytes() < needed
           && oldestSegment < newestSegment)
    {
      char oldPath[32];
      segmentPath(oldPath, sizeof(oldPath), oldestSegment);
      LittleFS.remove(oldPath);
      oldestSegment++;
    }
    if (LittleFS.totalBytes() - LittleFS.usedBytes() < needed)
    {
      // only possible if something else fills the partition; check
      //  again next time rather than write into a full partition
      newestSegmentSize = SIZE_MAX;
      Serial.println("# LOG ERROR - no space left, rows dropped");
      return;
    }
  }
  File f = LittleFS.open(path, FILE_APPEND);
  if (!f)
  {
    Serial.println("# LOG ERROR - could not open log file");
    return;
  }
  newestSegmentSize += f.write((const uint8_t *)rows, len);
  f.close();
}

void printLogInfo()
{
  if (!logReady)
  {
    Serial.println("# LOG ERROR - LittleFS mount failed, logging disabled");
    return;
  }
  Serial.print("# log: boot ");
  Serial.print(bootCount);
  Serial.print(", segments ");
  Serial.print(oldestSegment);
  Serial.print("-");
  Serial.print(newestSegment);
  Serial.print(", ");
  Serial.print(LittleFS.usedBytes());
  Serial.print(" / ");
  Serial.print(LittleFS.totalBytes());
  Serial.print(" bytes used, ");
  Serial.print(logRowsDropped);
  Serial.println(" rows dropped (log busy)");
}

// Serial commands. Dumping blocks sampling while it runs; sampling
//  resyncs automatically afterwards.
void handleCommand(char c)
{
  if (c == 'd')
  {
    finishStreamWrites();
    if (!logReady)
    {
      Serial.println("# LOG ERROR - LittleFS mount failed, no log");
      return;
    }
    finishLogWrites();
    Serial.setTxTimeoutMs(100); // dump is requested by a reader, so wait for
                                //  it rather than drop parts of the file
    Serial.println("# BEGIN " + String(LOG_DIR));
    Serial.print(LOG_HEADER);
    uint8_t chunk[256];
    for (uint32_t segment = oldestSegment; segment <= newestSegment; segment++)
    {
      char path[32];
      segmentPath(path, sizeof(path), segment);
      if (!LittleFS.exists(path))
      {
        continue; // newest segment not written yet
      }
      File f = LittleFS.open(path, FILE_READ);
      while (f.available())
      {
        size_t n = f.read(chunk, sizeof(chunk));
        Serial.write(chunk, n);
      }
      f.close();
    }
    Serial.println("# END");
    Serial.flush();
    Serial.setTxTimeoutMs(0);
  }
  else if (c == 'e')
  {
    finishStreamWrites();
    logLen = 0;
    finishLogWrites(); // a write in progress would recreate its segment
    for (uint32_t segment = oldestSegment; segment <= newestSegment; segment++)
    {
      char path[32];
      segmentPath(path, sizeof(path), segment);
      LittleFS.remove(path);
    }
    oldestSegment = newestSegment = 1;
    newestSegmentSize = SIZE_MAX;
    Serial.println("# log erased");
    Serial.print(LOG_HEADER); // header for the live rows
  }
  else if (c == 'i')
  {
    finishStreamWrites();
    printLogInfo();
  }
  else if (c == 's')
  {
    groupSamples = 0; // start with a fresh packet
    packetSamples = 0;
    packetOverruns = 0;
    streamMode = true;
  }
  else if (c == 'r')
  {
    Serial.setTimeout(20); // the digits arrive right after the 'r'
    long n = Serial.parseInt(); // leaves the '\n', ignored as a command
    streamDecimation = constrain(n, 1, MAX_STREAM_DECIMATION);
    groupSamples = 0; // the packet size may change, start a fresh one
    packetSamples = 0;
    packetOverruns = 0;
  }
  else if (c == 't')
  {
    if (streamMode)
    {
      streamMode = false;
      finishStreamWrites();
      Serial.println();  // end any partial packet bytes in the monitor
      Serial.println("# text output");
      Serial.print(LOG_HEADER); // header for the live rows
    }
  }
}
