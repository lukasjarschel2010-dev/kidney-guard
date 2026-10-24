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

// FSR voltage divider on D8 (GPIO7, analog A8) with a 10k pull-down to GND
const int FSR_PIN = A8;

// CSV event log on internal flash (LittleFS on the "spiffs" partition of
//  the default partition scheme, ~1.5 MB). Rows are buffered in RAM and
//  written once an event is over, so flash writes don't stall sampling
//  during an impact.
// The log is a ring buffer of numbered segment files (/log/00000001.csv,
//  ...) holding rows only, the header is added when dumping. When the
//  partition is full the oldest segment is deleted, so the newest data is
//  always kept and at most one segment of old rows is lost at a time.
const char *LOG_DIR = "/log";
const char *LEGACY_LOG_PATH = "/log.csv";      // single-file log, pre ring buffer
const char *LOG_HEADER =
  "boot,ms,x_g,y_g,z_g,combined_g,peak_axis_g,peak_axis,fsr_adc,fsr_v\n";
const size_t LOG_BUFFER_SIZE = 4096;
const size_t LOG_SEGMENT_SIZE = 65536;         // ~1500 rows, ~4% of the log
const unsigned long LOG_FLUSH_QUIET_MS = 1000; // flush after 1 s w/o events
const size_t LOG_FREE_MARGIN = 8192;           // keep 2 flash blocks spare
char logBuffer[LOG_BUFFER_SIZE];
size_t logLen = 0;
bool logReady = false;
uint32_t oldestSegment = 1; // segment numbers only grow, so oldest..newest
uint32_t newestSegment = 1; //  is the order of the rows
uint32_t bootCount = 0; // distinguishes power cycles, since ms restarts at 0

void setup() 
{
  // put your setup code here, to run once:
  pinMode(FSR_PIN, INPUT); // FSR analog input
  Wire.begin();           // Turns on I2C communication
  Wire.setClock(400000);  // 400 kHz fast-mode I2C, so STATUS_REG can be
                          //  polled several times per 1 ms sample
  xl.setI2CAddr(LIS331_ADDR);    // This MUST be called BEFORE .begin() so 
                          //  .begin() can communicate with the chip
  xl.begin(LIS331::USE_I2C); // Selects the bus to be used and sets
                          //  the power up bit on the accelerometer.
                          //  Also zeroes out all accelerometer
                          //  registers that are user writable.
  // INT1/INT2 are not wired to the XIAO, so no hardware interrupts are
  //  configured. New samples are detected by polling STATUS_REG (ZYXDA).
  xl.setFullScale(LIS331::MED_RANGE); // +/-200g
  xl.setODR(LIS331::DR_1000HZ); // Accelerometer output data rate: 1000 Hz
  enableBlockDataUpdate(); // MUST be called AFTER setFullScale(), which
                          //  clears bit 7 of CTRL_REG4
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
  }
  printLogInfo();
  Serial.println("# Commands: d = dump log, e = erase log, i = log info");
  Serial.print(LOG_HEADER); // header for the live rows

  // Verify accelerometer configuration: expect BDU = 1 and FS = 01 (200g)
  uint8_t ctrlReg4 = readRegister(CTRL_REG4_ADDR);
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

  Serial.flush();           // let the startup messages above go out first
  Serial.setTxTimeoutMs(0); // Never block on Serial from here on: with USB
                            //  plugged in but no program reading the port,
                            //  prints would otherwise stall the loop for up
                            //  to 100 ms each. Unread output is dropped.
}

const int ACCEL_RANGE_G = 200;                  // must match setFullScale()
const unsigned long PRINT_INTERVAL_MS = 100;    // peak window, max 10 prints/sec
const unsigned long DIAG_INTERVAL_MS = 1000;    // diagnostics, 1 print/sec

// A window's peak is only printed if one of these thresholds is reached
const float ACCEL_PRINT_THRESHOLD_G = 100.0; // any single axis
const int FSR_HEAVY_ADC = 2800;              // "Heavy Squeeze"

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
  static int maxFsrAdc = 0;
  static unsigned int pollCount = 0;    // STATUS_REG polls since diagnostics
  static unsigned int newDataCount = 0; // new sensor samples processed
  static unsigned int overrunCount = 0; // polls where a sample was missed

  // Software data-ready: poll STATUS_REG every loop pass (INT1/INT2 are not
  //  wired) and only read the axes when it says a new sample is ready.
  //  Reading STATUS_REG alone does not clear ZYXDA, only reading the output
  //  registers does, so a sample arriving mid-read can't be lost unseen.
  uint8_t status = readRegister(STATUS_REG_ADDR);
  pollCount++;
  if (status & ZYXOR_BIT)
  {
    overrunCount++;  // a sample was overwritten before it was read
  }
  if (status & ZYXDA_BIT)
  {
    newDataCount++;
    int16_t x, y, z;
    readAxesBurst(x, y, z);
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

    int fsrAdc = analogRead(FSR_PIN); // 12-bit ADC (0 to 4095)
    if (fsrAdc > maxFsrAdc)
    {
      maxFsrAdc = fsrAdc;
    }
  }

  if (millis() - printTimer >= PRINT_INTERVAL_MS)
  {
    printTimer = millis();
    if (maxAxisG >= ACCEL_PRINT_THRESHOLD_G || maxFsrAdc >= FSR_HEAVY_ADC)
    {
      char row[160];
      int len = snprintf(row, sizeof(row),
        "%lu,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%c,%d,%.2f\n",
        (unsigned long)bootCount, printTimer, maxGx, maxGy, maxGz,
        maxCombined, peakAxisG, peakAxis, maxFsrAdc,
        (maxFsrAdc * 3.3) / 4095.0); // ESP32-S3 logic is 3.3V
      len = min(len, (int)sizeof(row) - 1); // snprintf returns untruncated length
      Serial.print(row);
      appendLog(row, len);
      lastEventMs = printTimer;
    }
    maxGx = maxGy = maxGz = 0;
    maxCombined = 0;
    maxAxisG = 0;
    peakAxisG = 0;
    peakAxis = 'x';
    maxFsrAdc = 0;
  }

  if (millis() - diagTimer >= DIAG_INTERVAL_MS)
  {
    unsigned long now = millis();
    unsigned long elapsedMs = now - diagTimer;
    diagTimer = now;
    // Sensor sample rate over the actual window (expect ~1000 Hz)
    Serial.print("[diag] samples: ");
    Serial.print(newDataCount);
    Serial.print(" rate: ");
    Serial.print(newDataCount * 1000UL / elapsedMs);
    Serial.print("Hz polls: ");
    Serial.print(pollCount);
    Serial.print(" overrun: ");
    Serial.println(overrunCount);
    pollCount = 0;
    newDataCount = 0;
    overrunCount = 0;
  }

  // Write buffered rows to flash once the event is over (no new rows for
  //  LOG_FLUSH_QUIET_MS), so the write never lands mid-impact
  if (logLen > 0 && millis() - lastEventMs >= LOG_FLUSH_QUIET_MS)
  {
    flushLog();
  }

  if (Serial.available())
  {
    handleCommand(Serial.read());
  }
}

// Reads a single accelerometer register.
uint8_t readRegister(uint8_t reg)
{
  Wire.beginTransmission(LIS331_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);    // repeated START, keep the bus
  Wire.requestFrom(LIS331_ADDR, (uint8_t)1);
  return Wire.read();
}

// Sets the BDU bit so the output registers are not updated until both
//  the low and high byte of the current sample have been read. This
//  prevents torn readings that mix bytes from two samples.
void enableBlockDataUpdate()
{
  uint8_t reg = readRegister(CTRL_REG4_ADDR);

  Wire.beginTransmission(LIS331_ADDR);
  Wire.write(CTRL_REG4_ADDR);
  Wire.write(reg | BDU_BIT);
  Wire.endTransmission();
}

// Reads all six output bytes (X_L, X_H, Y_L, Y_H, Z_L, Z_H) in a single
//  I2C transaction using register auto-increment, instead of the six
//  separate transactions done by xl.readAxes().
void readAxesBurst(int16_t &x, int16_t &y, int16_t &z)
{
  uint8_t data[6] = {0};
  Wire.beginTransmission(LIS331_ADDR);
  Wire.write(OUT_X_L_ADDR | AUTO_INC);
  Wire.endTransmission(false);    // repeated START, keep the bus
  Wire.requestFrom(LIS331_ADDR, (uint8_t)6);
  for (int i = 0; i < 6 && Wire.available(); i++)
  {
    data[i] = Wire.read();
  }
  // The data is 12-bit, left justified, so shift right by four after
  //  combining to get a signed value.
  x = (int16_t)(data[0] | data[1] << 8) >> 4;
  y = (int16_t)(data[2] | data[3] << 8) >> 4;
  z = (int16_t)(data[4] | data[5] << 8) >> 4;
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
void appendLog(const char *row, int len)
{
  if (!logReady || len <= 0)
  {
    return;
  }
  if (logLen + len > LOG_BUFFER_SIZE)
  {
    flushLog();
  }
  memcpy(logBuffer + logLen, row, len);
  logLen += len;
}

// Appends the RAM buffer to the newest segment. Starts a new segment when
//  the current one is full, and deletes the oldest segments until the
//  rows fit, so the log never stops and always keeps the newest data.
void flushLog()
{
  if (!logReady || logLen == 0)
  {
    return;
  }
  char path[32];
  segmentPath(path, sizeof(path), newestSegment);
  size_t segmentSize = 0;
  if (LittleFS.exists(path))
  {
    File f = LittleFS.open(path, FILE_READ);
    segmentSize = f.size();
    f.close();
  }
  if (segmentSize > 0 && segmentSize + logLen > LOG_SEGMENT_SIZE)
  {
    newestSegment++;
    segmentPath(path, sizeof(path), newestSegment);
  }
  while (LittleFS.totalBytes() - LittleFS.usedBytes() < logLen + LOG_FREE_MARGIN
         && oldestSegment < newestSegment)
  {
    char oldPath[32];
    segmentPath(oldPath, sizeof(oldPath), oldestSegment);
    LittleFS.remove(oldPath);
    oldestSegment++;
  }
  if (LittleFS.totalBytes() - LittleFS.usedBytes() < logLen + LOG_FREE_MARGIN)
  {
    logLen = 0; // only possible if something else fills the partition
    Serial.println("# LOG ERROR - no space left, rows dropped");
    return;
  }
  File f = LittleFS.open(path, FILE_APPEND);
  if (!f)
  {
    Serial.println("# LOG ERROR - could not open log file");
    return;
  }
  f.write((const uint8_t *)logBuffer, logLen);
  f.close();
  logLen = 0;
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
  Serial.println(" bytes used");
}

// Serial commands. Dumping blocks sampling while it runs; sampling
//  resyncs automatically afterwards.
void handleCommand(char c)
{
  if (c == 'd')
  {
    if (!logReady)
    {
      Serial.println("# LOG ERROR - LittleFS mount failed, no log");
      return;
    }
    flushLog();
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
    logLen = 0;
    for (uint32_t segment = oldestSegment; segment <= newestSegment; segment++)
    {
      char path[32];
      segmentPath(path, sizeof(path), segment);
      LittleFS.remove(path);
    }
    oldestSegment = newestSegment = 1;
    Serial.println("# log erased");
    Serial.print(LOG_HEADER); // header for the live rows
  }
  else if (c == 'i')
  {
    printLogInfo();
  }
}
