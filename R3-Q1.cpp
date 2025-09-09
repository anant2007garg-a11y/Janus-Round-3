#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// GNSS module connected to pins 4 (RX), 3 (TX)
SoftwareSerial gnssSerial(4, 3);
TinyGPSPlus gps;

// Flight state machine
enum State { IDLE, ASCENT, APOGEE, DESCENT, PAYLOAD_DEPLOYED, LANDED };
State flightState = IDLE;

// Variables for altitude tracking
double currentAltitude = 0;
double previousAltitude = 0;
double maxAltitude = 0;
bool payloadDeployed = false;

void setup() {
  Serial.begin(115200);
  gnssSerial.begin(9600);

  Serial.println("Initializing GNSS...");

  // Configure Quectel L89HA to output only GNRMC + GNGGA
  // (These command strings come from Quectel documentation)
  gnssSerial.println("$PQGNME,GP,0,0,0,0,1,1,0,0,0,0*2A"); // Enable only GGA + RMC
}

void loop() {
  while (gnssSerial.available() > 0) {
    char c = gnssSerial.read();
    gps.encode(c);
  }

  if (gps.location.isUpdated() && gps.altitude.isUpdated()) {
    // Update altitude values
    previousAltitude = currentAltitude;
    currentAltitude = gps.altitude.meters();

    // Track maximum altitude
    if (currentAltitude > maxAltitude) {
      maxAltitude = currentAltitude;
    }

    // Run state machine
    updateState();

    // Print clean telemetry
    printTelemetry();
  }
}

void updateState() {
  switch (flightState) {
    case IDLE:
      if (currentAltitude > previousAltitude + 1.0) { // started rising
        flightState = ASCENT;
      }
      break;

    case ASCENT:
      if (currentAltitude < previousAltitude) { // stopped rising
        flightState = APOGEE;
      }
      break;

    case APOGEE:
      flightState = DESCENT;
      break;

    case DESCENT:
      if (!payloadDeployed && currentAltitude <= (0.75 * maxAltitude)) {
        flightState = PAYLOAD_DEPLOYED;
        payloadDeployed = true;
      } else if (currentAltitude <= gps.altitude.meters() + 2) {
        // crude landing detection (altitude stops changing significantly)
        flightState = LANDED;
      }
      break;

    case PAYLOAD_DEPLOYED:
      if (currentAltitude <= gps.altitude.meters() + 2) {
        flightState = LANDED;
      }
      break;

    case LANDED:
      // Do nothing
      break;
  }
}

void printTelemetry() {
  Serial.print("Time: ");
  if (gps.time.isValid()) {
    Serial.printf("%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
  } else {
    Serial.print("N/A");
  }

  Serial.print(" | Lat: ");
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
  } else {
    Serial.print("N/A");
  }

  Serial.print(" | Lon: ");
  if (gps.location.isValid()) {
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print("N/A");
  }

  Serial.print(" | Alt (MSL): ");
  if (gps.altitude.isValid()) {
    Serial.print(currentAltitude, 2);
    Serial.print(" m");
  } else {
    Serial.print("N/A");
  }

  Serial.print(" | State: ");
  switch (flightState) {
    case IDLE: Serial.print("IDLE"); break;
    case ASCENT: Serial.print("ASCENT"); break;
    case APOGEE: Serial.print("APOGEE"); break;
    case DESCENT: Serial.print("DESCENT"); break;
    case PAYLOAD_DEPLOYED: Serial.print("PAYLOAD DEPLOYED"); break;
    case LANDED: Serial.print("LANDED"); break;
  }
  Serial.println();
}