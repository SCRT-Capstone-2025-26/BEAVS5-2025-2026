#include <ISM6HG256XSensor.h>
#include <SoftwareSPI.h>
#include <MS5611_SPI.h>
#include <SPI.h>

#include "pins.h"
#include "state.h"
#include "logging.h"
#include "led.h"
#include "util.h"

BoardMode board_mode = BOOTING;
FlightState flight_state;
RestState rest_state;

// The pins aren't correctly assigned for hardware SPI on the board
// I assume it is a mistake (?) so we have to use bit banging
SoftwareSPI softSPI(SPI_SCK, SPI_MISO, SPI_MOSI);

// Pressure drop when hitting around mach numbers
// Data loss for possibly 1-2 seconds (has to be filtered)
MS5611_SPI baro(BAROMETER_CS, &softSPI);
ISM6HG256XSensor imu(&softSPI, IMU_CS);

void init_pins() {
  // Disable servo power on startup due to inrush
  // Removing this could damage the board
  pinMode(SERVO_POWER_ENABLE, OUTPUT);
  digitalWrite(SERVO_POWER_ENABLE, LOW);

  // Put the level shifter into the regular mode
  pinMode(LEVELSHIFT_DIR, OUTPUT);
  digitalWrite(LEVELSHIFT_DIR, LOW);

  // No floating pins for levelshifter
  pinMode(SERVO_1, OUTPUT);  digitalWrite(SERVO_1, LOW);
  pinMode(SERVO_2, OUTPUT);  digitalWrite(SERVO_2, LOW);
  pinMode(SERVO_3, OUTPUT);  digitalWrite(SERVO_3, LOW);
  pinMode(SERVO_4, OUTPUT);  digitalWrite(SERVO_4, LOW);
  pinMode(SERVO_5, OUTPUT);  digitalWrite(SERVO_5, LOW);
  pinMode(SERVO_6, OUTPUT);  digitalWrite(SERVO_6, LOW);
  pinMode(LED_DATA, OUTPUT); digitalWrite(LED_DATA, LOW);

  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(SCK, OUTPUT);

  pinMode(BAROMETER_CS, OUTPUT);
  pinMode(IMU_CS, OUTPUT);
  pinMode(RADIO_CS, OUTPUT);

  pinMode(BATTERY_SENSE, INPUT);

  digitalWrite(BAROMETER_CS, HIGH);
  digitalWrite(IMU_CS, HIGH);
  digitalWrite(RADIO_CS, HIGH);
}

void push_failure(LEDs failure_led) {
  leds[failure_led] = LED_NEGATIVE;
  led_show();
}

// NOTE: Init values are temporary and will be determined by data later
// TODO: Check the boards current state instead of assuming it is one the ground and booting
void setup() {
  // Check where servo is
  // Confirm on the ground
  // Check Beavs open/closed
  // Use accelerometer to establish coordinates
  // No partial deployments? (real time trajectory)
  // Reset clock when on land

  // Account for reset during flight
  // Account for gyro saturation (~11 rpms/~4000dps)
  // Check SD card write rate

  // Calibrate sensors after boot
  // Expect to have setup run multiple times (Watchdoy timers)
  // Arm switch deboucing if no hardware debouncing
  // Don't set board_mode
  // Graph out fault chart

  // Initialize the pins
  // This initializes the servo power pins which if improperly initialized can cause
  //  problems with the capacitors when charging
  init_pins();
  log_message("Pins inited");

  // This inits the LEDs it sets them all to powered off
  led_init();
  log_message("LEDs inited");

  // The radio is not currently used (or installed) so we just set the led to mark that (neutral is blue which is visible)
  leds[LED_RADIO] = LED_NEUTRAL;
  // Same with the magnetometer
  leds[LED_MAGN] = LED_NEUTRAL;
  led_show();

  // Initialize the LED the rp2040 has two SPIs and we init the first one to be able to communicate to the sensors
  softSPI.begin();
  log_message("SPI inited");

  // Initialize the barometer we has an MS5607, but the interface should be the same as the MS5611 which
  // is the library we are using
  bool baro_init = baro.begin();
  // Since we are using softwareSPI we have to set the rate in this library
  // This is what the brinup code sets it to so ...
  baro.setSPIspeed(10000000);
  // This sampling will change in the future
  baro.setOversampling(OSR_ULTRA_HIGH);

  if (baro_init) { log_message("Barometer inited"); }
  leds[LED_BARO] = baro_init ? LED_POSITIVE : LED_NEGATIVE;
  led_show();

  // The imu returns custom status values instead of booleans
  // So the returns are checked (as shown in their example code you can also or all these values and then check ISM6HG256_OK at the end)
  bool imu_init = imu.begin() == ISM6HG256X_OK;

  // Enable all the sensors on the imu. It has two accelerometers one for low g and one for high g.
  // Both can be active at once
  imu_init &= imu.Enable_X() == ISM6HG256X_OK;
  imu_init &= imu.Enable_HG_X() == ISM6HG256X_OK;
  imu_init &= imu.Enable_G() == ISM6HG256X_OK;

  // TODO: Set up the accelerometer mode currently ISM6HG256X_ACC_HIGH_ACCURACY_ODR_MODE
  //  just immediatly causes the init to do nothing and return error

  if (baro_init) { log_message("IMU inited"); }
  leds[LED_IMU] = imu_init ? LED_POSITIVE : LED_NEGATIVE;
  led_show();

  // Wait for the other core to finish booting
  // This returns when the other core has booted with whether it has created log files
  bool sd_failure = wait_log_boot();
  // If there is an SD failure mark that (it is not critical though).
  // Since the function returned the other core has booted and we can continue
  log_message("Log core booted")
  if (!sd_failure) { log_message("SD inited"); }
  leds[LED_SD] = sd_failure ? LED_NEGATIVE : LED_POSITIVE;
  led_show();

  // An sd_failure isn't critical so it is not included in this if
  if (baro_init && imu_init) {
    // The board is now ready to be armed
    board_mode = UNARMED;
    leds[LED_STATUS] = LED_POSITIVE;
  } else {
    // The board has failed to init
    board_mode = FAILURE;
    leds[LED_STATUS] = LED_POSITIVE;
  }

  led_show();
}

// This is called every loop iteration and is responsible for managing the state transitions
void update_mode() {
  switch (board_mode) {
    // TODO: Unarmed to armed transition
    case ARMED:
      // If the rest_state can init the flying state then it means that it has detected high acceleration
      //  and we are in flight
      if (rest_state.try_init_flying(flight_state)) {
        board_mode = FLYING;
      }
    case FLYING:
      // If the flight_state decides that we are done (practically our angle is too high or we have timed out)
      if (flight_state.done()) {
        board_mode = DONE;
      }
    default:
      board_mode = FAILURE;
      break;
  }
}

void update_servo() {
  if (board_mode == FLYING) {
    // TODO: Set servo to something from flight_state
  } else {
    // TODO: Set servo to flush
  }
}

// TODO: Check self heating mentioned for similar product in MS5xxx library docs
// TODO: Add error handling
void sample_baro() {
  baro.read();
  double temp = baro.getPressure();
  double pressure = baro.getTemperature();

  if (board_mode == FLYING) {
    flight_state.push_baro(temp, pressure);
  } else if (board_mode == UNARMED || board_mode == ARMED) {
    rest_state.push_baro(temp, pressure);
  }
}

// TODO: Add error handling
void sample_imu() {
  ISM6HG256X_Axes_t acc_axis;
  ISM6HG256X_Axes_t gyro_axis;

  imu.Get_X_Axes(&acc_axis);
  imu.Get_G_Axes(&gyro_axis);

  if (board_mode == FLYING) {
    flight_state.push_imu(acc_axis, gyro_axis);
  } else if (board_mode == UNARMED || board_mode == ARMED) {
    rest_state.push_imu(acc_axis, gyro_axis);
  }
}

// This handles what the board should do when it has reached a critical failure
// TODO: One sensor fail may not be critical
void do_failure() {
  delay(100);
}

void loop() {
  // If we have reached critical failure then we return early
  if (board_mode == FAILURE) {
    do_failure();
    return;
  }

  // Sample the sensors (this updates the relevant state object)
  sample_baro();
  sample_imu();

  update_mode();

  // Update the servo based on the state object
  update_servo();

  // TODO: Some amount of time that is correct and based on the current time to reduce drift
  delay(1);
}
