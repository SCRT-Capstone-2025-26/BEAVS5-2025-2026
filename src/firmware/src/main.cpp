#include <ISM6HG256XSensor.h>
#include <SoftwareSPI.h>
#include <MS5611_SPI.h>
#include <SPI.h>
#include <RP2040_PWM.h>

#include "pins.h"
#include "state.h"
#include "logging.h"
#include "led.h"
#include "util.h"

// TODO: Look into clock overflow
// TODO: Look into gyro saturation
// TODO: Look into pressure drop when hitting around mach numbers
// TODO: Create a function to switch mode that logs the switches

BoardMode board_mode = BOOTING;
FlightState flight_state = FlightState();
RestState rest_state = RestState();

// The pins aren't correctly assigned for hardware SPI on the board
// I assume it is a mistake (?) so we have to use bit banging
SoftwareSPI softSPI(SPI_SCK, SPI_MISO, SPI_MOSI);

MS5611_SPI baro(BAROMETER_CS, &softSPI);
ISM6HG256XSensor imu(&softSPI, IMU_CS);

// The servo has an operating frequency of 50-300Hz
RP2040_PWM servo(SERVO_1, 300, 0);

// TODO: Handle wrap around
Millis next_sample;
Millis sample_rate = 100;

void init_pins() {
  // Disable servo power on startup due to inrush
  // Removing this could damage the board
  pinMode(SERVO_POWER_ENABLE, OUTPUT);
  digitalWrite(SERVO_POWER_ENABLE, LOW);

  // Put the level shifter into the regular mode
  pinMode(LEVELSHIFT_DIR, OUTPUT);
  digitalWrite(LEVELSHIFT_DIR, LOW);

  // We only use servo 1 so that is the only one inited to be a pwm pin
  // The others can just be 0
  servo.setPWM();
  // No floating pins for levelshifter
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

// The servo cannot be enabled before the capacitors charge
void init_servo() {
  // We could do 2000 - millis(), but it is safer not to
  // Once we add booting in flight this may change
  delay(2000);

  digitalWrite(SERVO_POWER_ENABLE, HIGH);
}

void push_mode(BoardMode mode) {
  log_message(ModeChange(board_mode, mode));
  board_mode = mode;
}

void push_failure(LEDs failure_led) {
  leds[failure_led] = LED_NEGATIVE;
  leds[LED_STATUS] = LED_NEGATIVE;
  led_show();

  push_mode(FAILURE);
}

// NOTE: Init values are temporary and will be determined by data later
// TODO: Check the boards current state instead of assuming it is one the ground and booting (in case of power loss or watchdog)
void setup() {
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
  log_message("Log core booted");
  if (!sd_failure) { log_message("SD inited"); }
  leds[LED_SD] = sd_failure ? LED_NEGATIVE : LED_POSITIVE;
  led_show();

  // init_servo();
  // TODO: Prove/test servo works here

  // An sd_failure isn't critical so it is not included in this if
  if (baro_init && imu_init) {
    // The board is now ready to be armed
    push_mode(UNARMED);
    leds[LED_STATUS] = LED_POSITIVE;
  } else {
    // The board has failed to init
    push_mode(FAILURE);
    leds[LED_STATUS] = LED_POSITIVE;
  }

  led_show();

  next_sample = millis();
}

// This is called every loop iteration and is responsible for managing the state transitions
void update_mode() {
  switch (board_mode) {
    // TODO: Unarmed to armed transition
    case UNARMED:
      // NOTE: This is for testing
      if (millis() > 8000) { push_mode(ARMED); }
      break;
    case ARMED:
      // If the rest_state can init the flying state then it means that it has detected high acceleration
      //  and we are in flight
      if (rest_state.try_init_flying(flight_state)) {
        push_mode(FLYING);
      }
      break;
    case FLYING:
      // If the flight_state decides that we are done (practically our angle is too high or we have timed out)
      if (flight_state.done()) {
        push_mode(DONE);
      }
      break;
    default:
      push_failure(LED_STATUS);
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
void sample_baro(Millis sample_rate) {
  baro.read();
  double temp = baro.getPressure();
  double pressure = baro.getTemperature();

  if (board_mode == FLYING) {
    flight_state.push_baro(temp, pressure, sample_rate);
  } else if (board_mode == UNARMED || board_mode == ARMED) {
    rest_state.push_baro(temp, pressure, sample_rate);
  }
}

// TODO: Add error handling
void sample_imu(Millis sample_rate) {
  ISM6HG256X_Axes_t acc_axis;
  ISM6HG256X_Axes_t gyro_axis;

  imu.Get_X_Axes(&acc_axis);
  imu.Get_G_Axes(&gyro_axis);

  if (board_mode == FLYING) {
    flight_state.push_imu(acc_axis, gyro_axis, sample_rate);
  } else if (board_mode == UNARMED || board_mode == ARMED) {
    rest_state.push_imu(acc_axis, gyro_axis, sample_rate);
  }
}

// This handles what the board should do when it has reached a critical failure
// TODO: One sensor fail may not be critical
void do_failure() {
  delay(1000);
}

void loop() {
  // If we have reached critical failure then we return early
  if (board_mode == FAILURE) {
    do_failure();
    return;
  }

  // Sample the sensors (this updates the relevant state object)
  sample_baro(sample_rate);
  sample_imu(sample_rate);

  update_mode();

  // Update the servo based on the state object
  update_servo();

  next_sample += sample_rate;
  Millis curr = millis();
  // Since millis is unsigned we have to check for overflow
  if (curr <= next_sample) { delay(next_sample - curr); }
}
