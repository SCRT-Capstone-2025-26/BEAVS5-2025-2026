#include <ISM6HG256XSensor.h>
#include <MS5611_SPI.h>
#include <SPI.h>

#include "defs.h"
#include "state.h"

enum BoardMode {
  BOOTING,
  UNARMED,
  ARMED,
  FLYING,
  DONE,
  FAILURE
};

BoardMode board_mode = BOOTING;
FlightState flight_state;
RestState rest_state;

MS5611_SPI baro(BAROMETER_CS, &SPI);
ISM6HG256XSensor imu(&SPI, IMU_CS);

// NOTE: Init values are temporary and will be determined by data later
void setup() {
  SPI.setRX(SPI_MISO);
  SPI.setTX(SPI_MOSI);
  SPI.setSCK(SPI_SCK);
  SPI.begin();

  baro.begin();
  baro.setOversampling(OSR_ULTRA_HIGH);

  imu.begin();

  imu.Enable_HG_X();
  imu.Enable_X();
  imu.Enable_G();

  imu.Set_X_OutputDataRate_With_Mode(ISM6HG256X_ACC_SENSITIVITY_FS_16G, ISM6HG256X_ACC_HIGH_ACCURACY_ODR_MODE);
  imu.Set_HG_X_OutputDataRate(ISM6HG256X_ACC_SENSITIVITY_FS_256G);
  imu.Set_G_OutputDataRate_With_Mode(ISM6HG256X_GYRO_SENSITIVITY_FS_4000DPS, ISM6HG256X_GYRO_HIGH_ACCURACY_ODR_MODE);

  board_mode = UNARMED;

  return;
failure:
  board_mode = FAILURE;
}

void loop() {
  if (board_mode == FAILURE) {
    do_failure();
    return;
  }

  sample_baro();
  sample_imu();

  update_mode();

  // TODO: Some amount of time that is correct and based on the current time to reduce drift
  delay(1);
}

// TODO: Unarmed to armed transition
void update_mode() {
  switch (board_mode) {
    case ARMED:
      if (rest_state.try_init_flying(flight_state)) {
        board_mode = FLYING;
      }
    case FLYING:
      if (flight_state.done()) {
        board_mode = DONE;
      }
    default:
      board_mode = FAILURE;
      break;
  }
}

void do_failure() {
  
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
  ISM6HG256X_Axes_t acc_hg_axis;
  ISM6HG256X_Axes_t gyro_axis;

  imu.Get_X_Axes(acc_axis);
  imu.Get_X_HG_Axes(acc_hg_axis);
  imu.Get_G_Axes(gyro_axis);

  if (board_mode == FLYING) {
    flight_state.push_imu(imu_axis, gyro_axis);
  } else if (board_mode == UNARMED || board_mode == ARMED) {
    rest_state.push_imu(imu_axis, gyro_axis);
  }
}

