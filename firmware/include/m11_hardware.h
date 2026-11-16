/* Generated: tools/generate_m11_hardware.py; do not edit. */
#ifndef BIPE_M11_HARDWARE_H
#define BIPE_M11_HARDWARE_H
/* Teensy 4.0 pins from the routed carrier (electronics/m11/carrier). */
#define BIPE_M11_PIN_CAN_STBY 2
#define BIPE_M11_PIN_LED_DATA 3
#define BIPE_M11_PIN_ARM_N 4
#define BIPE_M11_PIN_STOP_N 5
#define BIPE_M11_PIN_PWR_GOOD_N 7
#define BIPE_M11_PIN_IMU_INT1 9
#define BIPE_M11_PIN_IMU_CS 10
#define BIPE_M11_PIN_SPI_MOSI 11
#define BIPE_M11_PIN_SPI_MISO 12
#define BIPE_M11_PIN_SPI_SCK 13
#define BIPE_M11_PIN_BUS_ADC 14
#define BIPE_M11_PIN_CAN_RX 30
#define BIPE_M11_PIN_CAN_TX 31
#define BIPE_M11_CAN_NOMINAL_BIT_S 1000000
#define BIPE_M11_CAN_DATA_BIT_S 5000000
#define BIPE_M11_CAN_SOURCE 0
/* LSM6DSO (ST DS12140 Rev 3). */
#define BIPE_M11_IMU_SPI_HZ 8000000
#define BIPE_M11_IMU_WHO_AM_I 0x6c
#define BIPE_M11_IMU_CTRL3_C 0x44
#define BIPE_M11_IMU_CTRL1_XL 0x88
#define BIPE_M11_IMU_CTRL2_G 0x84
#define BIPE_M11_IMU_INT1_CTRL 0x02
#define BIPE_M11_IMU_FIRST_OUTPUT 0x22
#define BIPE_M11_IMU_ACCEL_M_S2_PER_LSB 0.0011964113
#define BIPE_M11_IMU_GYRO_RAD_S_PER_LSB 0.0003054326190990077
#define BIPE_M11_IMU_GRAVITY_CHECK_RAD 0.5235987755982988
/* Body = R chip, row-major. */
#define BIPE_M11_IMU_TO_BODY {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0}
/* Bus monitor: divider into the Teensy ADC. */
#define BIPE_M11_BUS_ADC_SCALE 19.0
#define BIPE_M11_ADC_REFERENCE_V 3.3
#define BIPE_M11_ADC_MAX_COUNTS 4095.0
/* Drivers in JOINTS + WHEELS order: {id, wheel, sign, centre_rad}. */
#define BIPE_M11_AXES { \
    {1, 0, 1.0, -2.748893571891069}, \
    {2, 0, 1.0, -0.5497787143782138}, \
    {3, 0, 1.0, -2.748893571891069}, \
    {4, 0, 1.0, -0.5497787143782138}, \
    {5, 1, 1.0, 0.0}, \
    {6, 1, 1.0, 0.0}, \
}
/* Inputs: config/robot.json, config/controller_m10.json, electronics/m11/carrier/bipe-m11-carrier.kicad_pcb. tests/test_m11_firmware.py regenerates. */
#endif
