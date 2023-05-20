// Package bno055 implements a driver for the BNO055 Absolute Orientation Sensor.
//
// Datasheet: https://cdn-learn.adafruit.com/assets/assets/000/036/832/original/BST_BNO055_DS000_14.pdf
package bno055

import (
	"errors"
	"time"

	"tinygo.org/x/drivers"
)

type Device struct {
	bus             drivers.I2C
	Address         uint8
	temperatureUnit TemperatureUnit
	angularRateUnit AngularRateUnit
	accelUnit       AccelUnit
}

type OperationMode uint8

type AccelRange uint8
type AccelBandwidth uint8
type AccelMode uint8

type GyroRange uint8
type GyroBandwidth uint8
type GyroMode uint8

type MagRate uint8
type MagMode uint8
type MagPower uint8

type AccelUnit uint8
type AngularRateUnit uint8
type EulerAngleUnit uint8
type TemperatureUnit uint8

type FusionDataFormat uint8

type PowerMode uint8

type AxisMapping uint8

type AxisSign uint8

type Configuration struct {
	OperationMode OperationMode

	AccelRange     AccelRange
	AccelBandwidth AccelBandwidth
	AccelMode      AccelMode

	GyroRange     GyroRange
	GyroBandwidth GyroBandwidth
	GyroMode      GyroMode

	MagRate  MagRate
	MagMode  MagMode
	MagPower MagPower

	AccelUnit       AccelUnit
	AngularRateUnit AngularRateUnit
	EulerAngleUnit  EulerAngleUnit
	TemperatureUnit TemperatureUnit

	FusionDataFormat FusionDataFormat

	AxisMapping AxisMapping
	AxisSign    AxisSign

	PowerMode PowerMode

	UseExternalClock bool
}

func New(bus drivers.I2C) Device {
	// Create device with default configuration
	return Device{
		bus:     bus,
		Address: Address,
	}
}

func DefaultConfig() Configuration {
	return Configuration{
		OperationMode:    OPERATION_MODE_NDOF,
		FusionDataFormat: FUSION_DATA_FORMAT_ANDROID,
		AxisMapping:      AXIS_X_AS_X | AXIS_Y_AS_Y | AXIS_Z_AS_Z,
		PowerMode:        POWER_MODE_NORMAL,
		UseExternalClock: true,
	}
}

// Configure sets up the device
func (d Device) Configure(config Configuration) (err error) {
	err = d.doConfigure(config)
	if err != nil {
		return
	}

	// Reset interrupt status bits
	err = d.bus.WriteRegister(d.Address, SYS_TRIGGER, []byte{0x20})
	time.Sleep(60 * time.Millisecond)
	if err != nil {
		return err
	}

	// run self test
	err = d.bus.WriteRegister(d.Address, SYS_TRIGGER, []byte{0x00})
	time.Sleep(10 * time.Millisecond)
	if err != nil {
		return err
	}

	return nil
}

func (d Device) doConfigure(config Configuration) (err error) {
	// Wait for chip to boot
	r := []byte{0}
	timeout := 850
	for timeout = 850; timeout > 0; timeout -= 10 {
		if d.bus.Tx(uint16(d.Address), nil, r) == nil {
			break
		}
		time.Sleep(10 * time.Millisecond)
	}
	if timeout <= 0 {
		return errors.New("Timeout waiting for BNO055 to boot")
	}

	// Check chip id
	d.bus.ReadRegister(d.Address, REG_CHIP_ID, r)
	if r[0] != BNO055Id {
		time.Sleep(time.Second) // wait further for boot
		d.bus.ReadRegister(d.Address, REG_CHIP_ID, r)
		if r[0] != BNO055Id {
			return errors.New("Timeout waiting for BNO055 to identify")
		}
	}

	err = d.SetMode(OPERATION_MODE_CONFIG)
	if err != nil {
		return err
	}

	// reset
	d.bus.WriteRegister(d.Address, SYS_TRIGGER, []byte{0x20})
	time.Sleep(30 * time.Millisecond)

	r[0] = 0
	d.bus.ReadRegister(d.Address, REG_CHIP_ID, r)
	for err != nil && r[0] == BNO055Id {
		d.bus.ReadRegister(d.Address, REG_CHIP_ID, r)
		time.Sleep(10 * time.Millisecond)
	}
	time.Sleep(50 * time.Millisecond)

	err = d.bus.WriteRegister(d.Address, PWR_MODE, []byte{POWER_MODE_NORMAL})
	if err != nil {
		return err
	}
	time.Sleep(10 * time.Millisecond)

	// Configure Page 1 registers
	err = d.bus.WriteRegister(uint8(d.Address), REG_PAGE_ID, []byte{0x01})
	time.Sleep(10 * time.Millisecond)
	if err != nil {
		return err
	}

	// Set Accelerometer config
	err = d.bus.WriteRegister(uint8(d.Address), ACCEL_CONFIG, []byte{byte(config.AccelRange) | byte(config.AccelBandwidth) | byte(config.AccelMode)})
	time.Sleep(10 * time.Millisecond)
	if err != nil {
		return err
	}

	// Set Gyro config
	err = d.bus.WriteRegister(uint8(d.Address), GYRO_CONFIG_0, []byte{byte(config.GyroRange) | byte(config.GyroBandwidth)})
	time.Sleep(10 * time.Millisecond)
	if err != nil {
		return err
	}
	err = d.bus.WriteRegister(uint8(d.Address), GYRO_CONFIG_1, []byte{byte(config.GyroMode)})
	time.Sleep(10 * time.Millisecond)
	if err != nil {
		return err
	}

	// Set Magnetometer config
	err = d.bus.WriteRegister(uint8(d.Address), MAG_CONFIG, []byte{byte(config.MagRate) | byte(config.MagMode) | byte(config.MagPower)})
	time.Sleep(10 * time.Millisecond)
	if err != nil {
		return err
	}

	// Configure Page 0 registers
	err = d.bus.WriteRegister(uint8(d.Address), REG_PAGE_ID, []byte{0x00})
	time.Sleep(10 * time.Millisecond)
	if err != nil {
		return err
	}

	// Set power mode
	err = d.bus.WriteRegister(uint8(d.Address), PWR_MODE, []byte{byte(config.PowerMode)})
	time.Sleep(10 * time.Millisecond)
	if err != nil {
		return err
	}

	// Set Unit selection
	err = d.bus.WriteRegister(uint8(d.Address), UNIT_SEL, []byte{byte(config.FusionDataFormat) | byte(config.AccelUnit) | byte(config.AngularRateUnit) | byte(config.EulerAngleUnit) | byte(config.TemperatureUnit)})
	d.temperatureUnit = config.TemperatureUnit
	d.angularRateUnit = config.AngularRateUnit
	d.accelUnit = config.AccelUnit
	time.Sleep(10 * time.Millisecond)
	if err != nil {
		return err
	}

	// Set Axis Mapping
	err = d.bus.WriteRegister(uint8(d.Address), AXIS_MAP_CONFIG, []byte{byte(config.AxisMapping)})
	time.Sleep(10 * time.Millisecond)
	if err != nil {
		return err
	}

	// Set Axis Sign
	err = d.bus.WriteRegister(uint8(d.Address), AXIS_MAP_SIGN, []byte{byte(config.AxisSign)})
	time.Sleep(10 * time.Millisecond)
	if err != nil {
		return err
	}

	// Use External Clock
	if config.UseExternalClock {
		err = d.bus.WriteRegister(uint8(d.Address), SYS_TRIGGER, []byte{0x80})
	} else {
		err = d.bus.WriteRegister(uint8(d.Address), SYS_TRIGGER, []byte{0x00})
	}
	time.Sleep(10 * time.Millisecond)
	if err != nil {
		return err
	}

	// default mode -- fusion 9 deg of freedom
	err = d.SetMode(OPERATION_MODE_NDOF)
	if err != nil {
		return err
	}
	time.Sleep(20 * time.Millisecond)
	return nil
}

// Connected returns whether a BNO055 has been found.
// It does a device id request and checks the response.
func (d Device) Connected() bool {
	data := []byte{0}
	d.bus.ReadRegister(d.Address, REG_CHIP_ID, data)
	return data[0] == BNO055Id
}

// SetMode sets the mode of operation for the sensor.
// Where mode is one of:
//
//	OPERATION_MODE_CONFIG
//	OPERATION_MODE_ACCONLY
//	OPERATION_MODE_MAGONLY
//	OPERATION_MODE_GYRONLY
//	OPERATION_MODE_ACCMAG
//	OPERATION_MODE_ACCGYRO
//	OPERATION_MODE_MAGGYRO
//	OPERATION_MODE_AMG
//	OPERATION_MODE_IMUPLUS
//	OPERATION_MODE_COMPASS
//	OPERATION_MODE_M4G
//	OPERATION_MODE_NDOF_FMC_OFF
//	OPERATION_MODE_NDOF
//
// Refer to table 3.3 on page 30 of the datasheet
func (d Device) SetMode(mode byte) (err error) {
	return d.bus.WriteRegister(d.Address, OPR_MODE, []byte{mode})
}

// GetMode Retrieves the current mode
func (d Device) GetMode() byte {
	data := []byte{0}
	d.bus.ReadRegister(d.Address, OPR_MODE, data)
	return data[0]
}

// GetSystemStatus returns information about the system status, self test results, and system errors
func (d Device) GetStatus(systemStatus bool, selfTestResult bool, systemError bool) (system uint8, selfTest uint8, sysError uint8) {
	system = 0
	selfTest = 0
	sysError = 0
	data := []byte{0}
	d.bus.WriteRegister(d.Address, REG_PAGE_ID, []byte{0x00})
	if systemStatus {
		d.bus.ReadRegister(d.Address, SYS_STAT, data)
		system = data[0]
	}
	if selfTestResult {
		d.bus.ReadRegister(d.Address, SELFTEST_RESULT, data)
		selfTest = data[0]
	}
	if systemError {
		d.bus.ReadRegister(d.Address, SYS_ERR, data)
		sysError = data[0]
	}
	return
}

// GetCalibrationStatus returns whether the system, gyro, accelerometer, and the magnetometer are calibrated
func (d Device) GetCalibrationStatus() (system bool, gyro bool, accel bool, mag bool) {
	data := []byte{0}
	d.bus.ReadRegister(d.Address, CALIB_STAT, data)
	system = (data[0]>>6)&0x03 > 0
	gyro = (data[0]>>4)&0x03 > 0
	accel = (data[0]>>2)&0x03 > 0
	mag = data[0]&0x03 > 0

	return
}

// ReadTemperature returns the current chip temperature
func (d *Device) ReadTemperature() float64 {
	data := []byte{0}
	d.bus.ReadRegister(d.Address, TEMPERATURE, data)
	scale := TempScaleC
	if d.temperatureUnit == TEMPERATURE_UNIT_F {
		scale = TempScaleF
	}
	return float64(int8(data[0])) / scale
}

// ReadQuaternion reads the current absolute orientation as a quaternion and returns it
func (d *Device) ReadQuaternion() (w, x, y, z float64, err error) {
	data := make([]byte, 8)
	err = d.bus.ReadRegister(d.Address, QUATERNION_W_LSB, data)
	if err != nil {
		return 0, 0, 0, 0, err
	}

	w = float64((uint16(data[1])<<8)|uint16(data[0])) * QuaternionScale
	x = float64((uint16(data[3])<<8)|uint16(data[2])) * QuaternionScale
	y = float64((uint16(data[5])<<8)|uint16(data[4])) * QuaternionScale
	z = float64((uint16(data[7])<<8)|uint16(data[6])) * QuaternionScale
	return
}

func (d Device) readVector(register uint8) (x float64, y float64, z float64, err error) {
	data := make([]byte, 6)
	err = d.bus.ReadRegister(d.Address, register, data)
	if err != nil {
		return 0, 0, 0, err
	}

	xInt := int16(data[0]) | int16(data[1])<<8
	yInt := int16(data[2]) | int16(data[3])<<8
	zInt := int16(data[4]) | int16(data[5])<<8

	var scale float64 = 0

	switch register {
	case MAG_X_LSB:
		scale = MagScale
	case GYRO_X_LSB:
		scale = GyroScaleDegrees
		if d.angularRateUnit == ANGULAR_RATE_UNIT_RAD_SEC {
			scale = GyroScaleRadians
		}
		fallthrough
	case EULER_H_LSB:
		scale = EulerScaleDegrees
		if d.angularRateUnit == ANGULAR_RATE_UNIT_RAD_SEC {
			scale = EulerScaleRadians
		}
	case ACCEL_X_LSB:
		fallthrough
	case ACCEL_OFFSET_X_LSB:
		fallthrough
	case GRAVITY_X_LSB:
		scale = LinearAccelScaleMeterSec
		if d.accelUnit == ACCEL_UNIT_MICRO_G {
			scale = AccelScaleMicroGrav
		}
	}
	x = float64(xInt) / scale
	y = float64(yInt) / scale
	z = float64(zInt) / scale

	return
}

// ReadLinearAcceleration returns the current linear acceleration from the sensor
func (d Device) ReadLinearAcceleration() (x, y, z float64, err error) {
	x, y, z, err = d.readVector(ACCEL_OFFSET_X_LSB)
	return
}

// ReadAngularAcceleration returns the current angular acceleration from the sensor
func (d Device) ReadAngularAcceleration() (x, y, z float64, err error) {
	x, y, z, err = d.readVector(ACCEL_X_LSB)
	return
}

// ReadGravityVector returns the gravity vector from the sensor
func (d Device) ReadGravityVector() (x, y, z float64, err error) {
	x, y, z, err = d.readVector(GRAVITY_X_LSB)
	return
}

// ReadRotation returns the current euler angle representation of absolute orientation
func (d Device) ReadRotation() (x, y, z float64, err error) {
	x, y, z, err = d.readVector(EULER_H_LSB)
	return
}

// ReadRotation returns the current gyro sensor readings
func (d Device) ReadGyro() (x, y, z float64, err error) {
	x, y, z, err = d.readVector(GYRO_X_LSB)
	return
}

// ReadRotation returns the current gyro sensor readings
func (d Device) ReadMagneticField() (x, y, z float64, err error) {
	x, y, z, err = d.readVector(MAG_OFFSET_X_LSB)
	return
}
