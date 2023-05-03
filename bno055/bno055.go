// Package bno055 implements a driver for the BNO055 Absolute Orientation Sensor.
//
// Datasheet: https://cdn-learn.adafruit.com/assets/assets/000/036/832/original/BST_BNO055_DS000_14.pdf
package bno055

import (
	"time"

	"tinygo.org/x/drivers"
)

type Device struct {
	bus     drivers.I2C
	Address uint16
	buf     [8]uint8
	mode    byte
}

func New(bus drivers.I2C) Device {
	// Create device with default configuration
	return Device{
		bus:     bus,
		Address: AddressLow,
	}
}

// Configure sets up the device
func (d *Device) Configure() {
	// Config
	d.bus.WriteRegister(uint8(d.Address), OPR_MODE, []byte{OPERATION_MODE_CONFIG})
	time.Sleep(30 * time.Millisecond)
	// Reset
	d.bus.WriteRegister(uint8(d.Address), SYS_TRIGGER, []byte{0x20})
	time.Sleep(60 * time.Millisecond)

	d.bus.WriteRegister(uint8(d.Address), PWR_MODE, []byte{POWER_MODE_NORMAL})
	time.Sleep(10 * time.Millisecond)

	d.bus.WriteRegister(uint8(d.Address), REG_PAGE_ID, []byte{0x00})
	time.Sleep(10 * time.Millisecond)

	d.bus.WriteRegister(uint8(d.Address), SYS_TRIGGER, []byte{0x00})
	time.Sleep(10 * time.Millisecond)
}

// Connected returns whether a BNO055 has been found.
// It does a device id request and checks the response.
func (d *Device) Connected() bool {
	data := d.buf[:1]
	d.bus.ReadRegister(uint8(d.Address), REG_DEV_ID, data)
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
func (d *Device) SetMode(mode byte) {
	d.mode = mode
	d.bus.WriteRegister(uint8(d.Address), OPR_MODE, []byte{mode})
	time.Sleep(30 * time.Millisecond)
}

// GetMode Retrieves the current mode
func (d *Device) GetMode() byte {
	data := d.buf[:1]
	d.bus.ReadRegister(uint8(d.Address), OPR_MODE, data)
	d.mode = data[0]
	return d.mode
}

// GetSystemStatus returns information about the system status, self test results, and system errors
func (d *Device) GetSystemStatus(systemStatus bool, selfTestResult bool, systemError bool) (system uint8, selfTest uint8, sysError uint8) {
	system = 0
	selfTest = 0
	sysError = 0
	d.bus.WriteRegister(uint8(d.Address), REG_PAGE_ID, []byte{0x00})
	if systemStatus {
		data := d.buf[:1]
		d.bus.ReadRegister(uint8(d.Address), SYS_STAT, data)
		system = data[0]
	}
	if selfTestResult {
		data := d.buf[:1]
		d.bus.ReadRegister(uint8(d.Address), SELFTEST_RESULT, data)
		selfTest = data[0]
	}
	if systemError {
		data := d.buf[:1]
		d.bus.ReadRegister(uint8(d.Address), SYS_ERR, data)
		sysError = data[0]
	}
	return
}

// GetCalibrationStatus returns whether the system, gyro, accelerometer, and the magnetometer are calibrated
func (d *Device) GetCalibrationStatus() (system bool, gyro bool, accel bool, mag bool) {
	data := d.buf[:1]
	d.bus.ReadRegister(uint8(d.Address), CALIB_STAT, data)
	system = (data[0]>>6)&0x03 > 0
	gyro = (data[0]>>4)&0x03 > 0
	accel = (data[0]>>2)&0x03 > 0
	mag = data[0]&0x03 > 0

	return
}

// ReadTemperature returns the current chip temperature
func (d *Device) ReadTemperature() int8 {
	data := d.buf[:1]
	d.bus.ReadRegister(uint8(d.Address), TEMPERATURE, data)
	return int8(data[0])
}

func (d *Device) readVector(register uint8) (x float64, y float64, z float64, err error) {
	data := d.buf[:6]
	d.bus.ReadRegister(uint8(d.Address), register, data)
	if err != nil {
		return 0, 0, 0, err
	}

	xInt := int16(data[0]) | int16(data[1])<<8
	yInt := int16(data[2]) | int16(data[3])<<8
	zInt := int16(data[4]) | int16(data[5])<<8

	switch register {
	case MAG_X_LSB:
		fallthrough
	case GYRO_X_LSB:
		fallthrough
	case EULER_H_LSB:
		x = float64(xInt) / 16.0
		y = float64(yInt) / 16.0
		z = float64(zInt) / 16.0
	case ACCEL_X_LSB:
		fallthrough
	case ACCEL_OFFSET_X_LSB:
		fallthrough
	case GRAVITY_X_LSB:
		x = float64(xInt) / 100.0
		y = float64(yInt) / 100.0
		z = float64(zInt) / 100.0
	}

	return
}

// ReadQuaternion reads the current absolute orientation as a quaternion and returns it
func (d *Device) ReadQuaternion() (w, x, y, z float64, err error) {
	const scale float64 = (1.0 / (1 << 14))
	data := d.buf[:8]
	d.bus.ReadRegister(uint8(d.Address), QUATERNION_W_LSB, data)
	if err != nil {
		return 0, 0, 0, 0, err
	}

	w = float64((uint16(data[1])<<8)|uint16(data[0])) * scale
	x = float64((uint16(data[3])<<8)|uint16(data[2])) * scale
	y = float64((uint16(data[5])<<8)|uint16(data[4])) * scale
	z = float64((uint16(data[7])<<8)|uint16(data[6])) * scale
	return
}

// ReadLinearAcceleration returns the current linear acceleration from the sensor
func (d *Device) ReadLinearAcceleration() (x, y, z float64, err error) {
	x, y, z, err = d.readVector(ACCEL_OFFSET_X_LSB)
	return
}

// ReadAngularAcceleration returns the current angular acceleration from the sensor
func (d *Device) ReadAngularAcceleration() (x, y, z float64, err error) {
	x, y, z, err = d.readVector(ACCEL_X_LSB)
	return
}

// ReadGravityVector returns the gravity vector from the sensor
func (d *Device) ReadGravityVector() (x, y, z float64, err error) {
	x, y, z, err = d.readVector(GRAVITY_X_LSB)
	return
}

// ReadRotation returns the current euler angle representation of absolute orientation
func (d *Device) ReadRotation() (x, y, z float64, err error) {
	x, y, z, err = d.readVector(EULER_H_LSB)
	return
}

// ReadRotation returns the current gyro sensor readings
func (d *Device) ReadGyro() (x, y, z float64, err error) {
	x, y, z, err = d.readVector(GYRO_X_LSB)
	return
}

// SetPowerLevel sets the power level of the sensor. Options are:
//
//	POWER_MODE_NORMAL -- All sensors for the operating mode are always on
//	POWER_MODE_LOWPOWER -- Only the accelerometer is active until motion is detected
//	POWER_MODE_SUSPEND -- All sensors are put to sleep. Manual power level change is required to exit
//
// See the datasheet for more details
func (d *Device) SetPowerLevel(power byte) {
	currentMode := d.mode
	d.SetMode(OPERATION_MODE_CONFIG)
	time.Sleep(25 * time.Millisecond)

	d.bus.WriteRegister(uint8(d.Address), PWR_MODE, []byte{power})
	time.Sleep(10 * time.Millisecond)

	d.SetMode(currentMode)
	time.Sleep(20 * time.Millisecond)
}

// SetAxisMapping sets the axis configuration for the sensor.
// See the datasheet for the options available.
func (d *Device) SetAxisMapping(axisMap byte, axisSign byte) {
	currentMode := d.mode
	d.SetMode(OPERATION_MODE_CONFIG)
	d.bus.WriteRegister(uint8(d.Address), AXIS_MAP_CONFIG, []byte{axisMap})
	time.Sleep(10 * time.Millisecond)
	d.bus.WriteRegister(uint8(d.Address), AXIS_MAP_SIGN, []byte{axisSign})
	time.Sleep(10 * time.Millisecond)
	d.SetMode(currentMode)
	time.Sleep(20 * time.Millisecond)
}

// SetUseExtCrystal tells the sensor whether or not to use an external 32KHz clock source
func (d *Device) SetUseExtCrystal(useExt bool) {
	currentMode := d.mode
	d.SetMode(OPERATION_MODE_CONFIG)
	d.bus.WriteRegister(uint8(d.Address), REG_PAGE_ID, []byte{0x00})
	if useExt {
		d.bus.WriteRegister(uint8(d.Address), SYS_TRIGGER, []byte{0x80})
	} else {
		d.bus.WriteRegister(uint8(d.Address), SYS_TRIGGER, []byte{0x00})
	}
	time.Sleep(10 * time.Millisecond)
	d.SetMode(currentMode)
	time.Sleep(20 * time.Millisecond)
}
