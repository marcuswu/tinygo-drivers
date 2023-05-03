package bno055

const AddressLow = 0x28 // TODO: figure out what these should be
const AddressHigh = 0x28

const BNO055Id = 0xA0

const (
	OPERATION_MODE_CONFIG       = 0x00
	OPERATION_MODE_ACCONLY      = 0x01
	OPERATION_MODE_MAGONLY      = 0x02
	OPERATION_MODE_GYRONLY      = 0x03
	OPERATION_MODE_ACCMAG       = 0x04
	OPERATION_MODE_ACCGYRO      = 0x05
	OPERATION_MODE_MAGGYRO      = 0x06
	OPERATION_MODE_AMG          = 0x07
	OPERATION_MODE_IMUPLUS      = 0x08
	OPERATION_MODE_COMPASS      = 0x09
	OPERATION_MODE_M4G          = 0x0A
	OPERATION_MODE_NDOF_FMC_OFF = 0x0B
	OPERATION_MODE_NDOF         = 0x0C

	POWER_MODE_NORMAL   = 0x00
	POWER_MODE_LOWPOWER = 0x01
	POWER_MODE_SUSPEND  = 0x02

	REMAP_CONFIG_P0 = 0x21
	REMAP_CONFIG_P1 = 0x24
	REMAP_CONFIG_P2 = 0x24
	REMAP_CONFIG_P3 = 0x21
	REMAP_CONFIG_P4 = 0x24
	REMAP_CONFIG_P5 = 0x21
	REMAP_CONFIG_P6 = 0x21
	REMAP_CONFIG_P7 = 0x24

	REMAP_SIGN_P0 = 0x04
	REMAP_SIGN_P1 = 0x00
	REMAP_SIGN_P2 = 0x06
	REMAP_SIGN_P3 = 0x02
	REMAP_SIGN_P4 = 0x03
	REMAP_SIGN_P5 = 0x01
	REMAP_SIGN_P6 = 0x07
	REMAP_SIGN_P7 = 0x05

	REG_DEV_ID     = 0x00
	REG_ACCEL_REV  = 0x01
	REG_MAG_REV    = 0x02
	REG_GYRO_REV   = 0x03
	REG_SW_REV_LSB = 0x04
	REG_SW_REV_MSB = 0x05
	REG_BL_REV     = 0x06
	REG_PAGE_ID    = 0x07

	ACCEL_X_LSB = 0x08
	ACCEL_X_MSB = 0x09
	ACCEL_Y_LSB = 0x0A
	ACCEL_Y_MSB = 0x0B
	ACCEL_Z_LSB = 0x0C
	ACCEL_Z_MSB = 0x0D

	MAG_X_LSB = 0x0E
	MAG_X_MSB = 0x0F
	MAG_Y_LSB = 0x10
	MAG_Y_MSB = 0x11
	MAG_Z_LSB = 0x12
	MAG_Z_MSB = 0x13

	GYRO_X_LSB = 0x14
	GYRO_X_MSB = 0x15
	GYRO_Y_LSB = 0x16
	GYRO_Y_MSB = 0x17
	GYRO_Z_LSB = 0x18
	GYRO_Z_MSB = 0x19

	EULER_H_LSB = 0x1A
	EULER_H_MSB = 0x1B
	EULER_R_LSB = 0x1C
	EULER_R_MSB = 0x1D
	EULER_P_LSB = 0x1E
	EULER_P_MSB = 0x1F

	QUATERNION_W_LSB = 0x20
	QUATERNION_W_MSB = 0x21
	QUATERNION_X_LSB = 0x22
	QUATERNION_X_MSB = 0x23
	QUATERNION_Y_LSB = 0x24
	QUATERNION_Y_MSB = 0x25
	QUATERNION_Z_LSB = 0x26
	QUATERNION_Z_MSB = 0x27

	LINEAR_ACCEL_X_LSB = 0x28
	LINEAR_ACCEL_X_MSB = 0x29
	LINEAR_ACCEL_Y_LSB = 0x2A
	LINEAR_ACCEL_Y_MSB = 0x2B
	LINEAR_ACCEL_Z_LSB = 0x2C
	LINEAR_ACCEL_Z_MSB = 0x2D

	GRAVITY_X_LSB = 0x2E
	GRAVITY_X_MSB = 0x2F
	GRAVITY_Y_LSB = 0x30
	GRAVITY_Y_MSB = 0x31
	GRAVITY_Z_LSB = 0x32
	GRAVITY_Z_MSB = 0x33

	TEMPERATURE = 0x34

	CALIB_STAT      = 0x35
	SELFTEST_RESULT = 0x36
	INTR_STAT       = 0x37

	SYS_CLK  = 0x38
	SYS_STAT = 0x39
	SYS_ERR  = 0x3A

	UNIT_SEL = 0x3B

	OPR_MODE = 0x3D
	PWR_MODE = 0x3E

	SYS_TRIGGER = 0x3F
	TEMP_SOURCE = 0x40

	AXIS_MAP_CONFIG = 0x41
	AXIS_MAP_SIGN   = 0x42

	SIC_MATRIX_0_LSB = 0x43
	SIC_MATRIX_0_MSB = 0x44
	SIC_MATRIX_1_LSB = 0x45
	SIC_MATRIX_1_MSB = 0x46
	SIC_MATRIX_2_LSB = 0x47
	SIC_MATRIX_2_MSB = 0x48
	SIC_MATRIX_3_LSB = 0x49
	SIC_MATRIX_3_MSB = 0x4A
	SIC_MATRIX_4_LSB = 0x4B
	SIC_MATRIX_4_MSB = 0x4C
	SIC_MATRIX_5_LSB = 0x4D
	SIC_MATRIX_5_MSB = 0x4E
	SIC_MATRIX_6_LSB = 0x4F
	SIC_MATRIX_6_MSB = 0x50
	SIC_MATRIX_7_LSB = 0x51
	SIC_MATRIX_7_MSB = 0x52
	SIC_MATRIX_8_LSB = 0x53
	SIC_MATRIX_8_MSB = 0x54

	ACCEL_OFFSET_X_LSB = 0x55
	ACCEL_OFFSET_X_MSB = 0x56
	ACCEL_OFFSET_Y_LSB = 0x57
	ACCEL_OFFSET_Y_MSB = 0x58
	ACCEL_OFFSET_Z_LSB = 0x59
	ACCEL_OFFSET_Z_MSB = 0x5A

	MAG_OFFSET_X_LSB = 0x5B
	MAG_OFFSET_X_MSB = 0x5C
	MAG_OFFSET_Y_LSB = 0x5D
	MAG_OFFSET_Y_MSB = 0x5E
	MAG_OFFSET_Z_LSB = 0x5F
	MAG_OFFSET_Z_MSB = 0x60

	GYRO_OFFSET_X_LSB = 0x61
	GYRO_OFFSET_X_MSB = 0x62
	GYRO_OFFSET_Y_LSB = 0x63
	GYRO_OFFSET_Y_MSB = 0x64
	GYRO_OFFSET_Z_LSB = 0x65
	GYRO_OFFSET_Z_MSB = 0x66

	ACCEL_RADIUS_LSB = 0x67
	ACCEL_RADIUS_MSB = 0x68
	MAG_RADIUS_LSB   = 0x69
	MAG_RADIUS_MSB   = 0x6A
)
