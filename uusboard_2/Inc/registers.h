/* Page UART register definition*/
#define UART_START_BYTE									0xAA
#define UART_WRITE											0x00
#define UART_READ												0x01
/* Page id register definition*/
#define PAGE_ID_ADDR										0X07

/* PAGE0 REGISTER DEFINITION START*/
#define CHIP_ID_ADDR         				    0x00
#define ACCEL_REV_ID_ADDR								0x01
#define MAG_REV_ID_ADDR      					  0x02
#define GYRO_REV_ID_ADDR     					  0x03
#define SW_REV_ID_LSB_ADDR							0x04
#define SW_REV_ID_MSB_ADDR							0x05
#define BL_REV_ID_ADDR									0x06

/* Accel data register*/
#define ACCEL_DATA_X_LSB_ADDR						0x08
#define ACCEL_DATA_X_MSB_ADDR						0x09
#define ACCEL_DATA_Y_LSB_ADDR						0x0A
#define ACCEL_DATA_Y_MSB_ADDR						0x0B
#define ACCEL_DATA_Z_LSB_ADDR						0x0C
#define ACCEL_DATA_Z_MSB_ADDR						0x0D

/*Mag data register*/
#define MAG_DATA_X_LSB_ADDR							0x0E
#define MAG_DATA_X_MSB_ADDR							0x0F
#define MAG_DATA_Y_LSB_ADDR							0x10
#define MAG_DATA_Y_MSB_ADDR							0x11
#define MAG_DATA_Z_LSB_ADDR							0x12
#define MAG_DATA_Z_MSB_ADDR							0x13

/*Gyro data registers*/
#define GYRO_DATA_X_LSB_ADDR						0x14
#define GYRO_DATA_X_MSB_ADDR						0x15
#define GYRO_DATA_Y_LSB_ADDR						0x16
#define GYRO_DATA_Y_MSB_ADDR						0x17
#define GYRO_DATA_Z_LSB_ADDR						0x18
#define GYRO_DATA_Z_MSB_ADDR						0x19

/*Euler data registers*/
#define EULER_H_LSB_ADDR								0x1A
#define EULER_H_MSB_ADDR								0x1B

#define EULER_R_LSB_ADDR								0x1C
#define EULER_R_MSB_ADDR								0x1D

#define EULER_P_LSB_ADDR								0x1E
#define EULER_P_MSB_ADDR								0x1F

/*Quaternion data registers*/
#define QUATERNION_DATA_W_LSB_ADDR			0x20
#define QUATERNION_DATA_W_MSB_ADDR			0x21
#define QUATERNION_DATA_X_LSB_ADDR			0x22
#define QUATERNION_DATA_X_MSB_ADDR			0x23
#define QUATERNION_DATA_Y_LSB_ADDR			0x24
#define QUATERNION_DATA_Y_MSB_ADDR			0x25
#define QUATERNION_DATA_Z_LSB_ADDR			0x26
#define QUATERNION_DATA_Z_MSB_ADDR			0x27

/* Linear acceleration data registers*/
#define LINEAR_ACCEL_DATA_X_LSB_ADDR		0x28
#define LINEAR_ACCEL_DATA_X_MSB_ADDR		0x29
#define LINEAR_ACCEL_DATA_Y_LSB_ADDR		0x2A
#define LINEAR_ACCEL_DATA_Y_MSB_ADDR		0x2B
#define LINEAR_ACCEL_DATA_Z_LSB_ADDR		0x2C
#define LINEAR_ACCEL_DATA_Z_MSB_ADDR		0x2D

/*Gravity data registers*/
#define GRAVITY_DATA_X_LSB_ADDR					0x2E
#define GRAVITY_DATA_X_MSB_ADDR					0x2F
#define GRAVITY_DATA_Y_LSB_ADDR					0x30
#define GRAVITY_DATA_Y_MSB_ADDR					0x31
#define GRAVITY_DATA_Z_LSB_ADDR					0x32
#define GRAVITY_DATA_Z_MSB_ADDR					0x33

/* Temperature data register*/
#define TEMP_ADDR												0x34

/* Status registers*/
#define CALIB_STAT_ADDR									0x35
#define SELFTEST_RESULT_ADDR						0x36
#define INTR_STAT_ADDR									0x37
#define SYS_CLK_STAT_ADDR								0x38
#define SYS_STAT_ADDR										0x39
#define SYS_ERR_ADDR										0x3A

/* Unit selection register*/
#define UNIT_SEL_ADDR										0x3B
#define DATA_SELECT_ADDR								0x3C

/* Mode registers*/
#define OPR_MODE_ADDR										0x3D
#define PWR_MODE_ADDR										0x3E

#define SYS_TRIGGER_ADDR								0x3F
#define TEMP_SOURCE_ADDR								0x40
/* Axis remap registers*/
#define AXIS_MAP_CONFIG_ADDR						0x41
#define AXIS_MAP_SIGN_ADDR							0x42

/* SIC registers*/
#define SIC_MATRIX_0_LSB_ADDR						0x43
#define SIC_MATRIX_0_MSB_ADDR						0x44
#define SIC_MATRIX_1_LSB_ADDR						0x45
#define SIC_MATRIX_1_MSB_ADDR						0x46
#define SIC_MATRIX_2_LSB_ADDR						0x47
#define SIC_MATRIX_2_MSB_ADDR						0x48
#define SIC_MATRIX_3_LSB_ADDR						0x49
#define SIC_MATRIX_3_MSB_ADDR						0x4A
#define SIC_MATRIX_4_LSB_ADDR						0x4B
#define SIC_MATRIX_4_MSB_ADDR						0x4C
#define SIC_MATRIX_5_LSB_ADDR						0x4D
#define SIC_MATRIX_5_MSB_ADDR						0x4E
#define SIC_MATRIX_6_LSB_ADDR						0x4F
#define SIC_MATRIX_6_MSB_ADDR						0x50
#define SIC_MATRIX_7_LSB_ADDR						0x51
#define SIC_MATRIX_7_MSB_ADDR						0x52
#define SIC_MATRIX_8_LSB_ADDR						0x53
#define SIC_MATRIX_8_MSB_ADDR						0x54

/* Accelerometer Offset registers*/
#define ACCEL_OFFSET_X_LSB_ADDR					0x55
#define ACCEL_OFFSET_X_MSB_ADDR					0x56
#define ACCEL_OFFSET_Y_LSB_ADDR					0x57
#define ACCEL_OFFSET_Y_MSB_ADDR					0x58
#define ACCEL_OFFSET_Z_LSB_ADDR					0x59
#define ACCEL_OFFSET_Z_MSB_ADDR					0x5A

/* Magnetometer Offset registers*/
#define MAG_OFFSET_X_LSB_ADDR						0x5B
#define MAG_OFFSET_X_MSB_ADDR						0x5C
#define MAG_OFFSET_Y_LSB_ADDR						0x5D
#define MAG_OFFSET_Y_MSB_ADDR						0x5E
#define MAG_OFFSET_Z_LSB_ADDR						0x5F
#define MAG_OFFSET_Z_MSB_ADDR						0x60

/* Gyroscope Offset registers*/
#define GYRO_OFFSET_X_LSB_ADDR					0x61
#define GYRO_OFFSET_X_MSB_ADDR					0x62
#define GYRO_OFFSET_Y_LSB_ADDR					0x63
#define GYRO_OFFSET_Y_MSB_ADDR					0x64
#define GYRO_OFFSET_Z_LSB_ADDR					0x65
#define GYRO_OFFSET_Z_MSB_ADDR					0x66

/* Radius registers*/
#define	ACCEL_RADIUS_LSB_ADDR						0x67
#define	ACCEL_RADIUS_MSB_ADDR						0x68
#define	MAG_RADIUS_LSB_ADDR							0x69
#define	MAG_RADIUS_MSB_ADDR							0x6A
/* PAGE0 REGISTERS DEFINITION END*/

/* PAGE1 REGISTERS DEFINITION START*/
/* Configuration registers*/
#define ACCEL_CONFIG_ADDR								0x08
#define MAG_CONFIG_ADDR									0x09
#define GYRO_CONFIG_ADDR								0x0A
#define GYRO_MODE_CONFIG_ADDR						0x0B
#define ACCEL_SLEEP_CONFIG_ADDR					0x0C
#define GYRO_SLEEP_CONFIG_ADDR					0x0D
#define MAG_SLEEP_CONFIG_ADDR						0x0E

/* Interrupt registers*/
#define INT_MASK_ADDR										0x0F
#define INT_ADDR												0x10
#define ACCEL_ANY_MOTION_THRES_ADDR			0x11
#define ACCEL_INTR_SETTINGS_ADDR				0x12
#define ACCEL_HIGH_G_DURN_ADDR					0x13
#define ACCEL_HIGH_G_THRES_ADDR					0x14
#define ACCEL_NO_MOTION_THRES_ADDR			0x15
#define ACCEL_NO_MOTION_SET_ADDR				0x16
#define GYRO_INTR_SETING_ADDR						0x17
#define GYRO_HIGHRATE_X_SET_ADDR				0x18
#define GYRO_DURN_X_ADDR								0x19
#define GYRO_HIGHRATE_Y_SET_ADDR				0x1A
#define GYRO_DURN_Y_ADDR								0x1B
#define GYRO_HIGHRATE_Z_SET_ADDR				0x1C
#define GYRO_DURN_Z_ADDR								0x1D
#define GYRO_ANY_MOTION_THRES_ADDR			0x1E
#define GYRO_ANY_MOTION_SET_ADDR				0x1F
