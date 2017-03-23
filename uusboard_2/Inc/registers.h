/* Page UART register definition*/
#define UART_START_BYTE									0XAA
#define UART_WRITE											0X00
#define UART_READ												0X01
/* Page id register definition*/
#define PAGE_ID_ADDR										0X07

/* PAGE0 REGISTER DEFINITION START*/
#define CHIP_ID_ADDR         				    0X00
#define ACCEL_REV_ID_ADDR								0X01
#define MAG_REV_ID_ADDR      					  0X02
#define GYRO_REV_ID_ADDR     					  0X03
#define SW_REV_ID_LSB_ADDR							0X04
#define SW_REV_ID_MSB_ADDR							0X05
#define BL_REV_ID_ADDR									0X06

/* Accel data register*/
#define ACCEL_DATA_X_LSB_ADDR						0X08
#define ACCEL_DATA_X_MSB_ADDR						0X09
#define ACCEL_DATA_Y_LSB_ADDR						0X0A
#define ACCEL_DATA_Y_MSB_ADDR						0X0B
#define ACCEL_DATA_Z_LSB_ADDR						0X0C
#define ACCEL_DATA_Z_MSB_ADDR						0X0D

/*Mag data register*/
#define MAG_DATA_X_LSB_ADDR							0X0E
#define MAG_DATA_X_MSB_ADDR							0X0F
#define MAG_DATA_Y_LSB_ADDR							0X10
#define MAG_DATA_Y_MSB_ADDR							0X11
#define MAG_DATA_Z_LSB_ADDR							0X12
#define MAG_DATA_Z_MSB_ADDR							0X13

/*Gyro data registers*/
#define GYRO_DATA_X_LSB_ADDR						0X14
#define GYRO_DATA_X_MSB_ADDR						0X15
#define GYRO_DATA_Y_LSB_ADDR						0X16
#define GYRO_DATA_Y_MSB_ADDR						0X17
#define GYRO_DATA_Z_LSB_ADDR						0X18
#define GYRO_DATA_Z_MSB_ADDR						0X19

/*Euler data registers*/
#define EULER_H_LSB_ADDR								0X1A
#define EULER_H_MSB_ADDR								0X1B

#define EULER_R_LSB_ADDR								0X1C
#define EULER_R_MSB_ADDR								0X1D

#define EULER_P_LSB_ADDR								0X1E
#define EULER_P_MSB_ADDR								0X1F

/*Quaternion data registers*/
#define QUATERNION_DATA_W_LSB_ADDR			0X20
#define QUATERNION_DATA_W_MSB_ADDR			0X21
#define QUATERNION_DATA_X_LSB_ADDR			0X22
#define QUATERNION_DATA_X_MSB_ADDR			0X23
#define QUATERNION_DATA_Y_LSB_ADDR			0X24
#define QUATERNION_DATA_Y_MSB_ADDR			0X25
#define QUATERNION_DATA_Z_LSB_ADDR			0X26
#define QUATERNION_DATA_Z_MSB_ADDR			0X27

/* Linear acceleration data registers*/
#define LINEAR_ACCEL_DATA_X_LSB_ADDR		0X28
#define LINEAR_ACCEL_DATA_X_MSB_ADDR		0X29
#define LINEAR_ACCEL_DATA_Y_LSB_ADDR		0X2A
#define LINEAR_ACCEL_DATA_Y_MSB_ADDR		0X2B
#define LINEAR_ACCEL_DATA_Z_LSB_ADDR		0X2C
#define LINEAR_ACCEL_DATA_Z_MSB_ADDR		0X2D

/*Gravity data registers*/
#define GRAVITY_DATA_X_LSB_ADDR					0X2E
#define GRAVITY_DATA_X_MSB_ADDR					0X2F
#define GRAVITY_DATA_Y_LSB_ADDR					0X30
#define GRAVITY_DATA_Y_MSB_ADDR					0X31
#define GRAVITY_DATA_Z_LSB_ADDR					0X32
#define GRAVITY_DATA_Z_MSB_ADDR					0X33

/* Temperature data register*/
#define TEMP_ADDR												0X34

/* Status registers*/
#define CALIB_STAT_ADDR									0X35
#define SELFTEST_RESULT_ADDR						0X36
#define INTR_STAT_ADDR									0X37
#define SYS_CLK_STAT_ADDR								0X38
#define SYS_STAT_ADDR										0X39
#define SYS_ERR_ADDR										0X3A

/* Unit selection register*/
#define UNIT_SEL_ADDR										0X3B
#define DATA_SELECT_ADDR								0X3C

/* Mode registers*/
#define OPR_MODE_ADDR										0X3D
#define PWR_MODE_ADDR										0X3E

#define SYS_TRIGGER_ADDR								0X3F
#define TEMP_SOURCE_ADDR								0X40
/* Axis remap registers*/
#define AXIS_MAP_CONFIG_ADDR						0X41
#define AXIS_MAP_SIGN_ADDR							0X42

/* SIC registers*/
#define SIC_MATRIX_0_LSB_ADDR						0X43
#define SIC_MATRIX_0_MSB_ADDR						0X44
#define SIC_MATRIX_1_LSB_ADDR						0X45
#define SIC_MATRIX_1_MSB_ADDR						0X46
#define SIC_MATRIX_2_LSB_ADDR						0X47
#define SIC_MATRIX_2_MSB_ADDR						0X48
#define SIC_MATRIX_3_LSB_ADDR						0X49
#define SIC_MATRIX_3_MSB_ADDR						0X4A
#define SIC_MATRIX_4_LSB_ADDR						0X4B
#define SIC_MATRIX_4_MSB_ADDR						0X4C
#define SIC_MATRIX_5_LSB_ADDR						0X4D
#define SIC_MATRIX_5_MSB_ADDR						0X4E
#define SIC_MATRIX_6_LSB_ADDR						0X4F
#define SIC_MATRIX_6_MSB_ADDR						0X50
#define SIC_MATRIX_7_LSB_ADDR						0X51
#define SIC_MATRIX_7_MSB_ADDR						0X52
#define SIC_MATRIX_8_LSB_ADDR						0X53
#define SIC_MATRIX_8_MSB_ADDR						0X54

/* Accelerometer Offset registers*/
#define ACCEL_OFFSET_X_LSB_ADDR					0X55
#define ACCEL_OFFSET_X_MSB_ADDR					0X56
#define ACCEL_OFFSET_Y_LSB_ADDR					0X57
#define ACCEL_OFFSET_Y_MSB_ADDR					0X58
#define ACCEL_OFFSET_Z_LSB_ADDR					0X59
#define ACCEL_OFFSET_Z_MSB_ADDR					0X5A

/* Magnetometer Offset registers*/
#define MAG_OFFSET_X_LSB_ADDR						0X5B
#define MAG_OFFSET_X_MSB_ADDR						0X5C
#define MAG_OFFSET_Y_LSB_ADDR						0X5D
#define MAG_OFFSET_Y_MSB_ADDR						0X5E
#define MAG_OFFSET_Z_LSB_ADDR						0X5F
#define MAG_OFFSET_Z_MSB_ADDR						0X60

/* Gyroscope Offset registers*/
#define GYRO_OFFSET_X_LSB_ADDR					0X61
#define GYRO_OFFSET_X_MSB_ADDR					0X62
#define GYRO_OFFSET_Y_LSB_ADDR					0X63
#define GYRO_OFFSET_Y_MSB_ADDR					0X64
#define GYRO_OFFSET_Z_LSB_ADDR					0X65
#define GYRO_OFFSET_Z_MSB_ADDR					0X66

/* Radius registers*/
#define	ACCEL_RADIUS_LSB_ADDR						0X67
#define	ACCEL_RADIUS_MSB_ADDR						0X68
#define	MAG_RADIUS_LSB_ADDR							0X69
#define	MAG_RADIUS_MSB_ADDR							0X6A
/* PAGE0 REGISTERS DEFINITION END*/

/* PAGE1 REGISTERS DEFINITION START*/
/* Configuration registers*/
#define ACCEL_CONFIG_ADDR								0X08
#define MAG_CONFIG_ADDR									0X09
#define GYRO_CONFIG_ADDR								0X0A
#define GYRO_MODE_CONFIG_ADDR						0X0B
#define ACCEL_SLEEP_CONFIG_ADDR					0X0C
#define GYRO_SLEEP_CONFIG_ADDR					0X0D
#define MAG_SLEEP_CONFIG_ADDR						0X0E

/* Interrupt registers*/
#define INT_MASK_ADDR										0X0F
#define INT_ADDR												0X10
#define ACCEL_ANY_MOTION_THRES_ADDR			0X11
#define ACCEL_INTR_SETTINGS_ADDR				0X12
#define ACCEL_HIGH_G_DURN_ADDR					0X13
#define ACCEL_HIGH_G_THRES_ADDR					0X14
#define ACCEL_NO_MOTION_THRES_ADDR			0X15
#define ACCEL_NO_MOTION_SET_ADDR				0X16
#define GYRO_INTR_SETING_ADDR						0X17
#define GYRO_HIGHRATE_X_SET_ADDR				0X18
#define GYRO_DURN_X_ADDR								0X19
#define GYRO_HIGHRATE_Y_SET_ADDR				0X1A
#define GYRO_DURN_Y_ADDR								0X1B
#define GYRO_HIGHRATE_Z_SET_ADDR				0X1C
#define GYRO_DURN_Z_ADDR								0X1D
#define GYRO_ANY_MOTION_THRES_ADDR			0X1E
#define GYRO_ANY_MOTION_SET_ADDR				0X1F
