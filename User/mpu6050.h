#ifndef __MPU6050_H
#define __MPU6050_H
#include "i2c.h"
#include "struct_typedef.h"
#include <math.h>

// I2C timeout
#define MPU6050_I2C_TIMEOUT 1000

//#define MPU_ACCEL_OFFS_REG		0X06	//accel_offs寄存器，可读取版本号
//#define MPU_PROD_ID_REG			0X0C	//prod id¼Ä´æÆ÷,ÔÚ¼Ä´æÆ÷ÊÖ²áÎ´Ìáµ½
#define MPU_SELF_TESTX_REG		0X0D	//×Ô¼ì¼Ä´æÆ÷X
#define MPU_SELF_TESTY_REG		0X0E	//×Ô¼ì¼Ä´æÆ÷Y
#define MPU_SELF_TESTZ_REG		0X0F	//×Ô¼ì¼Ä´æÆ÷Z
#define MPU_SELF_TESTA_REG		0X10	//×Ô¼ì¼Ä´æÆ÷A
#define MPU_SAMPLE_RATE_REG		0X19	//²ÉÑùÆµÂÊ·ÖÆµÆ÷
#define MPU_CFG_REG				0X1A	//ÅäÖÃ¼Ä´æÆ÷
#define MPU_GYRO_CFG_REG			0X1B	//ÍÓÂÝÒÇÅäÖÃ¼Ä´æÆ÷
#define MPU_ACCEL_CFG_REG			0X1C	//¼ÓËÙ¶È¼ÆÅäÖÃ¼Ä´æÆ÷
#define MPU_MOTION_DET_REG		0X1F	//ÔË¶¯¼ì²â·§ÖµÉèÖÃ¼Ä´æÆ÷
#define MPU_FIFO_EN_REG			0X23	//FIFOÊ¹ÄÜ¼Ä´æÆ÷
#define MPU_I2CMST_CTRL_REG		0X24	//IICÖ÷»ú¿ØÖÆ¼Ä´æÆ÷
#define MPU_I2CSLV0_ADDR_REG		0X25	//IIC´Ó»ú0Æ÷¼þµØÖ·¼Ä´æÆ÷
#define MPU_I2CSLV0_REG			0X26	//IIC´Ó»ú0Êý¾ÝµØÖ·¼Ä´æÆ÷
#define MPU_I2CSLV0_CTRL_REG		0X27	//IIC´Ó»ú0¿ØÖÆ¼Ä´æÆ÷
#define MPU_I2CSLV1_ADDR_REG		0X28	//IIC´Ó»ú1Æ÷¼þµØÖ·¼Ä´æÆ÷
#define MPU_I2CSLV1_REG			0X29	//IIC´Ó»ú1Êý¾ÝµØÖ·¼Ä´æÆ÷
#define MPU_I2CSLV1_CTRL_REG		0X2A	//IIC´Ó»ú1¿ØÖÆ¼Ä´æÆ÷
#define MPU_I2CSLV2_ADDR_REG		0X2B	//IIC´Ó»ú2Æ÷¼þµØÖ·¼Ä´æÆ÷
#define MPU_I2CSLV2_REG			0X2C	//IIC´Ó»ú2Êý¾ÝµØÖ·¼Ä´æÆ÷
#define MPU_I2CSLV2_CTRL_REG		0X2D	//IIC´Ó»ú2¿ØÖÆ¼Ä´æÆ÷
#define MPU_I2CSLV3_ADDR_REG		0X2E	//IIC´Ó»ú3Æ÷¼þµØÖ·¼Ä´æÆ÷
#define MPU_I2CSLV3_REG			0X2F	//IIC´Ó»ú3Êý¾ÝµØÖ·¼Ä´æÆ÷
#define MPU_I2CSLV3_CTRL_REG		0X30	//IIC´Ó»ú3¿ØÖÆ¼Ä´æÆ÷
#define MPU_I2CSLV4_ADDR_REG		0X31	//IIC´Ó»ú4Æ÷¼þµØÖ·¼Ä´æÆ÷
#define MPU_I2CSLV4_REG			0X32	//IIC´Ó»ú4Êý¾ÝµØÖ·¼Ä´æÆ÷
#define MPU_I2CSLV4_DO_REG		0X33	//IIC´Ó»ú4Ð´Êý¾Ý¼Ä´æÆ÷
#define MPU_I2CSLV4_CTRL_REG		0X34	//IIC´Ó»ú4¿ØÖÆ¼Ä´æÆ÷
#define MPU_I2CSLV4_DI_REG		0X35	//IIC´Ó»ú4¶ÁÊý¾Ý¼Ä´æÆ÷

#define MPU_I2CMST_STA_REG		0X36	//IICÖ÷»ú×´Ì¬¼Ä´æÆ÷
#define MPU_INTBP_CFG_REG			0X37	//ÖÐ¶Ï/ÅÔÂ·ÉèÖÃ¼Ä´æÆ÷
#define MPU_INT_EN_REG			0X38	//ÖÐ¶ÏÊ¹ÄÜ¼Ä´æÆ÷
#define MPU_INT_STA_REG			0X3A	//ÖÐ¶Ï×´Ì¬¼Ä´æÆ÷

#define MPU_ACCEL_XOUTH_REG		0X3B	//¼ÓËÙ¶ÈÖµ,XÖá¸ß8Î»¼Ä´æÆ÷
#define MPU_ACCEL_XOUTL_REG		0X3C	//¼ÓËÙ¶ÈÖµ,XÖáµÍ8Î»¼Ä´æÆ÷
#define MPU_ACCEL_YOUTH_REG		0X3D	//¼ÓËÙ¶ÈÖµ,YÖá¸ß8Î»¼Ä´æÆ÷
#define MPU_ACCEL_YOUTL_REG		0X3E	//¼ÓËÙ¶ÈÖµ,YÖáµÍ8Î»¼Ä´æÆ÷
#define MPU_ACCEL_ZOUTH_REG		0X3F	//¼ÓËÙ¶ÈÖµ,ZÖá¸ß8Î»¼Ä´æÆ÷
#define MPU_ACCEL_ZOUTL_REG		0X40	//¼ÓËÙ¶ÈÖµ,ZÖáµÍ8Î»¼Ä´æÆ÷

#define MPU_TEMP_OUTH_REG			0X41	//ÎÂ¶ÈÖµ¸ß°ËÎ»¼Ä´æÆ÷
#define MPU_TEMP_OUTL_REG			0X42	//ÎÂ¶ÈÖµµÍ8Î»¼Ä´æÆ÷

#define MPU_GYRO_XOUTH_REG		0X43	//ÍÓÂÝÒÇÖµ,XÖá¸ß8Î»¼Ä´æÆ÷
#define MPU_GYRO_XOUTL_REG		0X44	//ÍÓÂÝÒÇÖµ,XÖáµÍ8Î»¼Ä´æÆ÷
#define MPU_GYRO_YOUTH_REG		0X45	//ÍÓÂÝÒÇÖµ,YÖá¸ß8Î»¼Ä´æÆ÷
#define MPU_GYRO_YOUTL_REG		0X46	//ÍÓÂÝÒÇÖµ,YÖáµÍ8Î»¼Ä´æÆ÷
#define MPU_GYRO_ZOUTH_REG		0X47	//ÍÓÂÝÒÇÖµ,ZÖá¸ß8Î»¼Ä´æÆ÷
#define MPU_GYRO_ZOUTL_REG		0X48	//ÍÓÂÝÒÇÖµ,ZÖáµÍ8Î»¼Ä´æÆ÷

#define MPU_I2CSLV0_DO_REG		0X63	//IIC´Ó»ú0Êý¾Ý¼Ä´æÆ÷
#define MPU_I2CSLV1_DO_REG		0X64	//IIC´Ó»ú1Êý¾Ý¼Ä´æÆ÷
#define MPU_I2CSLV2_DO_REG		0X65	//IIC´Ó»ú2Êý¾Ý¼Ä´æÆ÷
#define MPU_I2CSLV3_DO_REG		0X66	//IIC´Ó»ú3Êý¾Ý¼Ä´æÆ÷

#define MPU_I2CMST_DELAY_REG		0X67	//IICÖ÷»úÑÓÊ±¹ÜÀí¼Ä´æÆ÷
#define MPU_SIGPATH_RST_REG		0X68	//ÐÅºÅÍ¨µÀ¸´Î»¼Ä´æÆ÷
#define MPU_MDETECT_CTRL_REG		0X69	//ÔË¶¯¼ì²â¿ØÖÆ¼Ä´æÆ÷
#define MPU_USER_CTRL_REG			0X6A	//ÓÃ»§¿ØÖÆ¼Ä´æÆ÷
#define MPU_PWR_MGMT1_REG			0X6B	//µçÔ´¹ÜÀí¼Ä´æÆ÷1
#define MPU_PWR_MGMT2_REG			0X6C	//µçÔ´¹ÜÀí¼Ä´æÆ÷2 
#define MPU_FIFO_CNTH_REG			0X72	//FIFO¼ÆÊý¼Ä´æÆ÷¸ß°ËÎ»
#define MPU_FIFO_CNTL_REG			0X73	//FIFO¼ÆÊý¼Ä´æÆ÷µÍ°ËÎ»
#define MPU_FIFO_RW_REG			0X74	//FIFO¶ÁÐ´¼Ä´æÆ÷
#define MPU_DEVICE_ID_REG			0X75	//Æ÷¼þID¼Ä´æÆ÷

//Èç¹ûAD0½Å(9½Å)½ÓµØ,IICµØÖ·Îª0X68(²»°üº¬×îµÍÎ»).
//Èç¹û½ÓV3.3,ÔòIICµØÖ·Îª0X69(²»°üº¬×îµÍÎ»).
#define MPU_ADDR					0X68

// MPU6050结构体，用于存储姿态角
typedef struct
{
    float pitch;  // 俯仰角
    float roll;   // 横滚角
    float yaw;    // 航向角
} MPU6050_Attitude;

// 与 RM INS 接口保持一致的角度索引定义（数组顺序：yaw, pitch, roll）
#define INS_YAW_ADDRESS_OFFSET   0
#define INS_PITCH_ADDRESS_OFFSET 1
#define INS_ROLL_ADDRESS_OFFSET  2

uint8_t MPU_Init(void); 								//³õÊ¼»¯MPU6050
void MPU_Calibrate_Gyro(uint16_t samples); // 陀螺仪零偏校准（静止时调用）
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);//IICÁ¬ÐøÐ´
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf); //IICÁ¬Ðø¶Á
uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data);				//IICÐ´Ò»¸ö×Ö½Ú
uint8_t MPU_Read_Byte(uint8_t reg);						//IIC¶ÁÒ»¸ö×Ö½Ú

uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr);
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr);
uint8_t MPU_Set_LPF(uint16_t lpf);
uint8_t MPU_Set_Rate(uint16_t rate);
uint8_t MPU_Set_Fifo(uint8_t sens);


short MPU_Get_Temperature(void);
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az);

// 新增姿态角读取函数
extern uint8_t MPU_Get_Attitude(MPU6050_Attitude *attitude);
extern void MPU_Get_Pitch_Roll_Yaw(float *pitch, float *roll, float *yaw);
extern void MPU6050_Get_Yaw(float *yaw);

// 获取/更新姿态角（欧拉角，单位：rad），用于 chassis_task
const fp32 *get_INS_angle_point(void);
void MPU6050_Update_INS_Angle(void);

#endif
