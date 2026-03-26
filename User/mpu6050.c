#include "stm32f1xx_hal.h"
#include "mpu6050.h"

// 外部I2C句柄
extern I2C_HandleTypeDef hi2c2;

// MPU6050姿态角计算所需的变量
// 预留用于陀螺仪校准
static float gyro_offset_x = 0; // 陀螺仪X轴零偏(°/s)
static float gyro_offset_y = 0; // 陀螺仪Y轴零偏(°/s)
static float gyro_offset_z = 0; // 陀螺仪Z轴零偏(°/s)
static float angle_x = 0;
static float angle_y = 0;
static float angle_z = 0;
static uint32_t last_time = 0;

// INS 角度缓存（单位：rad）
static fp32 mpu6050_ins_angle[3] = {0.0f, 0.0f, 0.0f};


uint8_t MPU_Init(void)
{
    uint8_t res;

    HAL_Delay(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);
    HAL_Delay(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);
    MPU_Set_Gyro_Fsr(3);					
    MPU_Set_Accel_Fsr(0);				
    MPU_Set_Rate(100);						
    MPU_Write_Byte(MPU_INT_EN_REG,0X00);	
    MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	
    MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	
    MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	
    res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
    if(res==MPU_ADDR)//Æ÷¼þIDÕýÈ·
    {
        MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	
        MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	
        MPU_Set_Rate(100);						
        
        // 初始化时间戳
        last_time = HAL_GetTick();
    } else return 1;
    return 0;
}

// 根据配置的量程返回陀螺仪LSB/(°/s)比例
static float MPU_Get_Gyro_Scale(void)
{
    uint8_t cfg = MPU_Read_Byte(MPU_GYRO_CFG_REG) >> 3; // 量程选择 bits[4:3]
    switch(cfg)
    {
        case 0: return 131.0f;   // ±250 dps
        case 1: return 65.5f;    // ±500 dps
        case 2: return 32.8f;    // ±1000 dps
        case 3: return 16.4f;    // ±2000 dps
        default: return 16.4f;   // 回退
    }
}

// 陀螺仪零偏校准：在静止状态下调用，采集多次求平均
void MPU_Calibrate_Gyro(uint16_t samples)
{
    if(samples < 500) samples = 500; // 最低样本数
    uint32_t start = HAL_GetTick();
    float scale = MPU_Get_Gyro_Scale();
    long sum_x = 0, sum_y = 0, sum_z = 0;
    for(uint16_t i=0;i<samples;i++)
    {
        short gx, gy, gz;
        if(MPU_Get_Gyroscope(&gx,&gy,&gz)!=0) continue;
        sum_x += gx;
        sum_y += gy;
        sum_z += gz;
        HAL_Delay(2); // 给I2C一点间隔，避免阻塞
    }
    gyro_offset_x = (float)sum_x / samples / scale;
    gyro_offset_y = (float)sum_y / samples / scale;
    gyro_offset_z = (float)sum_z / samples / scale;
    last_time = HAL_GetTick(); // 校准完重新开始时间基准
    (void)start; // 避免编译器警告
}

uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr) // 
{
    return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);
}

uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);
}

uint8_t MPU_Set_LPF(uint16_t lpf) // 设置低通滤波频率
{
    uint8_t data=0;
    if(lpf>=188)data=1;
    else if(lpf>=98)data=2;
    else if(lpf>=42)data=3;
    else if(lpf>=20)data=4;
    else if(lpf>=10)data=5;
    else data=6;
    return MPU_Write_Byte(MPU_CFG_REG,data);
}

uint8_t MPU_Set_Rate(uint16_t rate)
{
    uint8_t data;
    if(rate>1000)rate=1000;
    if(rate<4)rate=4;
    data=1000/rate-1;
    data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	
    return MPU_Set_LPF(rate/2);
}

short MPU_Get_Temperature(void)
{
    uint8_t buf[2];
    short raw;
    float temp;
    MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf);
    raw=((uint16_t)buf[0]<<8)|buf[1];
    temp=36.53+((double)raw)/340;
    return temp;
}

uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz) // 获取陀螺仪原始数据
{
    uint8_t buf[6],res;
    res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
    if(res==0)
    {
        *gx=((uint16_t)buf[0]<<8)|buf[1];
        *gy=((uint16_t)buf[2]<<8)|buf[3];
        *gz=((uint16_t)buf[4]<<8)|buf[5];
    }
    return res;
}

uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az) // 获取加速度原始数据
{
    uint8_t buf[6],res;
    res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
    if(res==0)
    {
        *ax=((uint16_t)buf[0]<<8)|buf[1];
        *ay=((uint16_t)buf[2]<<8)|buf[3];
        *az=((uint16_t)buf[4]<<8)|buf[5];
    }
    return res;;
}

uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write(&hi2c2, (addr<<1), reg, I2C_MEMADD_SIZE_8BIT, buf, len, MPU6050_I2C_TIMEOUT);
    if(status == HAL_OK)
        return 0;
    else
        return 1;
}

uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(&hi2c2, (addr<<1), reg, I2C_MEMADD_SIZE_8BIT, buf, len, MPU6050_I2C_TIMEOUT);
    if(status == HAL_OK)
        return 0;
    else
        return 1;
}

uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write(&hi2c2, (MPU_ADDR<<1), reg, I2C_MEMADD_SIZE_8BIT, &data, 1, MPU6050_I2C_TIMEOUT);
    if(status == HAL_OK)
        return 0;
    else
        return 1;
}

uint8_t MPU_Read_Byte(uint8_t reg)
{
    uint8_t res;
    HAL_I2C_Mem_Read(&hi2c2, (MPU_ADDR<<1), reg, I2C_MEMADD_SIZE_8BIT, &res, 1, MPU6050_I2C_TIMEOUT);
    return res;
}

/**
 * @brief  获取MPU6050姿态角（pitch, roll, yaw）
 * @param  attitude: 指向MPU6050_Attitude结构体的指针
 * @retval 0:成功, 其他:失败
 */
uint8_t MPU_Get_Attitude(MPU6050_Attitude *attitude)
{
    short ax, ay, az;
    short gx, gy, gz;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float dt;
    uint32_t current_time;
    
    // 读取加速度计和陀螺仪数据
    if(MPU_Get_Accelerometer(&ax, &ay, &az) != 0)
        return 1;
    if(MPU_Get_Gyroscope(&gx, &gy, &gz) != 0)
        return 1;
    
    // 计算时间差（秒）
    current_time = HAL_GetTick();
    dt = (current_time - last_time) / 1000.0f;
    last_time = current_time;
    
    // 转换加速度计数据（±2g量程，灵敏度为16384 LSB/g）
    accel_x = ax / 16384.0f;
    accel_y = ay / 16384.0f;
    accel_z = az / 16384.0f;
    
    // 转换陀螺仪数据 (动态按当前量程比例)
    float g_scale = MPU_Get_Gyro_Scale();
    gyro_x = gx / g_scale - gyro_offset_x;
    gyro_y = gy / g_scale - gyro_offset_y;
    gyro_z = gz / g_scale - gyro_offset_z;
    
    // 使用加速度计计算俯仰角和横滚角
    float accel_pitch = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * 180.0f / 3.14159265f;
    float accel_roll = atan2(-accel_x, accel_z) * 180.0f / 3.14159265f;
    
    // 陀螺仪积分
    angle_x += gyro_x * dt;
    angle_y += gyro_y * dt;
    angle_z += gyro_z * dt;
    
    // 互补滤波器（加速度计和陀螺仪融合）
    // 0.98的权重给陀螺仪，0.02的权重给加速度计
    attitude->pitch = 0.98f * angle_x + 0.02f * accel_pitch;
    attitude->roll = 0.98f * angle_y + 0.02f * accel_roll;
    attitude->yaw = angle_z; // 航向角：仅陀螺仪积分+零偏校准，仍为相对航向
    
    // 更新角度
    angle_x = attitude->pitch;
    angle_y = attitude->roll;
    
    return 0;
}

/**
 * @brief  直接获取pitch, roll, yaw角度值
 * @param  pitch: 俯仰角指针
 * @param  roll: 横滚角指针
 * @param  yaw: 航向角指针
 * @retval None
 */
void MPU_Get_Pitch_Roll_Yaw(float *pitch, float *roll, float *yaw)
{
    MPU6050_Attitude attitude;
    
    if(MPU_Get_Attitude(&attitude) == 0)
    {
        *pitch = attitude.pitch;
        *roll = attitude.roll;
        *yaw = attitude.yaw;
    }
}
// 仅仅在代码层修改，增加静止检测：计算过去 N 帧加速度模的均值与方差，
// 满足方差 < 阈值且模接近 1g 时，执行 gyro_offset_z = (1-α)*gyro_offset_z + α*(gz/g_scale)（α 很小，比如 0.001）。
//温度补偿实测记录温度 vs 零偏，建立线性关系 gyro_offset_z = a*T + b，在运行时修正。

/**
 * @brief  直接获取yaw角度值
 * @param  yaw: 航向角指针
 * @retval None
 */
void MPU6050_Get_Yaw(float *yaw)
{
    MPU6050_Attitude attitude;
    
    if(MPU_Get_Attitude(&attitude) == 0)
    {
        *yaw = attitude.yaw;
    }
}

/**
 * @brief  获取姿态角指针（欧拉角，单位：rad）
 * @retval 指向 yaw/pitch/roll 数组的指针
 */
const fp32 *get_INS_angle_point(void)
{
    return mpu6050_ins_angle;
}

/**
 * @brief  更新姿态角缓存（欧拉角，单位：rad）
 * @retval None
 */
void MPU6050_Update_INS_Angle(void)
{
    MPU6050_Attitude attitude;
    if (MPU_Get_Attitude(&attitude) == 0)
    {
        const fp32 deg_to_rad = 0.017453292519943295f;
        mpu6050_ins_angle[INS_YAW_ADDRESS_OFFSET] = attitude.yaw * deg_to_rad;
        mpu6050_ins_angle[INS_PITCH_ADDRESS_OFFSET] = attitude.pitch * deg_to_rad;
        mpu6050_ins_angle[INS_ROLL_ADDRESS_OFFSET] = attitude.roll * deg_to_rad;
    }
}