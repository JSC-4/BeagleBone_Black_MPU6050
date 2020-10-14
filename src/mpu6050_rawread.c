/*
 =======================================================================================================
 Name        : mpu6050_rawread.c
 Author      : JSC
 Website	 : https://github.com/JSCBLOG/BeagleBone_Black_MPU6050
 Description : This project is to communicate with the mpu6050 sensor,
 	 	 	   using the BeagleBone Black. This was taken from:
 	 	 	   https://github.com/niekiran/EmbeddedLinuxBBB/tree/master/Project_Src/MPU6050_raw_read
 =======================================================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

// MPU6050 registers + addresses
#define MPU6050_SLAVE_ADDR			0x68
#define MPU6050_REG_PWR_MGMT_1		0x6B
#define MPU6050_REG_ACC_CONFIG		0x1C
#define MPU6050_REG_GYRO_CONFIG		0x1B

// Accelerometer address registers
#define MPU6050_REG_ACC_X_HIGH		0x3B
#define MPU6050_REG_ACC_X_LOW		0x3C
#define MPU6050_REG_ACC_Y_HIGH		0x3D
#define MPU6050_REG_ACC_Y_LOW		0x3E
#define MPU6050_REG_ACC_Z_HIGH		0x3F
#define MPU6050_REG_ACC_Z_LOW		0x40

// Gyro address registers
#define MPU6050_REG_GYRO_X_HIGH		0x43
#define MPU6050_REG_GYRO_X_LOW		0x44
#define MPU6050_REG_GYRO_Y_HIGH		0x45
#define MPU6050_REG_GYRO_Y_LOW		0x46
#define MPU6050_REG_GYRO_Z_HIGH		0x47
#define MPU6050_REG_GYRO_Z_LOW		0x48

// Full scale range for accelerometer and gyroscope
#define ACC_FS_SENSITIVITY_0		16384
#define ACC_FS_SENSITIVITY_1		8192
#define ACC_FS_SENSITIVITY_2		4096
#define ACC_FS_SENSITIVITY_3		2048

#define GYRO_FS_SENSITIVITY_0		131
#define GYRO_FS_SENSITIVITY_1		65.5
#define GYRO_FS_SENSITIVITY_2		32.8
#define GYRO_FS_SENSITIVITY_3		16.4

// Linux OS device file for i2c-2
#define I2C_DEVICE_FILE				"/dev/i2c-2"

int fd;

int mpu6050_write(uint8_t addr, uint8_t data)
{
	int ret;
	char buf[2];

	buf[0] = addr;
	buf[1] = data;

	ret = write(fd, buf, 2);
	if (ret <= 0)
	{
		perror("Write failed\n");
		return -1;
	}
	return 0;
}

/*read "len" many bytes from "addr" of the sensor in to the adresss indicated by "pBuffer" */
int mpu6050_read(uint8_t base_addr, char *pBuffer,uint32_t len)
{
  int ret;
  char buf[2];
  buf[0]=base_addr;
  ret = write(fd,buf,1);
  if (ret <= 0)
  {
      perror("write address failed\n");
      return -1;
  }

  ret = read(fd,pBuffer,len);
  if(ret <= 0)
  {
      perror("read failed\n");
  }
  return 0;
}

void mpu6050_read_acc(short *pbuffer)
{
	char acc_buffer[6];

	mpu6050_read(MPU6050_REG_ACC_X_HIGH, acc_buffer, 6);

	pbuffer[0] = ( (acc_buffer[0] << 8) + acc_buffer[1] );
	pbuffer[1] = ( (acc_buffer[2] << 8) + acc_buffer[3] );
	pbuffer[2] = ( (acc_buffer[3] << 8) + acc_buffer[5] );
}

void mpu6050_read_gyro(short *pbuffer)
{
	char gyro_buffer[6];

	mpu6050_read(MPU6050_REG_ACC_X_HIGH, gyro_buffer, 6);

	pbuffer[0] = ( (gyro_buffer[0] << 8) + gyro_buffer[1] );
	pbuffer[1] = ( (gyro_buffer[2] << 8) + gyro_buffer[3] );
	pbuffer[2] = ( (gyro_buffer[3] << 8) + gyro_buffer[5] );
}

void mpu6050_init()
{
	// Take mpu6050 out of sleep
	mpu6050_write(MPU6050_REG_PWR_MGMT_1, 0x00);
	usleep(500);

	// Set FS_SEL to max
	mpu6050_write(MPU6050_REG_ACC_CONFIG, 0x18);
	usleep(500);

	// Set FS_SEL to max
	mpu6050_write(MPU6050_REG_GYRO_CONFIG, 0x18);
	usleep(500);
}

int main(void)
{

		short acc_value[3], gyro_value[3];
		double accx, accy, accz, gyrox, gyroy, gyroz;

		// Open I2C device file
		if ((fd = open(I2C_DEVICE_FILE, O_RDWR)) < 0)
		{
			perror("Failed to open I2C device file. \n");
			return -1;
		}

		// Set the I2C slave address using the ioctl I2C_SLAVE command
		if (ioctl(fd, I2C_SLAVE, MPU6050_SLAVE_ADDR) < 0)
		{
			perror("Failed to set I2C slave address. \n");
			close(fd);
			return -1;
		}

		mpu6050_init();

		while(1)
		{
			mpu6050_read_acc(acc_value);
			mpu6050_read_gyro(gyro_value);

			// Convert acc raw values into 'g' values
			accx = (double) acc_value[0]/ACC_FS_SENSITIVITY_3;
			accy = (double) acc_value[1]/ACC_FS_SENSITIVITY_3;
			accz = (double) acc_value[2]/ACC_FS_SENSITIVITY_3;

			// Convert gyro raw values into 'g' values
			gyrox = (double) gyro_value[0]/GYRO_FS_SENSITIVITY_3;
			gyroy = (double) gyro_value[1]/GYRO_FS_SENSITIVITY_3;
			gyroz = (double) gyro_value[2]/GYRO_FS_SENSITIVITY_3;

			printf("Acc(raw)=> X:%d Y:%d Z:%d gyro(raw)=> X:%d Y:%d Z:%d \n", \
				   acc_value[0],acc_value[1],acc_value[2],gyro_value[0],gyro_value[1],gyro_value[2]);

			/* print the 'g' and '°/s' values */
			printf("Acc(g)=> X:%.2f Y:%.2f Z:%.2f gyro(dps)=> X:%.2f Y:%.2f Z:%.2f \n", \
				   accx,accy,accz,gyrox,gyroy,gyroz);

			usleep(250 * 1000);
		}

}
