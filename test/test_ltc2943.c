#include <stdio.h>
#include <fcntl.h>
#include "ltc2943lib.h"

int main()
{
	int ret = 0;
	int fd = 0;
	unsigned char databuf[16] = {0};
	
	ret = open("/dev/ltc2943", O_RDWR);
	if(ret < 0)
	{
		goto error;
	}
	
	fd = ret;
	
	ret = ioctl(fd, SET_ALCC_MODE, 0);
	if(ret < 0)
	{
		goto error;
	}
	
	ret = ioctl(fd, SET_PRESCALER, 6);
	if(ret < 0)
	{
		goto error;
	}
	
	ret = ioctl(fd, SET_ADC_MODE, 3);
	if(ret < 0)
	{
		goto error;
	}
	
	ret = ioctl(fd, SET_ENABLE, 1);
	if(ret < 0)
	{
		goto error;
	}
	
	while(1)
	{
		int *pdata;
		ret = read(fd, databuf, sizeof(databuf));
		if(ret < sizeof(databuf))
		{
			goto error;
		}
		
		pdata = (int *)databuf;
		printf("charge:%lfmAh\n", ((double)pdata[0] * 0.000001));
		printf("volatge:%lfV\n", ((double)pdata[1] * 0.001));
		printf("current:%lfmA\n", ((double)pdata[2] * 1.0));
		printf("temp:%lfC\n", ((double)pdata[3] * 0.01));
		sleep(1);
	}
error:

	return ret;
}
