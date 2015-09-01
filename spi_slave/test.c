#include <stdio.h>

int main(int argc,void *argv[])
{
	char buf[1024]={0};
	int i=0,len=0;
	int fd=open("/dev/s3c24xx_spi",0);
	if(fd<0)
	{
			printf("can not open /dev/s3c24xx_spi\n");
			return 0;
	}
	else
		printf("open /dev/s3c24xx_spi ok\n");
	len=read(fd,buf,1024);
	if(len>0)
	{
		printf("to print \n");
		for(i=0;i<len;i++)
		{
			printf("%02x ",buf[i]);
			if((((i+1)%16)==0 && i!=0)||(buf[i]==255))
				printf("\n");
		}
		printf("\n");
	}
	else
		printf("no data \n");
	close(fd);
}
