#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/errno.h>
#include <poll.h>  
#include <signal.h>  
char *start,loop=1;
FILE *file_fd;
int fd;
void my_signal_fun(int signum)  
{  
	int i,begin=0;
	//unsigned char key_val;  
    //read(fd, &key_val, 1); 
	//if(key_val)
	//	begin=2048;
	for(i=0;i<2048;i++)
	{
		printf("%02x ",start[i]);
		if((((i+1)%16)==0 && i!=0)||(start[i]==255))
			printf("\n");
	}

	//printf("%d\n",signum);
	//fwrite(start+begin,2048,1,file_fd);
	loop=0;
}
int main(int argc,void *argv[])
{
	int i=0,len=0;
	int Oflags;  
	fd=open("/dev/s3c24xx_spi",O_RDWR);
	if(fd<0)
	{
			printf("can not open /dev/s3c24xx_spi\n");
			return 0;
	}
	start=mmap(NULL,4096,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
	if(start==NULL || start ==MAP_FAILED)
	{
		printf("mmap failed %s\n",strerror(errno));
		close(fd);
		return 0;
	}
	file_fd=fopen("1","wb");
	signal(SIGIO, my_signal_fun); 
	fcntl(fd, F_SETOWN, getpid());  
	Oflags = fcntl(fd, F_GETFL);   
	fcntl(fd, F_SETFL, Oflags | FASYNC);  	
	while(loop)
		sleep(1000);  
	fclose(file_fd);
	munmap(start,4096);
	close(fd);
}
