#define _XOPEN_SOURCE 500

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>


#include "bw_bridge.h"

#define GPIO_START_ADDR		0x4804C000
#define GPIO_END_ADDR		0x4804DFFF
#define GPIO_SIZE (GPIO_END_ADDR - GPIO_START_ADDR)
#define GPIO_OE			0x134
#define GPIO_DATAIN		0x138
#define GPIO_SETDATAOUT		0x194
#define GPIO_CLEARDATAOUT	0x190

#define PIN (1 << 17)	

int write_dsp_data();
uint16_t sw_on = 0;
uint16_t sw_off = 7500;
struct bridge br;
uint16_t setupLW;

int write_dsp_data()
{
	unsigned int b_error = 1;


	//uint16_t src = 0xFFFF;
	//uint16_t dest = 0;


	//uint16_t setupUW = 0b00000000;
	uint16_t dsp_err = 0;

	//	uint32_t sw_on = 5000; //clock
	sw_off = 7500; 
	if(sw_on == 0){
		sw_on = 8000;
	} else{
		sw_on = 0;
	}
	
	setupLW = (uint16_t) ((setupLW + 0b00001000)& 0x000F);

//	uint16_t periodUW = (uint16_t) (sw_on >> 16); //ns
	uint16_t periodLW = (uint16_t) (sw_on & 0x0000FFFF); 
//	uint16_t dutyUW = (uint16_t) (sw_off >> 16); //ns
	uint16_t dutyLW = (uint16_t) (sw_off & 0x0000FFFF);

	/*	
	uint16_t periodLW = 0xC200; //(uint16_t) period >> 16; //ns
	uint16_t periodUW = 0x0BEB; //(uint16_t) period >> 16; //ns

	uint16_t dutyLW = 0x0000; //(uint16_t) duty >> 16; //ns
	uint16_t dutyUW = 0x0830; //(uint16_t) duty >> 16; //ns
	*/

	uint16_t data[] = {periodLW, dutyLW, setupLW};
	int regAddr [] = {2,3,0};


	set_fpga_mem(&br, (uint16_t) 1*sizeof(uint16_t), &dsp_err, 1);

	//printf("Memory Address: 0x%X\n", BW_BRIDGE_MEM_ADR);

	unsigned int i = 0;	
//	printf("Data Write:---------------------------- \n");	
	while(i < sizeof(data)/sizeof(uint16_t))
	{
//		printf("Index: %X Written: %X \n", regAddr[i], data[i]);
		set_fpga_mem(&br, (uint16_t) regAddr[i]*sizeof(uint16_t), &data[i], 1);
		i++;	
//		printf("Data: 0x%X\n", dest);
		//src++;
	}
	
/*	int regRetAddr [] = {4,5,6};
	uint16_t retrieveData;
	i = 0;
//	printf("Data Read:----------------------------- \n");	
	while (i <  sizeof(data)/sizeof(uint16_t))
	{
		get_fpga_mem(&br, (uint16_t) regRetAddr[i]*sizeof(uint16_t), &retrieveData, 1);
//		printf("Index: %X  Retrieved: 0x%X \n", regRetAddr[i], retrieveData);
		i++;
	}
	
//	printf("--------------------------------------- \n");
*/	b_error = 0;
	return b_error;
}


int main()
{
	volatile void *gpio_addr = NULL;
	//volatile unsigned int *gpio_oe_addr = NULL;
	volatile unsigned int *gpio_datain = NULL;
	//volatile unsigned int *gpio_setdataout_addr = NULL;
	//volatile unsigned int *gpio_cleardataout_addr = NULL;
	//unsigned int reg;
	//
	int latch = 0;

	int fd = open("/dev/mem", O_RDWR);
	
	gpio_addr = mmap(0, GPIO_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO_START_ADDR);

	//gpio_oe_addr = gpio_addr + GPIO_OE;
	gpio_datain = gpio_addr + GPIO_DATAIN;
        //gpio_setdataout_addr = gpio_addr + GPIO_SETDATAOUT;
	//gpio_cleardataout_addr = gpio_addr + GPIO_CLEARDATAOUT;

	if (gpio_addr == MAP_FAILED)
	{
		fprintf(stderr, "Unable to map GPIO\n");
		return -1;
	}	

	//printf("GPIO mapped to %p\n", gpio_addr);
	//printf("GPIO_OE mapped to %p \n", gpio_oe_addr);
	//printf("GPIO SetDataOut_Addr mapped to %p\n", gpio_setdataout_addr);
	//printf("GPIO ClearDataOut_Addr mapped to %p\n", gpio_cleardataout_addr);

/*
	reg = *gpio_oe_addr;
	printf("GPIO configuration: %X\n", reg);
	reg = reg & (0xFFFFFFFF - PIN);
	*gpio_oe_addr = reg;
	printf("GPIO configuration: %X\n", reg);
*/
	int bridge_error = 0;
	if (bridge_init(&br, BW_BRIDGE_MEM_ADR, BW_BRIDGE_MEM_SIZE) < 0)
	{
		bridge_error = 1;
	}
	if (!bridge_error) 
	{

		// Enable PWM
		// reset 0, enable 1, polarity 0
		printf("Enable PWM\n");
		setupLW = 0b00001011;
		set_fpga_mem(&br, (uint16_t) 0*sizeof(uint16_t), &setupLW, 1);
//		setupLW = 0b00000010;
//		set_fpga_mem(&br, (uint16_t) 0*sizeof(uint16_t), &setupLW, 1);
//		printf("Wait for GPIO Signal\n");
		while(1)
		{
			if (*gpio_datain & PIN && !latch)
			{
				latch = 1;
				if (write_dsp_data())
				{
					printf("ERRRR\n");
					break;
				}
			} else
			{
				latch = 0;
			}
		}
	}else{
		printf("Bridge failed to open");
	}

	bridge_close(&br);
	munmap((void*)gpio_addr, GPIO_SIZE);
	close(fd);
	return 0;












	/*
	// READ GPIO - functions similar to "cat /sys/class/gpio/gpio49/value"
	int fd;
	char buffer[16];

	if ((fd = open("/sys/class/gpio/gpio49/value", O_RDONLY)) < 0) 
	{
		fprintf(stderr, "Can't read file \n");
	}

	struct pollfd pfd;
	pfd.fd = fd;
	pfd.events = POLL_GPIO;
	

	while(1)
	{
		fprintf(stderr, "Waiting for poll\n");
		if (poll(&pfd, 1, -1) > 0)
		{
			if (pfd.revents == POLL_GPIO) {
				lseek(fd, 0, SEEK_SET);
				if (read(fd, buffer, sizeof buffer) == -1) 
				{
					//Error
				} else if (write_dsp_data())
				{
					printf("ERRRR\n");
					break;
				}
			}
		}

		*/	
		/*	

		fgets(buffer, sizeof(buffer), io);
		gpio_pin = atoi(buffer);
		printf("GPIO_value %d\n", gpio_pin);
		fseek(io, 0, SEEK_SET);
		fflush(io);
		usleep(10000);

		// Call function here
		if (gpio_pin == 1) 
		{
			if(write_dsp_data())
			{
				printf("ERRRRR");
				break;
			}
		}
	}



	return 0*/;
}
