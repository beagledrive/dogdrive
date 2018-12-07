/*
 * Beaglebone Open-Source Machine Drive
 * Bridge function between DSP - FPGA 
 * KTH Project Work - 2018 
 */

#ifndef _BW_BRIDGE_H_		// Header 2rapper name
#define _BW_BRIDGE_H_

#include <fcntl.h>
#include <sys/mman.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>


/* ================================== MACROS ================================ */


#define BW_BRIDGE_MEM_ADR 0x01000000
#define BW_BRIDGE_MEM_SIZE 0x2000


/* ================================== TYPEDEFS ============================== */


struct bridge {
	void		*virt_addr;
	int		mem_dev;
	uint32_t	alloc_mem_size;
	void		*mem_pointer;
};


/* ================================== FUNCTION PROTOTYPES =================== */

int bridge_init();

void bridge_close();

uint16_t get_word(struct bridge *br, uint16_t reg_addr);

void set_word(struct bridge *br, uint16_t reg_addr, uint16_t word);

void set_fpga_mem(struct bridge *br, uint16_t reg_addr, const uint16_t* source,
		     size_t reg_num);

void get_fpga_mem(struct bridge *br, uint16_t reg_addr, uint16_t* destination,
		  size_t reg_num);

#endif
