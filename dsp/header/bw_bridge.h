/*
 * Beaglebone Open-Source Machine Drive
 * Bridge function between DSP - FPGA 
 * KTH Project Work - 2018 
 */

#ifndef _BW_BRIDGE_H_		// Header wrapper name
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

/*
 * Communication bridge descriptor between DSP and FPGA
 */
struct bridge {
	void		*virt_addr;
	int		mem_dev;
	uint32_t	alloc_mem_size;
	void		*mem_pointer;
};


/* ================================== FUNCTION PROTOTYPES =================== */

/*
 * Initialize Bridge
 */
int bridge_init();

/*
 * Close bridge connection
 */
void bridge_close();

/* UNUSED */
uint16_t get_word(struct bridge *br, uint16_t reg_addr);

/* UNUSED */
void set_word(struct bridge *br, uint16_t reg_addr, uint16_t word);

/*
 * Set FPGA memory address
 * br - bridge descriptor
 * reg_addr - memory address of register
 * source - variable to write into register
 * reg_num - size of data to be written
 */
void set_fpga_mem(struct bridge *br, uint16_t reg_addr, const uint16_t* source,
		     size_t reg_num);

/*
 * Read FPGA memory address
 * br - bridge descriptor
 * reg_addr - memory address of register
 * destination - where to store data read
 * reg_num - size of data to be read
 */

void get_fpga_mem(struct bridge *br, uint16_t reg_addr, uint16_t* destination,
		  size_t reg_num);

#endif
