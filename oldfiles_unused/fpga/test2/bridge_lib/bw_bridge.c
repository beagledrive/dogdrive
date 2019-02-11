#include "bw_bridge.h"

int bridge_init(struct bridge *br, uint32_t mem_address, uint32_t mem_size) {
	uint32_t page_mask;
	uint32_t page_size;

	page_size = sysconf(_SC_PAGESIZE);
	br->alloc_mem_size = (((mem_size / page_size) + 1) * page_size);
	page_mask = (page_size - 1);
	//printf("Allocated mem size, 0x%X\n", br->alloc_mem_size);
	//printf("PM, PS, 0x%X 0x%X\n", page_mask, page_size);
	//printf("Mem addr, 0x%X\n",mem_address);
	//printf("Mem size, 0x%X\n",mem_size);

	br->mem_dev = open("/dev/mem", O_RDWR | O_SYNC);
	if (br->mem_dev < 0)
	{
		printf("Can't open dev/mem\n");
		return -EPERM;
	}

	br->mem_pointer = mmap(NULL,
		               br->alloc_mem_size,
			       PROT_READ | PROT_WRITE,
                               MAP_SHARED,
                               br->mem_dev,
                               (mem_address & ~page_mask));

	//printf("Mem pointer, value, 0x%X %X\n",(uint32_t)(uint32_t*)br->mem_pointer, *(uint32_t*)br->mem_pointer);
	if(br->mem_pointer == MAP_FAILED) {
		printf("Failed to map memory");
	        return -ENOMEM;
	}

	//printf("Mem addr& page mask 0x%X\n", (mem_address & page_mask));
	br->virt_addr = (br->mem_pointer + (mem_address & page_mask));

	//printf("Virtual address, value, 0x%X %X\n",(uint32_t)(uint32_t*)br->virt_addr, *(uint32_t*)br->virt_addr);
	return 0;
}

void bridge_close(struct bridge *br) {
	if (munmap(br->mem_pointer, br->alloc_mem_size) == -1)
		perror("Error un-mmapping the file");

	close(br->mem_dev);
}

uint16_t get_word(struct bridge *br, uint16_t reg_addr) {
	return *(uint16_t *)(br->virt_addr + reg_addr);
}

void set_word(struct bridge *br, uint16_t reg_addr, uint16_t word) {
	*(uint16_t *)(br->virt_addr + reg_addr) = word;
}

void set_fpga_mem(struct bridge *br, uint16_t reg_addr, const uint16_t* source,
		     size_t reg_num) {
	unsigned int c;
	for (c = 0; c < reg_num; c++)
		*(uint16_t *)(br->virt_addr + reg_addr + c*2) = source[c];
}

void get_fpga_mem(struct bridge *br, uint16_t reg_addr, uint16_t* destination,
		     size_t reg_num) {
	unsigned int c;
	uint16_t *udst = (uint16_t *)destination;
	for (c = 0; c < reg_num; c++)
		udst[c] = *(uint16_t *)(br->virt_addr + reg_addr + c*2);
}
