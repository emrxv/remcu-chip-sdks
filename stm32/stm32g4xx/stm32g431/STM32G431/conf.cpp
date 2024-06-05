
#include "target.h"
#include "AddressInterval.h"

namespace remcu {

void setConfig(){
    clearConfig();
    add_to_mem_interval(0x20000000, 0x20000000 + 8*1024); 		//SRAM
	add_to_adin_interval(0x40000000,  0x40009800); //APB1
    add_to_adin_interval(0x40010000,  0x40016400); //APB2
    add_to_adin_interval(0x40020000,  0x40024400); //AHB1
    add_to_adin_interval(0x48000000,  0x50060C00); //AHB2

}

uint32_t get_RAM_addr_for_test(){
    return 0x20000000;
}

} //namespace