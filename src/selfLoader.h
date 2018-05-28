#ifndef __SELF_LOADER__
#define __SELF_LOADER__

#include "MDR32Fx.h"
#include "loaders.h"





__ramfunc void selfloader_erase_mem(Loader_TypeDef *loader, uint16_t pagesCnt);
__ramfunc void selfloader_write_flash_page(Loader_TypeDef *loader, uint16_t numOfPage);
__ramfunc void selfloader_read_flash_page(Loader_TypeDef *loader, uint16_t numOfPage);






#endif //__SELF_LOADER__