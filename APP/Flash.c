#include "Flash.h"

void w25qxx_init_usr(w25qxx_handle_t *handle)
{
    uint8_t manufacturer = 0;
	uint8_t device_id[2] = {0};
	uint8_t res = 0;

    //W25Q128 SPI 初始化 
    DRIVER_W25QXX_LINK_INIT(handle, w25qxx_handle_t);
    DRIVER_W25QXX_LINK_SPI_QSPI_INIT(handle, w25qxx_interface_spi_qspi_init);
    DRIVER_W25QXX_LINK_SPI_QSPI_DEINIT(handle, w25qxx_interface_spi_qspi_deinit);
    DRIVER_W25QXX_LINK_SPI_QSPI_WRITE_READ(handle, w25qxx_interface_spi_qspi_write_read);
    DRIVER_W25QXX_LINK_DELAY_MS(handle, w25qxx_interface_delay_ms);
    DRIVER_W25QXX_LINK_DELAY_US(handle, w25qxx_interface_delay_us);
    DRIVER_W25QXX_LINK_DEBUG_PRINT(handle, w25qxx_interface_debug_print);

    handle->type = W25Q128; 
	res = w25qxx_init(handle);
	if(res == 0){
		res = w25qxx_get_jedec_id(handle, &manufacturer, device_id);
		if(res == 0){
			printf("W25QXX JEDEC ID: manufacturer=0x%02X, device_id=0x%02X 0x%02X\r\n", manufacturer, device_id[0], device_id[1]);
		}else{
			printf("读取W25QXX JEDEC ID失败\r\n");
		}
	}else{
		printf("W25QXX初始化失败 error code: %d\r\n",res);
	}
}
