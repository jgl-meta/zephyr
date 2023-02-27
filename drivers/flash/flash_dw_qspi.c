#include "flash_dw_qspi.h"

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT snps_qspi_nor

LOG_MODULE_REGISTER(flash_dw_qspi, CONFIG_FLASH_LOG_LEVEL);

struct flash_dw_qspi_config {

};

struct flash_dw_qspi_data {

};

static int flash_dw_qspi_read(const struct device *dev, off_t offset,
			      void *data,
			      size_t len) {

    return -ENOSYS;
}
static int flash_dw_qspi_write(const struct device *dev, off_t offset,
			       const void *data, size_t len) {
                    return -ENOSYS;
                   }

static int flash_dw_qspi_erase(const struct device *dev, off_t offset,
			       size_t size) {
                    return -ENOSYS;
                   }

static const struct flash_parameters* flash_dw_qspi_get_parameters(const struct device *dev) {
    return NULL;
}

static void flash_dw_qspi_pages_layout(const struct device *dev,
				       const struct flash_pages_layout **layout,
				       size_t *layout_size) {
                       }

static int flash_dw_qspi_sfdp_read(const struct device *dev, off_t offset,
				   void *data, size_t len) {
                    return -ENOSYS;
                   }
static int flash_dw_qspi_read_jedec_id(const struct device *dev, uint8_t *id) {
    return -ENOSYS;
}

static const struct flash_driver_api flash_dw_qspi_api = {
	.read = flash_dw_qspi_read,
	.write = flash_dw_qspi_write,
	.erase = flash_dw_qspi_erase,
	.get_parameters = flash_dw_qspi_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_dw_qspi_pages_layout,
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
#if defined(CONFIG_FLASH_JESD216_API)
	.sfdp_read = flash_dw_qspi_sfdp_read,
	.read_jedec_id = flash_dw_qspi_read_jedec_id,
#endif /* CONFIG_FLASH_JESD216_API */
};

static int flash_dw_qspi_init(const struct device *dev) {
    printk("HELLO FROM FLASH WORLD\n");
    return -ENOSYS;
}

#define DEFINE_DW_QSPI(inst) 									\
	static struct flash_dw_qspi_data flash_dw_qspi_data_##inst = {	\
	};								\
  	static const struct flash_dw_qspi_config flash_dw_qspi_config_##inst = {	\
	};								\
	DEVICE_DT_INST_DEFINE(inst,					\
			flash_dw_qspi_init,					\
			NULL,						\
			&flash_dw_qspi_data_##inst,				\
			&flash_dw_qspi_config_##inst,			\
			POST_KERNEL,					\
			CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
			&flash_dw_qspi_api)

DT_INST_FOREACH_STATUS_OKAY(DEFINE_DW_QSPI);
