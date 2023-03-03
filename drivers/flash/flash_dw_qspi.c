#include "flash_dw_qspi.h"

#include <assert.h>

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT snps_qspi_nor

#define DW_QSPI_FLASH_DT DT_PATH(flash1, flash_1)

LOG_MODULE_REGISTER(flash_dw_qspi, CONFIG_FLASH_LOG_LEVEL);

static uint32_t _get_reg_field(const uint32_t addr, const uint32_t width,
			       const uint32_t lsb) {
	uint32_t reg = sys_read32(addr);
	uint32_t mask;
	uint32_t val;

	assert(width <= 32);
	mask = GET_MASK(width);

	val = ((reg >> lsb) & mask);
	LOG_DBG("DW RD: 0x%x(m:0x%x, lsb:%d) 0x%x => val 0x%x\n",
	      addr, mask, lsb, reg, val);
	return val;
}
#define DW_SPI_GET_REG_FIELD(REG, FIELD) \
	_get_reg_field(LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_##REG##__ADDRESS, \
		       LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_##REG##_DATA__##FIELD##__WIDTH, \
		       LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_##REG##_DATA__##FIELD##__LSB)
#define DW_XIP_GET_REG_FIELD(REG, FIELD) \
	_get_reg_field(LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_##REG##__ADDRESS, \
		       LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_##REG##_DATA__##FIELD##__WIDTH, \
		       LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_##REG##_DATA__##FIELD##__LSB)

static void _set_reg_field(const uint32_t addr, const uint32_t width,
			   const uint32_t lsb, const uint32_t val) {
	uint32_t orig = sys_read32(addr), reg;
	uint32_t mask;

	assert(width <= 32);
	mask = GET_MASK(width);

	reg = (orig & ~(mask << lsb)) | ((val & mask) << lsb);
	LOG_DBG("DW WR 0x%x(m:0x%x, lsb:%d, v:0x%x) 0x%x => 0x%x\n",
	      addr, mask, lsb, val, orig, reg);
	sys_write32(reg, addr);
}
#define DW_SPI_SET_REG_FIELD(REG, FIELD, VAL) \
	_set_reg_field(LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_##REG##__ADDRESS, \
		       LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_##REG##_DATA__##FIELD##__WIDTH, \
		       LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_##REG##_DATA__##FIELD##__LSB, \
		       VAL)
#define DW_XIP_SET_REG_FIELD(REG, FIELD, VAL) \
	_set_reg_field(LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_##REG##__ADDRESS, \
		       LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_##REG##_DATA__##FIELD##__WIDTH, \
		       LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_##REG##_DATA__##FIELD##__LSB, \
		       VAL)

struct flash_dw_qspi_config {
	uint32_t    freq;	  /* flash clock freq */
	uint32_t    ahb_freq; /* ahb bus clock frequency */
	uint32_t    pp_lanes; /* lanes used in page prog */
    uint8_t     clk_pol;  /* clock polarity */
    uint8_t     clk_pha;  /* clock phase */
    uint8_t     spi_frf;  /* spi frame format */
};

struct flash_dw_qspi_data {
	struct qspi_cmd_info command_info;
	struct qspi_dev_intf *interface;
	struct qspi_flash_info info;
};

void dw_spi_enable(const bool enable) {
	DW_SPI_SET_REG_FIELD(SSIENR, SSIC_EN, (enable) ? 1 : 0);
}

bool dw_spi_idle(void) {
	return (DW_SPI_GET_REG_FIELD(SR, TFE) &&
		DW_SPI_GET_REG_FIELD(SR, TFNF) &&
		!DW_SPI_GET_REG_FIELD(SR, BUSY));
}

static int dw_spi_trans_completed(const uint32_t drain_timeout_ms)
{
	int64_t start_time = k_uptime_get();
	int64_t uptime;

	while (!dw_spi_idle()) {
		uptime = k_uptime_get();
		if (uptime >= start_time + drain_timeout_ms)
			return -ETIMEDOUT;
	}
	return 0;
}

static int dw_spi_rx_bytes(uint8_t *rx, uint32_t rx_len)
{
	uint32_t i;
	int64_t start_time = k_uptime_get();
	int64_t uptime;

	if (rx_len > DW_SPI_RX_FIFO_LEN) {
		LOG_ERR("Rx len %d greater than max %d\n",
		      rx_len, DW_SPI_RX_FIFO_LEN);
		return -ERANGE;
	}

	/* simply read it out from the fifo */
	for (i = 0; i < rx_len; i++) {
		uint32_t reg;

		while (DW_SPI_GET_REG_FIELD(RXFLR, RXTFL) == 0) {
			uptime = k_uptime_get();
			if (uptime >= start_time + CONFIG_DW_SPI_READ_TIMEOUT_MS)
				return -EIO;
		}

		reg = sys_read32(LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_DR0__ADDRESS);
		LOG_DBG("%s: Reading %d value 0x%x\n", __func__, i, reg);
		*rx++ = reg;
	}

	return 0;
}

static int dw_spi_tx_bytes(uint8_t *tx, uint32_t tx_len) {
	int ret;
	uint32_t i;
	uint32_t data;

	/* as we park everytime after use, we should be idle */
	if (!dw_spi_idle())
		return -EPERM;

	if (tx_len > DW_SPI_TX_FIFO_LEN) {
		LOG_ERR("Tx len %d greater than max %d\n",
		      tx_len, DW_SPI_TX_FIFO_LEN);
		return -ERANGE;
	}

	DW_SPI_SET_REG_FIELD(TXFTLR, TFT, 0);
	DW_SPI_SET_REG_FIELD(TXFTLR, TXFTHR, tx_len - 1);
	for (i = 0; i < tx_len; i++) {
		data = *tx++;
		LOG_DBG("%s(): Write reg 0x%x with data 0x%x\n",
		      __func__, LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_DR0__ADDRESS, data);
		sys_write32(data, LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_DR0__ADDRESS);
	}

	/* wait for draining */
	ret = dw_spi_trans_completed(CONFIG_DW_SPI_DRAIN_TIMEOUT_MS);
	if (ret)
		LOG_ERR("%s: Timeout %d ms on waiting for %d tx\n",
		      __func__, CONFIG_DW_SPI_DRAIN_TIMEOUT_MS, tx_len);
	return ret;
}

int dw_spi_transfer(const struct device *dev, uint8_t *tx, uint32_t tx_len,
		    uint8_t *rx, uint32_t rx_len) {
	int ret;
	uint32_t frf;
	bool duplex = ((rx != NULL) && (rx_len));

	/* Check params */
	if ((tx == NULL) || (tx_len == 0)) {
		LOG_ERR("Must have tx data\n");
		return -EPERM;
	}

	frf = DW_SPI_GET_REG_FIELD(CTRLR0, SPI_FRF);

	/* set up some parameters */
	dw_spi_enable(false);
	if (duplex) {
		DW_SPI_SET_REG_FIELD(CTRLR0, TMOD,
				     (frf == SPI_STD) ?
				     EEPROM_READ : RX_ONLY);
		DW_SPI_SET_REG_FIELD(CTRLR1, NDF, rx_len - 1);
		DW_SPI_SET_REG_FIELD(CTRLR0, SSTE, 0);
	} else {
		DW_SPI_SET_REG_FIELD(CTRLR0, TMOD, TX_ONLY);
		DW_SPI_SET_REG_FIELD(CTRLR1, NDF, DW_SPI_NDF_DEFAULT);
		DW_SPI_SET_REG_FIELD(CTRLR0, SSTE, 1);
	}
	dw_spi_enable(true);

	/* Send command and optionally address + data */
	ret = dw_spi_tx_bytes(tx, tx_len);
	if (ret)
		goto done;

	if (duplex) {
		/* Read data */
		ret = dw_spi_rx_bytes(rx, rx_len);
		if (ret)
			goto done;
	}

done:
	return ret;
}

int dw_spi_proc_completed(const struct device *dev, const uint32_t comp_timeout_ms)
{
	int ret;
	uint8_t cmd, stat;
	int64_t start_time = k_uptime_get();
	int64_t uptime;
	struct flash_dw_qspi_data *qspi_data = (struct flash_dw_qspi_data*) dev->data;
	struct qspi_cmd_info *p_cmd = &qspi_data->command_info;

	do {
		cmd = p_cmd->read_status;
		ret = dw_spi_transfer(dev, &cmd, sizeof(cmd),
				      &stat, sizeof(stat));
		if (ret)
			break;

		if (!(stat & (0x1 << p_cmd->busy_bit_pos)))
			break;

		uptime = k_uptime_get();
		if (uptime >= start_time + comp_timeout_ms)
			return -ETIMEDOUT;
	} while (1);

	return ret;
}

static int dw_spi_aligned_write(const struct device *dev,
				uint8_t cmd_code,
				uint32_t addr,
				const ADDR_L addrl,
				uint8_t *data,
				uint32_t len)
{
	bool once;
	uint32_t *w_p = (uint32_t *)data;
	uint32_t word;
	uint32_t i, remaining, write_sz, write_wd, thre;
	int ret = 0;
	uint32_t spi_frf, clk_stretch;
	uint8_t command;
	struct flash_dw_qspi_data *qspi_data = (struct flash_dw_qspi_data*) dev->data;
	struct flash_dw_qspi_config *cfg = (struct flash_dw_qspi_config*) dev->config;

	/* retrive parameters that need to be restored */
	spi_frf = DW_SPI_GET_REG_FIELD(CTRLR0, SPI_FRF);
	clk_stretch = DW_SPI_GET_REG_FIELD(SPI_CTRLR0, CLK_STRETCH_EN);

	remaining = len;
	once = false;
	while ((remaining) || (!once)) {
		once = true;
		write_sz = MIN(remaining, MAX_BYTE_PER_TRANS);
		/* always assume 4byte alignment */
		write_wd = (write_sz >> 2);

		/* Write enable
		 * Need WEL for every transaction because some flash
		 * devices reset WEL latch after the write is complete
		 * The addrl and dfs change will not affect this command
		 * as it does not have address or data.
		 */
		command = qspi_data->command_info.write_enable;
		ret = dw_spi_transfer(dev, &command, sizeof(command), NULL, 0);
		if (ret)
			break;

		/* set up parameters based on transfer mode */
		dw_spi_enable(false);
		DW_SPI_SET_REG_FIELD(SPI_CTRLR0, ADDR_L, addrl);
		DW_SPI_SET_REG_FIELD(SPI_CTRLR0, CLK_STRETCH_EN, 0);

		/* lane mode is based on configuration */
		DW_SPI_SET_REG_FIELD(CTRLR0, SPI_FRF, cfg->pp_lanes);

		/* always 32bit for DFS and TX_ONLY */
		DW_SPI_SET_REG_FIELD(CTRLR0, DFS, DEFAULT_DATA_TRANS_BIT_SZ - 1);
		DW_SPI_SET_REG_FIELD(CTRLR0, TMOD, TX_ONLY);
		dw_spi_enable(true);

		/*
		 * set the start transmit level so that tx starts before the
		 * last word is write. + 1 as there are always 2 extra, inst
		 * + addr.  If address is skipped, then 1 less
		 */
		thre = (addrl != ADDR_L0) ? write_wd + 1 : write_wd;
		DW_SPI_SET_REG_FIELD(TXFTLR, TXFTHR, thre);
		LOG_DBG("CmdCode 0x%x, addrl 0x%x addr 0x%x, write_wd %d thre %d\n",
		      cmd_code, addrl, addr, write_wd, thre);

		/* start pushing to DR */
		sys_write32(cmd_code, LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_DR0__ADDRESS);
		if (addrl != ADDR_L0) {
			sys_write32(addr, LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_DR0__ADDRESS);
			addr += write_sz;
		}
		for (i = 0; i < write_wd; i++) {
			/* Swap the byte-order as required by the QSPI controller */
			word = *w_p++;
			word = BSWAP_32(word);

			sys_write32(word, LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_DR0__ADDRESS);
		}

		/* wait until all drained on DW core */
		ret = dw_spi_trans_completed(CONFIG_DW_SPI_DRAIN_TIMEOUT_MS);

		/* always restore parameters after write */
		dw_spi_enable(false);
		DW_SPI_SET_REG_FIELD(SPI_CTRLR0, ADDR_L, ADDR_L0);
		DW_SPI_SET_REG_FIELD(SPI_CTRLR0, CLK_STRETCH_EN, clk_stretch);
		DW_SPI_SET_REG_FIELD(CTRLR0, SPI_FRF, spi_frf);
		DW_SPI_SET_REG_FIELD(CTRLR0, DFS, DEFAULT_DFS_BIT_SZ - 1);
		dw_spi_enable(true);

		if (ret)
			break;

		/* wait until write is complete on the flash */
		ret = dw_spi_proc_completed(dev, CONFIG_DW_SPI_DRAIN_TIMEOUT_MS);
		if (ret)
			break;

		remaining -= write_sz;
	}

	if (ret)
		LOG_ERR("%s: Failed waiting write_sz %d trans- remaining 0x%x\n",
		      __func__, write_sz, remaining);

	return ret;
}

void dw_spi_reset() {
	dw_spi_enable(false);

	/* disable all interrupts */
	DW_SPI_SET_REG_FIELD(IMR, TXEIM, DW_SPI_DISABLE);
	DW_SPI_SET_REG_FIELD(IMR, TXOIM, DW_SPI_DISABLE);
	DW_SPI_SET_REG_FIELD(IMR, RXFIM, DW_SPI_DISABLE);
	DW_SPI_SET_REG_FIELD(IMR, RXOIM, DW_SPI_DISABLE);
	DW_SPI_SET_REG_FIELD(IMR, RXUIM, DW_SPI_DISABLE);

    dw_spi_enable(true);
}

int dw_spi_configure_trans_mode(const enum qspi_trans_mode trans_mode) {
	static const struct {
		SPI_FRF ffmt;
		TRANS_TYPE ttype;
	} trans_mode_tab[QSPI_TRANS_MODE_MAX] = {
		[QSPI_TRANS_MODE_1_1_1] = {SPI_STD,    TT0},
		[QSPI_TRANS_MODE_1_1_2] = {SPI_DUAL,   TT0},
		[QSPI_TRANS_MODE_1_2_2] = {SPI_DUAL,   TT1},
		[QSPI_TRANS_MODE_2_2_2] = {SPI_DUAL,   TT2},
		[QSPI_TRANS_MODE_1_1_4] = {SPI_QUAD,   TT0},
		[QSPI_TRANS_MODE_1_4_4] = {SPI_QUAD,   TT1},
		[QSPI_TRANS_MODE_4_4_4] = {SPI_QUAD,   TT2},
	}, *entry;

	if ((trans_mode < 0) || (trans_mode >= QSPI_TRANS_MODE_MAX)) {
		LOG_ERR("Invalid lane mode %d\n", trans_mode);
		return -EPERM;
	}

	entry = &trans_mode_tab[trans_mode];
	/* Note: device has been disabled when it is entered */
	DW_SPI_SET_REG_FIELD(CTRLR0, SPI_FRF, entry->ffmt);
	DW_SPI_SET_REG_FIELD(SPI_CTRLR0, TRANS_TYPE, entry->ttype);
	return 0;
}

void dw_spi_set_dummy_cycles(const uint8_t dummy_cycles) {
	/* NOTE: assume device is disabled already */
	DW_SPI_SET_REG_FIELD(SPI_CTRLR0, WAIT_CYCLES, dummy_cycles);
}

void dw_spi_set_command_byte(uint8_t command) {
	/* Note: API has assumed device is disabled */
	DW_XIP_SET_REG_FIELD(INCR_INST, INCR_INST, command);
}

void dw_spi_set_xip_addrl(const ADDR_L addrl) {
	dw_spi_enable(false);
	DW_XIP_SET_REG_FIELD(CTRL, ADDR_L, addrl);
	dw_spi_enable(true);
}

void dw_spi_common_init(const struct device *dev) {
	uint32_t sck_dv;
	struct flash_dw_qspi_config *cfg = (struct flash_dw_qspi_config*) dev->config;

	/** dw_spi initialization **/
	dw_spi_reset();

	/* set up all necessary parameters */
	dw_spi_enable(false);

	/*
	 * SPI master mode && SER - no need to set, and use default
	 * according to ASIC.
	 *
	 * DW_SPI_SET_REG_FIELD(CTRLR0, SSI_IS_MST, ENABLE);
	 * DW_SPI_SET_REG_FIELD(SER, SER, 0x1 << ss);
	 */

	/* Baud rate */
	sck_dv = cfg->ahb_freq / cfg->freq;
	DW_SPI_SET_REG_FIELD(BAUDR, SCKDV, sck_dv);

	/* Clock phase and polarity */
	DW_SPI_SET_REG_FIELD(CTRLR0, SCPH, QSPI_CLOCK_PHASE(cfg));
	DW_SPI_SET_REG_FIELD(CTRLR0, SCPOL, QSPI_CLOCK_POL(cfg));

	/* frame format and transfer data frame size */
	DW_SPI_SET_REG_FIELD(CTRLR0, SSTE, 0);
	DW_SPI_SET_REG_FIELD(CTRLR0, FRF, SPI_PROT);
	DW_SPI_SET_REG_FIELD(CTRLR0, SPI_FRF, SPI_STD);

	DW_SPI_SET_REG_FIELD(CTRLR0, DFS, DEFAULT_DFS_BIT_SZ - 1);

	/* Default transfer parameters to use */
	DW_SPI_SET_REG_FIELD(SPI_CTRLR0, INST_L, INST_L8);
	DW_SPI_SET_REG_FIELD(SPI_CTRLR0, ADDR_L, ADDR_L0);
	DW_SPI_SET_REG_FIELD(SPI_CTRLR0, TRANS_TYPE, TT0);
	DW_SPI_SET_REG_FIELD(SPI_CTRLR0, CLK_STRETCH_EN, 0);

	/* set both TX/TX THRE to 0 to start with */
	DW_SPI_SET_REG_FIELD(TXFTLR, TFT, 0);
	DW_SPI_SET_REG_FIELD(RXFTLR, RFT, 0);
	dw_spi_enable(true);
}

int dw_spi_send_cmd(const struct device *dev,
		    		uint8_t cmd_code,
		    		uint32_t aux_data,
		    		uint32_t dlen)
{
	int ret;
	static const ADDR_L tab[] = {
		[0] = ADDR_L0,
		[1] = ADDR_L8,
		[2] = ADDR_L16,
		[3] = ADDR_L24,
		[4] = ADDR_L32,
	};

	if (dlen > sizeof(uint32_t))
		return -ERANGE;

	ret = dw_spi_aligned_write(dev, cmd_code, aux_data, tab[dlen],
				   NULL, 0);
	return ret;
}

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

	uint8_t command, resp;
	int ret = ENOSYS;
	struct flash_dw_qspi_config *cfg = (struct flash_dw_qspi_config*) dev->config;

	dw_spi_common_init(dev);

	dw_spi_enable(false);
	dw_spi_configure_trans_mode(QSPI_TRANS_MODE_1_1_1);
	switch (cfg->spi_frf) {
	default:
	case SPI_STD:
	case SPI_DUAL:
		dw_spi_set_command_byte(QSPI_SPANSION_CMD_1_1_2_READ);
		dw_spi_set_dummy_cycles(QSPI_SPANSION_1_1_2_READ_DUMMY);
		break;
	case SPI_QUAD:
		dw_spi_set_command_byte(QSPI_SPANSION_CMD_1_1_4_READ);
		dw_spi_set_dummy_cycles(QSPI_SPANSION_1_1_4_READ_DUMMY);
		break;
	}
	dw_spi_enable(true);

		ret = dw_spi_send_cmd(dev, QSPI_SPANSION_CMD_WRR, QSPI_SPANSION_WRR_QUAD_MODE_CMD, 2 /* cfg + stat reg */);
	if (ret)
		goto fail;

	/* read back cfg bits and verify */
	command = QSPI_SPANSION_CMD_RDCR;
	ret = dw_spi_transfer(dev, &command, sizeof(command),
			      &resp, sizeof(resp));
	if (ret)
		goto fail;

	LOG_INF("Flash QUAD mode set completed - CR 0x%x\n", resp);

	if (!(resp & (1 << QSPI_SPANSION_CFG_QUAD_BIT_POS)))
		ret = -EPERM;
fail:
	return ret;
}

#define DEFINE_DW_QSPI(inst) 									\
	static struct flash_dw_qspi_data flash_dw_qspi_data_##inst = {	\
	};								\
  	static const struct flash_dw_qspi_config flash_dw_qspi_config_##inst = {	\
		.freq = DT_INST_PROP(inst, clock_frequency), \
		.ahb_freq = DT_INST_PROP(inst, ahb_bus_frequency), \
		.pp_lanes = DT_INST_PROP(inst, pp_lanes), \
	};								\
	BUILD_ASSERT(DT_INST_PROP(inst, ahb_bus_frequency) != 0); \
	DEVICE_DT_INST_DEFINE(inst,					\
			flash_dw_qspi_init,					\
			NULL,						\
			&flash_dw_qspi_data_##inst,				\
			&flash_dw_qspi_config_##inst,			\
			POST_KERNEL,					\
			CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
			&flash_dw_qspi_api)

DT_INST_FOREACH_STATUS_OKAY(DEFINE_DW_QSPI);
