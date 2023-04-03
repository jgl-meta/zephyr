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

static uint32_t _get_reg_field(const uint32_t addr, const uint32_t width, const uint32_t lsb)
{
	uint32_t reg = sys_read32(addr);
	uint32_t mask;
	uint32_t val;

	assert(width <= 32);
	mask = GET_MASK(width);

	val = ((reg >> lsb) & mask);
	LOG_DBG("DW RD: 0x%x(m:0x%x, lsb:%d) 0x%x => val 0x%x\n", addr, mask, lsb, reg, val);
	return val;
}
#define DW_SPI_GET_REG_FIELD(REG, FIELD)                                                           \
	_get_reg_field(LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_##REG##__ADDRESS,                           \
		       LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_##REG##_DATA__##FIELD##__WIDTH,             \
		       LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_##REG##_DATA__##FIELD##__LSB)
#define DW_XIP_GET_REG_FIELD(REG, FIELD)                                                           \
	_get_reg_field(LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_##REG##__ADDRESS,                      \
		       LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_##REG##_DATA__##FIELD##__WIDTH,        \
		       LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_##REG##_DATA__##FIELD##__LSB)

static void _set_reg_field(const uint32_t addr, const uint32_t width, const uint32_t lsb,
			   const uint32_t val)
{
	uint32_t orig = sys_read32(addr), reg;
	uint32_t mask;

	assert(width <= 32);
	mask = GET_MASK(width);

	reg = (orig & ~(mask << lsb)) | ((val & mask) << lsb);
	LOG_DBG("DW WR 0x%x(m:0x%x, lsb:%d, v:0x%x) 0x%x => 0x%x\n", addr, mask, lsb, val, orig,
		reg);
	sys_write32(reg, addr);
}
#define DW_SPI_SET_REG_FIELD(REG, FIELD, VAL)                                                      \
	_set_reg_field(LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_##REG##__ADDRESS,                           \
		       LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_##REG##_DATA__##FIELD##__WIDTH,             \
		       LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_##REG##_DATA__##FIELD##__LSB, VAL)
#define DW_XIP_SET_REG_FIELD(REG, FIELD, VAL)                                                      \
	_set_reg_field(LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_##REG##__ADDRESS,                      \
		       LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_##REG##_DATA__##FIELD##__WIDTH,        \
		       LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_##REG##_DATA__##FIELD##__LSB, VAL)

struct flash_dw_qspi_config {
	uint32_t freq;	   /* flash clock freq */
	uint32_t ahb_freq; /* ahb bus clock frequency */
	uint32_t pp_lanes; /* lanes used in page prog */
	uint8_t clk_pol;   /* clock polarity */
	uint8_t clk_pha;   /* clock phase */
	uint8_t spi_frf;   /* spi frame format */
};

struct flash_dw_qspi_data {
	uint32_t flash_size;
};

void dw_spi_enable(const bool enable)
{
	DW_SPI_SET_REG_FIELD(SSIENR, SSIC_EN, (enable) ? 1 : 0);
}

bool dw_spi_idle()
{
	return (DW_SPI_GET_REG_FIELD(SR, TFE) && DW_SPI_GET_REG_FIELD(SR, TFNF) &&
		!DW_SPI_GET_REG_FIELD(SR, BUSY));
}

static int32_t dw_spi_trans_completed(const uint32_t drain_timeout_ms)
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

static int32_t dw_spi_rx_bytes(uint8_t *rx, uint32_t rx_len)
{
	uint32_t i;
	int64_t start_time = k_uptime_get();
	int64_t uptime;

	if (rx_len > DW_SPI_RX_FIFO_LEN) {
		LOG_ERR("Rx len %d greater than max %d\n", rx_len, DW_SPI_RX_FIFO_LEN);
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

static int32_t dw_spi_tx_bytes(uint8_t *tx, uint32_t tx_len)
{
	int ret;
	uint32_t i;
	uint32_t data;

	/* as we park everytime after use, we should be idle */
	if (!dw_spi_idle()) {
		ret = -EPERM;
		goto done;
	}

	if (tx_len > DW_SPI_TX_FIFO_LEN) {
		LOG_ERR("Tx len %d greater than max %d\n", tx_len, DW_SPI_TX_FIFO_LEN);
		ret = -ERANGE;
		goto done;
	}

	DW_SPI_SET_REG_FIELD(TXFTLR, TFT, 0);
	DW_SPI_SET_REG_FIELD(TXFTLR, TXFTHR, tx_len - 1);
	for (i = 0; i < tx_len; i++) {
		data = *tx++;
		LOG_DBG("%s(): Write reg 0x%x with data 0x%x\n", __func__,
			LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_DR0__ADDRESS, data);
		sys_write32(data, LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_DR0__ADDRESS);
	}

	/* wait for draining */
	ret = dw_spi_trans_completed(CONFIG_DW_SPI_DRAIN_TIMEOUT_MS);
	if (ret)
		LOG_ERR("%s: Timeout %d ms on waiting for %d tx\n", __func__,
			CONFIG_DW_SPI_DRAIN_TIMEOUT_MS, tx_len);
	return ret;

done:
	return ret;
}

int32_t dw_spi_transfer(uint8_t *tx, uint32_t tx_len, uint8_t *rx, uint32_t rx_len)
{
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
		DW_SPI_SET_REG_FIELD(CTRLR0, TMOD, (frf == SPI_STD) ? EEPROM_READ : RX_ONLY);
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

static inline void reg32clrbits(uintptr_t addr, uint32_t clear)
{
	sys_write32(sys_read32(addr) & ~clear, addr);
}

int32_t dw_spi_proc_completed(const struct device *dev, const uint32_t comp_timeout_ms)
{
	int ret;
	uint8_t cmd, stat;
	int64_t start_time = k_uptime_get();
	int64_t uptime;

	do {
		cmd = QSPI_SPANSION_CMD_READ_STATUS;
		ret = dw_spi_transfer(&cmd, sizeof(cmd), &stat, sizeof(stat));
		if (ret)
			break;

		if (!(stat & (0x1 << QSPI_SPANSION_CMD_BUSY_BIT_POS)))
			break;

		uptime = k_uptime_get();
		if (uptime >= start_time + comp_timeout_ms)
			return -ETIMEDOUT;
	} while (1);

	return ret;
}

static int32_t dw_spi_aligned_write(const struct device *dev, uint8_t cmd_code, uint32_t addr,
				    const ADDR_L addrl, const uint8_t *data, uint32_t len)
{
	bool once;
	uint32_t *w_p = (uint32_t *)data;
	uint32_t word;
	uint32_t i, remaining, write_sz, write_wd, thre;
	int ret = 0;
	uint32_t spi_frf, clk_stretch;
	uint8_t command;
	struct flash_dw_qspi_config *cfg = (struct flash_dw_qspi_config *)dev->config;

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

		command = QSPI_SPANSION_CMD_WRITE_ENABLE;
		ret = dw_spi_transfer(&command, sizeof(command), NULL, 0);
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
		LOG_DBG("CmdCode 0x%x, addrl 0x%x addr 0x%x, write_wd %d thre %d\n", cmd_code,
			addrl, addr, write_wd, thre);

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
		LOG_ERR("%s: Failed waiting write_sz %d trans- remaining 0x%x\n", __func__,
			write_sz, remaining);

	return ret;
}

void dw_spi_reset()
{
	dw_spi_enable(false);

	/* disable all interrupts */
	DW_SPI_SET_REG_FIELD(IMR, TXEIM, DW_SPI_DISABLE);
	DW_SPI_SET_REG_FIELD(IMR, TXOIM, DW_SPI_DISABLE);
	DW_SPI_SET_REG_FIELD(IMR, RXFIM, DW_SPI_DISABLE);
	DW_SPI_SET_REG_FIELD(IMR, RXOIM, DW_SPI_DISABLE);
	DW_SPI_SET_REG_FIELD(IMR, RXUIM, DW_SPI_DISABLE);

	dw_spi_enable(true);
}

int32_t dw_spi_configure_trans_mode(const enum qspi_trans_mode trans_mode)
{
	static const struct {
		SPI_FRF ffmt;
		TRANS_TYPE ttype;
	} trans_mode_tab[QSPI_TRANS_MODE_MAX] =
		{
			[QSPI_TRANS_MODE_1_1_1] = {SPI_STD, TT0},
			[QSPI_TRANS_MODE_1_1_2] = {SPI_DUAL, TT0},
			[QSPI_TRANS_MODE_1_2_2] = {SPI_DUAL, TT1},
			[QSPI_TRANS_MODE_2_2_2] = {SPI_DUAL, TT2},
			[QSPI_TRANS_MODE_1_1_4] = {SPI_QUAD, TT0},
			[QSPI_TRANS_MODE_1_4_4] = {SPI_QUAD, TT1},
			[QSPI_TRANS_MODE_4_4_4] = {SPI_QUAD, TT2},
		},
	  *entry;

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

void dw_spi_set_dummy_cycles(const uint8_t dummy_cycles)
{
	/* NOTE: assume device is disabled already */
	DW_SPI_SET_REG_FIELD(SPI_CTRLR0, WAIT_CYCLES, dummy_cycles);
}

void dw_spi_set_command_byte(uint8_t command)
{
	/* Note: API has assumed device is disabled */
	DW_XIP_SET_REG_FIELD(INCR_INST, INCR_INST, command);
}

void dw_spi_set_xip_addrl(const ADDR_L addrl)
{
	dw_spi_enable(false);
	DW_XIP_SET_REG_FIELD(CTRL, ADDR_L, addrl);
	dw_spi_enable(true);
}

void dw_spi_common_init(const struct device *dev)
{
	uint32_t sck_dv;
	struct flash_dw_qspi_config *cfg = (struct flash_dw_qspi_config *)dev->config;

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

int32_t dw_spi_send_cmd(const struct device *dev, uint8_t cmd_code, uint32_t aux_data,
			uint32_t dlen)
{
	int ret;
	static const ADDR_L tab[] = {
		[0] = ADDR_L0, [1] = ADDR_L8, [2] = ADDR_L16, [3] = ADDR_L24, [4] = ADDR_L32,
	};

	if (dlen > sizeof(uint32_t))
		return -ERANGE;

	ret = dw_spi_aligned_write(dev, cmd_code, aux_data, tab[dlen], NULL, 0);
	return ret;
}

static int32_t enable_xaddr(const struct device *dev)
{
	int ret;
	uint8_t cmd, resp;

	dw_spi_set_xip_addrl(ADDR_L32);
	ret = dw_spi_send_cmd(dev, QSPI_SPANSION_CMD_WRITE_EXT_ADDR, EXT_ADDR_ENABLED, 1);
	if (ret)
		goto fail;

	cmd = QSPI_SPANSION_CMD_ENABLE_EXT_ADDR;
	ret = dw_spi_transfer(&cmd, sizeof(cmd), &resp, sizeof(resp));
	if (ret)
		goto fail;

	if ((resp & EXT_ADDR_ENABLED) != EXT_ADDR_ENABLED) {
		LOG_ERR("EXT_ADDR not set successfully, val 0x%x\n", resp);
		ret = -EACCES;
	}
fail:
	return ret;
}

static int32_t spansion_sreset()
{
	uint8_t command;

	command = QSPI_SPANSION_CMD_SRESET;
	return dw_spi_transfer(&command, sizeof(command), NULL, 0);
}

static int32_t flash_dw_qspi_read_jedec_id(const struct device *dev, uint8_t *id)
{
	struct qspi_rdid_resp resp;
	uint8_t command;
	int32_t ret = 0;

	reg32clrbits(IO_PAD_CONTROL, 1 << CRM_LS_FORCE_OEB_TO_INPUT);
	dw_spi_common_init(dev);

	command = QSPI_JEDEC_READ_MFG_ID_COMMAND;
	ret = dw_spi_transfer(&command, sizeof(command), (uint8_t *)&resp, sizeof(resp));
	if (ret) {
		LOG_ERR("SPI TXFR error reading ID:%d\n", ret);
		goto done;
	}

	LOG_INF("MFG_ID 0x%x, dev_id 0x%x cap 0x%x for flash detected\n", resp.mgr_id, resp.dev_id,
		resp.cap);

	switch (resp.mgr_id) {
	case QSPI_SPANSION_MFG_ID:
		*id = resp.cap;
		break;
	default:
		ret = -EPERM;
		goto done;
	}

	ret = spansion_sreset();

done:
	return ret;
}

static uint32_t get_flash_size(const struct device *dev)
{
	int ret;
	uint32_t size = 0;
	uint8_t id = 0;

	ret = flash_dw_qspi_read_jedec_id(dev, &id);
	if (ret)
		goto fail;

	switch (id) {
	case 0x20:
		size = MBYTE_SZ(64);
		break;
	case 0x19:
		size = MBYTE_SZ(32);
		break;
	}
fail:
	return size;
}

static int32_t qspi_get_addr_list(const uint32_t addr, const uint32_t len,
				  uint32_t addr_list[MAX_SUB_ADDR], uint32_t sizes[MAX_SUB_ADDR])
{
	uint32_t num;

	if ((addr < SIZE_16MB) && ((addr + len - 1) >= SIZE_16MB)) {
		num = MAX_SUB_ADDR;
		addr_list[0] = addr;
		sizes[0] = SIZE_16MB - (addr & (SIZE_16MB - 1));
		addr_list[1] = addr + sizes[0];
		sizes[1] = len - sizes[0];
	} else {
		num = 1;
		addr_list[0] = addr;
		sizes[0] = len;
	}

	return num;
}

static int32_t disable_xaddr(const struct device *dev)
{
	int ret;
	ret = dw_spi_send_cmd(dev, QSPI_SPANSION_CMD_DISABLE_EXT_ADDR, EXT_ADDR_DISABLED, 1);
	dw_spi_set_xip_addrl(ADDR_L24);

	return ret;
}

static int32_t qspi_pwise_flash_erase(const struct device *dev, uint32_t address, uint32_t len)
{
	int ret = 0;
	uint8_t i;
	uint8_t command;
	uint32_t num_erases;
	uint8_t is_xaddr = IS_EXT_ADDR(address);
	uint32_t addr_bytes;

	/* Enter 4 byte mode */
	(is_xaddr) ? enable_xaddr(dev) : disable_xaddr(dev);

	/* Compute number of erases needed */
	num_erases = (len + CONFIG_DW_ERASE_SECTION_SIZE - 1) / CONFIG_DW_ERASE_SECTION_SIZE;
	command = QSPI_SPANSION_CMD_ERASE_256K;
	addr_bytes = is_xaddr ? XADDR_BYTES : SADDR_BYTES;

	LOG_DBG("%s(): cmd 0x%x Addr 0x%x len 0x%x - ext %d, num_erases %d(0x%x)\n", __func__,
		command, address, len, is_xaddr, num_erases, CONFIG_DW_ERASE_SECTION_SIZE);
	for (i = 0; i < num_erases; i++) {
		uint32_t erase_addr;

		/* Erase start address */
		erase_addr = address + (i * CONFIG_DW_ERASE_SECTION_SIZE);

		ret = dw_spi_send_cmd(dev, command, erase_addr, addr_bytes);
		if (ret)
			break;

		ret = dw_spi_proc_completed(dev, CONFIG_DW_SPI_DRAIN_TIMEOUT_MS);
		if (ret)
			break;
	}

	command = QSPI_SPANSION_CMD_WRITE_DISABLE;
	dw_spi_transfer(&command, sizeof(command), NULL, 0);

	/* exit extended addr mode */
	if (is_xaddr)
		disable_xaddr(dev);

	return ret;
}

static int32_t qspi_chip_erase(const struct device *dev)
{
	int ret = 0;
	uint32_t num_sect;
	uint8_t command;
	struct flash_dw_qspi_data *dd = (struct flash_dw_qspi_data *)dev->data;

	/* First Send write enable latch command */
	command = QSPI_SPANSION_CMD_WRITE_ENABLE;
	ret = dw_spi_transfer(&command, sizeof(command), NULL, 0);
	if (ret)
		goto done;

	/* Send Chip erase command */
	command = QSPI_SPANSION_CMD_CHIP_ERASE;
	num_sect = dd->flash_size / CONFIG_DW_ERASE_SECTION_SIZE;
	ret = dw_spi_transfer(&command, sizeof(command), NULL, 0);
	if (ret)
		goto done;

	ret = dw_spi_proc_completed(dev, num_sect * CONFIG_DW_SPI_DRAIN_TIMEOUT_MS);
	if (ret)
		LOG_ERR("%s(): Timeout waiting for flash processing\n", __func__);

	/* Send write disable command */
	command = QSPI_SPANSION_CMD_WRITE_DISABLE;
	dw_spi_transfer(&command, sizeof(command), NULL, 0);

done:
	return ret;
}

static int32_t flash_dw_qspi_erase(const struct device *dev, off_t address, size_t len)
{
	int ret = 0;
	uint32_t mask;
	uint32_t sizes[MAX_SUB_ADDR];
	uint32_t addr_list[MAX_SUB_ADDR];
	uint32_t num, i;
	struct flash_dw_qspi_data *dd = (struct flash_dw_qspi_data *)dev->data;

	if (len == 0) {
		LOG_ERR("Invalid len %zu for erase\n", len);
		ret = -EPERM;
		goto done;
	}

	if (len == dd->flash_size) {
		ret = qspi_chip_erase(dev);
		goto done;
	} else if ((address + len) > dd->flash_size) {
		ret = -ERANGE;
		goto done;
	}

	mask = CONFIG_DW_ERASE_SECTION_SIZE - 1;
	if ((address & mask) || (len & mask)) {
		LOG_ERR("Addr 0x%x or len 0x%x alignment issue\n", (uint32_t)address,
			(uint32_t)len);
		ret = -EPERM;
		goto done;
	}

	/* Check if the range crosses 16M boundary */
	num = qspi_get_addr_list(address, len, addr_list, sizes);
	for (i = 0; i < num; i++) {
		ret = qspi_pwise_flash_erase(dev, addr_list[i], sizes[i]);
		if (ret)
			goto done;
	}

done:
	return ret;
}

static int xip_pwise_read_data(const struct device *dev, uint32_t address, uint32_t size,
			       uint8_t *data)
{
	int ret = 0;
	uint8_t *phys_addr;
	uint32_t i, num_addr_bits;
	uint8_t is_xaddr = IS_EXT_ADDR(address);

	if (is_xaddr) {
		num_addr_bits = QSPI_SPANSION_CMD_NUM_ADDR_BITS;
		if (num_addr_bits != 32) {
			LOG_ERR("Requesting 32 bit on a flash that supports only %d\n",
				num_addr_bits);
			ret = -EIO;
			goto done;
		}
		enable_xaddr(dev);
	} else {
		num_addr_bits = 24;
		disable_xaddr(dev);
	}

	phys_addr = (uint8_t *)((address & GET_MASK(num_addr_bits)) + XIP_MMAPED_START);

	LOG_DBG("%s(): address 0x%x(m:0x%lx) phys_addr %p, four_byte %d\n", __func__, address,
		GET_MASK(num_addr_bits), phys_addr, is_xaddr);

	/* do a tight loop cpy here in case no memcpy lib */
	for (i = 0; i < size; i++)
		*data++ = *phys_addr++;

	/* Exit 4 byte mode */
	disable_xaddr(dev);

done:
	return ret;
}

static int xip_read_data(const struct device *dev, uint32_t addr, uint32_t len, uint8_t *data)
{
	int ret = 0;
	int num, i;
	uint32_t sizes[MAX_SUB_ADDR];
	uint32_t addr_list[MAX_SUB_ADDR];

	/* Check if address crosses 16M boundary */
	num = qspi_get_addr_list(addr, len, addr_list, sizes);

	for (i = 0; i < num; i++) {
		ret = xip_pwise_read_data(dev, addr_list[i], sizes[i], data);
		if (ret)
			break;

		data += sizes[i];
	}

	return ret;
}

static int32_t flash_dw_xip_qspi_read(const struct device *dev, off_t address, void *data,
				      size_t len)
{
	int ret = 0;
	struct flash_dw_qspi_data *dd = (struct flash_dw_qspi_data *)dev->data;

	if ((len == 0) || (data == NULL)) {
		LOG_ERR("Invalid data len or data ptr\n");
		ret = -EPERM;
		goto done;
	}

	/* Destination buffer cannot be in flash */
	if (((uintptr_t)data >= XIP_MMAPED_START) &&
	    ((uintptr_t)data < (XIP_MMAPED_START + dd->flash_size))) {
		ret = -ERANGE;
		goto done;
	}

	if ((address + len) > dd->flash_size) {
		ret = -ERANGE;
		goto done;
	}

	ret = xip_read_data(dev, address, len, data);

done:
	return ret;
}

int dw_spi_write_data(const struct device *dev, uint32_t addr, uint32_t len, const uint8_t *data)
{
	int ret = 0;
	uint32_t write_size, remaining, offset;
	uint8_t command;
	uint8_t is_xaddr = IS_EXT_ADDR(addr);

	if (!dw_spi_idle()) {
		ret = -EPERM;
		goto done;
	}

	/* Check if we need to enter 4 byte mode */
	(is_xaddr) ? enable_xaddr(dev) : disable_xaddr(dev);

	/* Write bytes upto the next 256 byte aligned address */
	remaining = len;
	write_size = CONFIG_FLASH_PAGE_SIZE - (addr & (CONFIG_FLASH_PAGE_SIZE - 1));
	write_size = MIN(remaining, write_size);
	offset = 0;
	do {
		const uint8_t *next = data + offset;
		ret = dw_spi_aligned_write(dev, QSPI_SPANSION_CMD_WRITE, addr + offset,
					   is_xaddr ? ADDR_L32 : ADDR_L24, next, write_size);
		if (ret)
			break;

		remaining -= write_size;
		offset += write_size;
		write_size = MIN(remaining, CONFIG_FLASH_PAGE_SIZE);
	} while (remaining);

	/* Write Disable */
	command = QSPI_SPANSION_CMD_WRITE_DISABLE;
	dw_spi_transfer(&command, sizeof(command), NULL, 0);

	/* Exit 4 byte mode */
	if (is_xaddr)
		disable_xaddr(dev);

done:
	return ret;
}

static int32_t qspi_write(const struct device *dev, off_t address, const uint8_t *data, size_t len)
{
	int32_t ret = 0;
	uint32_t addr_list[MAX_SUB_ADDR], sizes[MAX_SUB_ADDR];
	int i, num;

	num = qspi_get_addr_list(address, len, addr_list, sizes);
	for (i = 0; i < num; i++) {
		ret = dw_spi_write_data(dev, addr_list[i], sizes[i], data);
		if (ret)
			goto done;

		data += sizes[i];
	}
done:
	return ret;
}

static int32_t flash_dw_qspi_write(const struct device *dev, off_t address, const void *data,
				   size_t len)
{
	int ret = 0;
	struct flash_dw_qspi_data *dd = (struct flash_dw_qspi_data *)dev->data;

	if ((len == 0) || (!data)) {
		LOG_ERR("Invalid data len %ld or data\n", len);
		ret = -EPERM;
		goto done;
	}

	/* Source buffer cannot be in flash */
	if (((uintptr_t)data >= XIP_MMAPED_START) &&
	    ((uintptr_t)data < (XIP_MMAPED_START + dd->flash_size))) {
		ret = -ERANGE;
		goto done;
	}

	if ((address + len) > dd->flash_size) {
		LOG_ERR("Addr(0x%x) + len(0x%x) > than flash_size(0x%x)\n", (uint32_t)address,
			(uint32_t)len, (uint32_t)dd->flash_size);
		ret = -EPERM;
		goto done;
	}
	ret = qspi_write(dev, address, data, len);
done:
	return ret;
}

static const struct flash_parameters *flash_dw_qspi_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	static const struct flash_parameters qspi_flash_parameters = {
		.write_block_size = MAX_BYTE_PER_TRANS,
		// not used
		.erase_value = 0,
	};

	return &qspi_flash_parameters;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)

static struct flash_pages_layout dev_layout = {
	// page count set in init
	.pages_count = 0,
	.pages_size = CONFIG_FLASH_PAGE_SIZE,
};

static void flash_dw_qspi_pages_layout(const struct device *dev,
				       const struct flash_pages_layout **layout,
				       size_t *layout_size)
{
	*layout = &dev_layout;
	*layout_size = 1;
}

#endif

static int32_t flash_dw_qspi_sfdp_read(const struct device *dev, off_t offset, void *data,
				       size_t len)
{
	return -ENOSYS;
}

static const struct flash_driver_api flash_dw_qspi_api = {
	.read = flash_dw_xip_qspi_read,
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

static int32_t flash_dw_qspi_init(const struct device *dev)
{
	printk("Loading Designware QSPI and Spansion Flash\n");

	uint8_t command, resp;
	int ret = ENOSYS;
	struct flash_dw_qspi_config *cfg = (struct flash_dw_qspi_config *)dev->config;
	struct flash_dw_qspi_data *dd = (struct flash_dw_qspi_data *)dev->data;

	reg32clrbits(IO_PAD_CONTROL, 1 << CRM_LS_FORCE_OEB_TO_INPUT);

	// TODO not sure if this is needed only called in flash enumerate function
	// dw_spi_common_init(dev);

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

	ret = dw_spi_send_cmd(dev, QSPI_SPANSION_CMD_WRR, QSPI_SPANSION_WRR_QUAD_MODE_CMD,
			      2 /* cfg + stat reg */);
	if (ret)
		goto done;

	/* read back cfg bits and verify */
	command = QSPI_SPANSION_CMD_RDCR;
	ret = dw_spi_transfer(&command, sizeof(command), &resp, sizeof(resp));
	if (ret)
		goto done;

	dd->flash_size = get_flash_size(dev);
	if (!dd->flash_size) {
		LOG_ERR("Flash size not found");
		ret = -EINVAL;
		goto done;
	}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	dev_layout.pages_count = dd->flash_size / dev_layout.pages_size;
	if (dd->flash_size % dev_layout.pages_size) {
		LOG_ERR("Flash size is not a mutliple of page size.");
		ret = -EINVAL;
		goto done;
	}
#endif

	LOG_INF("Flash QUAD mode set completed - CR 0x%x\n", resp);

	if (!(resp & (1 << QSPI_SPANSION_CFG_QUAD_BIT_POS)))
		ret = -EPERM;
done:
	return ret;
}

#define DEFINE_DW_QSPI(inst)                                                                       \
	static struct flash_dw_qspi_data flash_dw_qspi_data_##inst = {};                           \
	static const struct flash_dw_qspi_config flash_dw_qspi_config_##inst = {                   \
		.freq = DT_INST_PROP(inst, clock_frequency),                                       \
		.ahb_freq = DT_INST_PROP(inst, ahb_bus_frequency),                                 \
		.pp_lanes = DT_INST_PROP(inst, pp_lanes),                                          \
	};                                                                                         \
	BUILD_ASSERT(DT_INST_PROP(inst, clock_frequency) != 0);                                    \
	BUILD_ASSERT(DT_INST_PROP(inst, ahb_bus_frequency) != 0);                                  \
	BUILD_ASSERT(DT_INST_PROP(inst, pp_lanes) != 0);                                           \
	DEVICE_DT_INST_DEFINE(inst, flash_dw_qspi_init, NULL, &flash_dw_qspi_data_##inst,          \
			      &flash_dw_qspi_config_##inst, POST_KERNEL,                           \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &flash_dw_qspi_api)

DT_INST_FOREACH_STATUS_OKAY(DEFINE_DW_QSPI);
