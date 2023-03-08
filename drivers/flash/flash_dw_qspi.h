#ifndef DW_QSPI_FLASH_NOR_H
#define DW_QSPI_FLASH_NOR_H

#include <zephyr/sys/util.h>

#define LS_QSPI_FLASH_LS_QSPI_FLASH_DUMMY__ADDRESS 0x40000000

//--------------Register:CRM_CFG_CRM_CHIP_IO_PAD_CONTROL
#define CRM_CFG_CRM_CHIP_IO_PAD_CONTROL__ADDRESS			       0x5200004c
#define CRM_CFG_CRM_CHIP_IO_PAD_CONTROL_DATA__RESERVED_31_1__WIDTH	       31
#define CRM_CFG_CRM_CHIP_IO_PAD_CONTROL_DATA__RESERVED_31_1__LSB	       1
#define CRM_CFG_CRM_CHIP_IO_PAD_CONTROL_DATA__CRM_LS_FORCE_OEB_TO_INPUT__WIDTH 1
#define CRM_CFG_CRM_CHIP_IO_PAD_CONTROL_DATA__CRM_LS_FORCE_OEB_TO_INPUT__LSB   0

/*
 * DW_SSI related registers
 * Only necessary fields are included here, which are
 * from the soc.h
 */
//--------------Register:LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0__ADDRESS			      0x500a0000
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0__ADDRESS_OFFSET		      0x0
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__SSI_IS_MST__WIDTH	      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__SSI_IS_MST__LSB	      31
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__RSVD_CTRLR0_26_31__WIDTH 5
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__RSVD_CTRLR0_26_31__LSB   26
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__SPI_DWS_EN__WIDTH	      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__SPI_DWS_EN__LSB	      25
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__SPI_HYPERBUS_EN__WIDTH   1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__SPI_HYPERBUS_EN__LSB     24
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__SPI_FRF__WIDTH	      2
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__SPI_FRF__LSB	      22
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__RSVD_CTRLR0_20_21__WIDTH 2
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__RSVD_CTRLR0_20_21__LSB   20
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__CFS__WIDTH		      4
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__CFS__LSB		      16
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__RSVD_CTRLR0_15__WIDTH    1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__RSVD_CTRLR0_15__LSB      15
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__SSTE__WIDTH	      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__SSTE__LSB		      14
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__SRL__WIDTH		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__SRL__LSB		      13
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__SLV_OE__WIDTH	      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__SLV_OE__LSB	      12
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__TMOD__WIDTH	      2
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__TMOD__LSB		      10
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__SCPOL__WIDTH	      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__SCPOL__LSB		      9
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__SCPH__WIDTH	      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__SCPH__LSB		      8
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__FRF__WIDTH		      2
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__FRF__LSB		      6
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__RSVD_CTRLR0_5__WIDTH     1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__RSVD_CTRLR0_5__LSB	      5
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__DFS__WIDTH		      5
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR0_DATA__DFS__LSB		      0
//--------------Register:LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR1__ADDRESS			      0x500a0004
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR1__ADDRESS_OFFSET		      0x4
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR1_DATA__RSVD_CTRLR1__WIDTH	      16
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR1_DATA__RSVD_CTRLR1__LSB	      16
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR1_DATA__NDF__WIDTH		      16
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_CTRLR1_DATA__NDF__LSB		      0
//--------------Register:LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SSIENR
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SSIENR__ADDRESS			      0x500a0008
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SSIENR__ADDRESS_OFFSET		      0x8
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SSIENR_DATA__RSVD_SSIENR__WIDTH	      31
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SSIENR_DATA__RSVD_SSIENR__LSB	      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SSIENR_DATA__SSIC_EN__WIDTH	      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SSIENR_DATA__SSIC_EN__LSB	      0
//--------------Register:LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_MWCR
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_MWCR__ADDRESS			      0x500a000c
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_MWCR__ADDRESS_OFFSET		      0xc
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_MWCR_DATA__RSVD_MWCR__WIDTH	      29
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_MWCR_DATA__RSVD_MWCR__LSB	      3
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_MWCR_DATA__MHS__WIDTH		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_MWCR_DATA__MHS__LSB		      2
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_MWCR_DATA__MDD__WIDTH		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_MWCR_DATA__MDD__LSB		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_MWCR_DATA__MWMOD__WIDTH		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_MWCR_DATA__MWMOD__LSB		      0
//--------------Register:LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SER
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SER__ADDRESS			      0x500a0010
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SER__ADDRESS_OFFSET		      0x10
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SER_DATA__RSVD_SER__WIDTH	      31
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SER_DATA__RSVD_SER__LSB		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SER_DATA__SER__WIDTH		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SER_DATA__SER__LSB		      0
//--------------Register:LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_BAUDR
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_BAUDR__ADDRESS			      0x500a0014
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_BAUDR__ADDRESS_OFFSET		      0x14
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_BAUDR_DATA__RSVD_BAUDR_16_31__WIDTH   16
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_BAUDR_DATA__RSVD_BAUDR_16_31__LSB     16
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_BAUDR_DATA__SCKDV__WIDTH	      15
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_BAUDR_DATA__SCKDV__LSB		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_BAUDR_DATA__RSVD_BAUDR_0__WIDTH	      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_BAUDR_DATA__RSVD_BAUDR_0__LSB	      0
//--------------Register:LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_TXFTLR
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_TXFTLR__ADDRESS			      0x500a0018
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_TXFTLR__ADDRESS_OFFSET		      0x18
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_TXFTLR_DATA__RSVD_TXFTHR__WIDTH	      11
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_TXFTLR_DATA__RSVD_TXFTHR__LSB	      21
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_TXFTLR_DATA__TXFTHR__WIDTH	      5
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_TXFTLR_DATA__TXFTHR__LSB	      16
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_TXFTLR_DATA__RSVD_TXFTLR__WIDTH	      11
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_TXFTLR_DATA__RSVD_TXFTLR__LSB	      5
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_TXFTLR_DATA__TFT__WIDTH		      5
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_TXFTLR_DATA__TFT__LSB		      0
//--------------Register:LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_RXFTLR
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_RXFTLR__ADDRESS			      0x500a001c
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_RXFTLR__ADDRESS_OFFSET		      0x1c
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_RXFTLR_DATA__RSVD_RXFTLR__WIDTH	      27
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_RXFTLR_DATA__RSVD_RXFTLR__LSB	      5
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_RXFTLR_DATA__RFT__WIDTH		      5
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_RXFTLR_DATA__RFT__LSB		      0
//--------------Register:LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_TXFLR
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_TXFLR__ADDRESS			      0x500a0020
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_TXFLR__ADDRESS_OFFSET		      0x20
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_TXFLR_DATA__RSVD_TXFLR__WIDTH	      26
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_TXFLR_DATA__RSVD_TXFLR__LSB	      6
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_TXFLR_DATA__TXTFL__WIDTH	      6
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_TXFLR_DATA__TXTFL__LSB		      0
//--------------Register:LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_RXFLR
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_RXFLR__ADDRESS			      0x500a0024
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_RXFLR__ADDRESS_OFFSET		      0x24
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_RXFLR_DATA__RSVD_RXFLR__WIDTH	      26
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_RXFLR_DATA__RSVD_RXFLR__LSB	      6
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_RXFLR_DATA__RXTFL__WIDTH	      6
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_RXFLR_DATA__RXTFL__LSB		      0
//--------------Register:LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SR
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SR__ADDRESS			      0x500a0028
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SR__ADDRESS_OFFSET		      0x28
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SR_DATA__CMPLTD_DF__WIDTH	      17
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SR_DATA__CMPLTD_DF__LSB		      15
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SR_DATA__RSVD_SR__WIDTH		      8
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SR_DATA__RSVD_SR__LSB		      7
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SR_DATA__DCOL__WIDTH		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SR_DATA__DCOL__LSB		      6
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SR_DATA__TXE__WIDTH		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SR_DATA__TXE__LSB		      5
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SR_DATA__RFF__WIDTH		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SR_DATA__RFF__LSB		      4
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SR_DATA__RFNE__WIDTH		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SR_DATA__RFNE__LSB		      3
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SR_DATA__TFE__WIDTH		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SR_DATA__TFE__LSB		      2
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SR_DATA__TFNF__WIDTH		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SR_DATA__TFNF__LSB		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SR_DATA__BUSY__WIDTH		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SR_DATA__BUSY__LSB		      0
//--------------Register:LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR__ADDRESS			      0x500a002c
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR__ADDRESS_OFFSET		      0x2c
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__RSVD_12_32_IMR__WIDTH	      20
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__RSVD_12_32_IMR__LSB	      12
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__DONEM__WIDTH		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__DONEM__LSB		      11
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__SPITEM__WIDTH		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__SPITEM__LSB		      10
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__RSVD_9_IMR__WIDTH	      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__RSVD_9_IMR__LSB	      9
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__AXIEM__WIDTH		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__AXIEM__LSB		      8
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__TXUIM__WIDTH		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__TXUIM__LSB		      7
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__XRXOIM__WIDTH		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__XRXOIM__LSB		      6
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__MSTIM__WIDTH		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__MSTIM__LSB		      5
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__RXFIM__WIDTH		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__RXFIM__LSB		      4
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__RXOIM__WIDTH		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__RXOIM__LSB		      3
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__RXUIM__WIDTH		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__RXUIM__LSB		      2
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__TXOIM__WIDTH		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__TXOIM__LSB		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__TXEIM__WIDTH		      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IMR_DATA__TXEIM__LSB		      0

//--------------Register:LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IDR
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IDR__ADDRESS				       0x500a0058
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IDR__ADDRESS_OFFSET			       0x58
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IDR_DATA__IDCODE__WIDTH			       32
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_IDR_DATA__IDCODE__LSB			       0
//--------------Register:LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SSIC_VERSION_ID
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SSIC_VERSION_ID__ADDRESS		       0x500a005c
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SSIC_VERSION_ID__ADDRESS_OFFSET		       0x5c
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SSIC_VERSION_ID_DATA__SSIC_COMP_VERSION__WIDTH 32
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SSIC_VERSION_ID_DATA__SSIC_COMP_VERSION__LSB   0
//--------------Register:LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_DR0
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_DR0__ADDRESS				       0x500a0060
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_DR0__ADDRESS_OFFSET			       0x60
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_DR0_DATA__DR__WIDTH			       32
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_DR0_DATA__DR__LSB			       0
//--------------Register:LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0__ADDRESS			       0x500a00f4
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0__ADDRESS_OFFSET		       0xf4
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__RSVD_SPI_CTRLR0__WIDTH	       1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__RSVD_SPI_CTRLR0__LSB	       31
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__CLK_STRETCH_EN__WIDTH	       1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__CLK_STRETCH_EN__LSB	       30
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__XIP_PREFETCH_EN__WIDTH	       1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__XIP_PREFETCH_EN__LSB	       29
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__RSVD_SPI_CTRLR0_28__WIDTH     1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__RSVD_SPI_CTRLR0_28__LSB       28
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__XIP_MBL__WIDTH		       2
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__XIP_MBL__LSB		       26
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__SPI_RXDS_SIG_EN__WIDTH	       1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__SPI_RXDS_SIG_EN__LSB	       25
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__SPI_DM_EN__WIDTH	       1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__SPI_DM_EN__LSB		       24
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__RSVD_SPI_CTRLR0_22_23__WIDTH  2
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__RSVD_SPI_CTRLR0_22_23__LSB    22
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__SSIC_XIP_CONT_XFER_EN__WIDTH  1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__SSIC_XIP_CONT_XFER_EN__LSB    21
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__XIP_INST_EN__WIDTH	       1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__XIP_INST_EN__LSB	       20
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__XIP_DFS_HC__WIDTH	       1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__XIP_DFS_HC__LSB	       19
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__SPI_RXDS_EN__WIDTH	       1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__SPI_RXDS_EN__LSB	       18
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__INST_DDR_EN__WIDTH	       1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__INST_DDR_EN__LSB	       17
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__SPI_DDR_EN__WIDTH	       1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__SPI_DDR_EN__LSB	       16
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__WAIT_CYCLES__WIDTH	       5
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__WAIT_CYCLES__LSB	       11
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__RSVD_SPI_CTRLR0_10__WIDTH     1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__RSVD_SPI_CTRLR0_10__LSB       10
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__INST_L__WIDTH		       2
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__INST_L__LSB		       8
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__XIP_MD_BIT_EN__WIDTH	       1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__XIP_MD_BIT_EN__LSB	       7
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__RSVD_SPI_CTRLR0_6__WIDTH      1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__RSVD_SPI_CTRLR0_6__LSB	       6
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__ADDR_L__WIDTH		       4
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__ADDR_L__LSB		       2
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__TRANS_TYPE__WIDTH	       2
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_SPI_CTRLR0_DATA__TRANS_TYPE__LSB	       0

//--------------Register:LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_INCR_INST
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_INCR_INST__ADDRESS			   0x500a0100
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_INCR_INST__ADDRESS_OFFSET		   0x100
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_INCR_INST_DATA__RSVD_INCR_INST__WIDTH 16
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_INCR_INST_DATA__RSVD_INCR_INST__LSB   16
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_INCR_INST_DATA__INCR_INST__WIDTH	   16
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_INCR_INST_DATA__INCR_INST__LSB	   0

//--------------Register:LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL__ADDRESS			  0x500a0108
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL__ADDRESS_OFFSET		  0x108
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__RSVD_XIP_CTRL__WIDTH	  2
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__RSVD_XIP_CTRL__LSB	  30
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__XIP_PREFETCH_EN__WIDTH	  1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__XIP_PREFETCH_EN__LSB	  29
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__RSVD_XIP_CTRL_28__WIDTH	  1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__RSVD_XIP_CTRL_28__LSB	  28
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__XIP_MBL__WIDTH		  2
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__XIP_MBL__LSB		  26
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__RXDS_SIG_EN__WIDTH	  1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__RXDS_SIG_EN__LSB	  25
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__XIP_HYPERBUS_EN__WIDTH	  1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__XIP_HYPERBUS_EN__LSB	  24
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__CONT_XFER_EN__WIDTH	  1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__CONT_XFER_EN__LSB	  23
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__INST_EN__WIDTH		  1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__INST_EN__LSB		  22
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__RXDS_EN__WIDTH		  1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__RXDS_EN__LSB		  21
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__INST_DDR_EN__WIDTH	  1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__INST_DDR_EN__MSB	  20
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__INST_DDR_EN__LSB	  20
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__DDR_EN__WIDTH		  1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__DDR_EN__LSB		  19
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__DFS_HC__WIDTH		  1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__DFS_HC__LSB		  18
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__WAIT_CYCLES__WIDTH	  5
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__WAIT_CYCLES__LSB	  13
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__MD_BITS_EN__WIDTH	  1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__MD_BITS_EN__LSB		  12
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__RSVD_SPI_CTRLR0_11__WIDTH 1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__RSVD_SPI_CTRLR0_11__LSB	  11
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__INST_L__WIDTH		  2
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__INST_L__LSB		  9
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__RSVD_XIP_CTRL_8__WIDTH	  1
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__RSVD_XIP_CTRL_8__LSB	  8
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__ADDR_L__WIDTH		  4
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__ADDR_L__LSB		  4
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__TRANS_TYPE__WIDTH	  2
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__TRANS_TYPE__LSB		  2
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__FRF__WIDTH		  2
#define LS_QSPI_REGS_SSIC_ADDRESS_BLOCK2_XIP_CTRL_DATA__FRF__LSB		  0

#define XIP_MMAPED_START \
	LS_QSPI_FLASH_LS_QSPI_FLASH_DUMMY__ADDRESS

#define FLASH_PAGE_SIZE	   256

#define QSPI_SPANSION_CMD_FAST_READ   0x0b
#define QSPI_SPANSION_CMD_1_1_2_READ  0x3b
#define QSPI_SPANSION_CMD_1_2_2_READ  0xbb
#define QSPI_SPANSION_CMD_1_1_4_READ  0x6b
#define QSPI_SPANSION_CMD_1_4_4_READ  0xeb
#define QSPI_SPANSION_CMD_1_1_2_WRITE 0xa2
#define QSPI_SPANSION_CMD_1_1_4_WRITE 0x32

#define QSPI_SPANSION_CMD_WRITE		 0x32
#define QSPI_SPANSION_CMD_WRR		 0x01
#define QSPI_SPANSION_CMD_CHIP_ERASE	 0xc7
#define QSPI_SPANSION_CMD_WRITE_ENABLE	 0x06
#define QSPI_SPANSION_CMD_WRITE_DISABLE	 0x04
#define QSPI_SPANSION_CMD_READ_STATUS	 0x05
#define QSPI_SPANSION_CMD_READ_EXT_ADDR	 0x16
#define QSPI_SPANSION_CMD_WRITE_EXT_ADDR 0x17
#define QSPI_SPANSION_CMD_SRESET	 0xf0
#define QSPI_SPANSION_CMD_RDCR		 0x35
#define QSPI_SPANSION_CMD_ERASE_256K		0xd8

#define QSPI_SPANSION_1_1_2_READ_DUMMY 8
#define QSPI_SPANSION_1_1_4_READ_DUMMY 8

/* qual mode wrr value, setting QUAD bit in cfg + WENL in status */
#define QSPI_SPANSION_WRR_QUAD_MODE_CMD 0x0202

/* cfg bit quad_mode */
#define QSPI_SPANSION_CFG_QUAD_BIT_POS 1

/* Status bit positions */
#define QSPI_SPANSION_CMD_BUSY_BIT_POS 0

/* Number of address bits in 4 byte addr mode */
#define QSPI_SPANSION_CMD_NUM_ADDR_BITS 32

#define DW_SPI_ENABLE  (1)
#define DW_SPI_DISABLE (0)

#define DW_SPI_NDF_DEFAULT 0x7

#define SIZE_16MB (1024 * 1024 * 16)
#define MBYTE_SZ(_sz)   ((_sz) << 20)
#define IS_EXT_ADDR(_a)    (((_a) >= SIZE_16MB) ? 1 : 0)
#define SADDR_BYTES        (3)
#define XADDR_BYTES        (4)

#define IO_PAD_CONTROL	CRM_CFG_CRM_CHIP_IO_PAD_CONTROL__ADDRESS
#define CRM_LS_FORCE_OEB_TO_INPUT		0

#define QSPI_SPANSION_MFG_ID		0x01
#define QSPI_JEDEC_READ_MFG_ID_COMMAND	0x9F

#define EXT_ADDR_DISABLED 0x00
#define EXT_ADDR_ENABLED 0x80

/*
 * per operation needs to take into consideration of crossing
 * the 16M boundary, and if so happens, the operation has to
 * be divided into 2 sub tasks.
 */
#define MAX_SUB_ADDR				(2)

#define DW_SPI_TX_FIFO_LEN (1 << LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_TXFTLR_DATA__TFT__WIDTH)
#define DW_SPI_RX_FIFO_LEN (1 << LS_QSPI_REGS_SSIC_ADDRESS_BLOCK_RXFTLR_DATA__RFT__WIDTH)

/* 2 below is because 1 word for inst and one for addr */
#define MAX_WORD_PER_TRANS (DW_SPI_TX_FIFO_LEN - 2)
#define MAX_BYTE_PER_TRANS (MAX_WORD_PER_TRANS * sizeof(uint32_t))

/* Default DFS size to have byte */
#define DEFAULT_DFS_BIT_SZ	  (8)
/* Data transfer/write dfs size */
#define DEFAULT_DATA_TRANS_BIT_SZ (32)

#define GET_MASK(n) ((0x1UL << (n)) - 1)

/* Macro for swapping the byte order of a 32 bits unsigned integer */
#define BSWAP_32(x)                                                                                \
	((uint32_t)((((x) >> 24) & 0xff) | (((x) >> 8) & 0xff00) | (((x)&0xff00) << 8) |           \
		    (((x)&0xff) << 24)))

#define QSPI_CLOCK_POL(_cfg)   ((_cfg)->clk_pol)
#define QSPI_CLOCK_PHASE(_cfg) ((_cfg)->clk_pha)
#define QSPI_SPI_FRF(_cfg)     ((_cfg)->spi_frf)

typedef enum {
	SPI_PROT = 0x0,	      /**< Motoral SPI format */
	SSP_PROT = 0x1,	      /**< TI SSP format - not used */
	MICROWIRE_PROT = 0x2, /**< National microwire - not used */
} FRF;

/* SPI_FRAME_FORMAT - data lane usage, SPI_FRAME_FORMAT */
typedef enum {
	SPI_STD = 0x0,	 /**< Standard SPI operation */
	SPI_DUAL = 0x1,	 /**< Dual SPI operation */
	SPI_QUAD = 0x2,	 /**< Quad SPI operation */
	SPI_OCTAL = 0x3, /**< Octal - reserved for future */
} SPI_FRF;

/*
 * TRANSFER_TYPE decides whether inst and addr will use the
 * specified SPI_FRF.
 * Note: this only applies to indirect write, while XIP will
 *       always send both inst/addr in std format as recommended
 *       by ASIC.
 */
typedef enum {
	TT0 = 0x0, /**< inst & addr sent in std format */
	TT1 = 0x1, /**< inst in std formt, addr in SPI_FRF */
	TT2 = 0x2, /**< inst & addr in SPI_FRF */
} TRANS_TYPE;

/*
 * ADDRESS_LENGTH - only define needed ones
 */
typedef enum {
	ADDR_L0 = 0x0,	/**< no address */
	ADDR_L8 = 0x2,	/**< 8 bit address */
	ADDR_L16 = 0x4, /**< 16 bit address */
	ADDR_L24 = 0x6, /**< 24 bit address */
	ADDR_L32 = 0x8, /**< 32 bit address */
} ADDR_L;

/*
 * INSTRUCTION length
 */
typedef enum {
	INST_L0 = 0x0, /**< no inst */
	INST_L8 = 0x2, /**< 8 bit instruction length */
} INST_L;

/*
 * Transfer mode
 */
typedef enum {
	TX_AND_RX = 0x0,
	TX_ONLY = 0x1,
	RX_ONLY = 0x2,
	EEPROM_READ = 0x3,
} TMOD;

enum qspi_ftype {
    FTYPE_SPANSION = 0,
	FTYPE_MAX
};

enum qspi_trans_mode {
	QSPI_TRANS_MODE_1_1_1 = 0, /* (std, std, std)   */
	QSPI_TRANS_MODE_1_1_2,	   /* (std, std, dual)  */
	QSPI_TRANS_MODE_1_2_2,	   /* (std, dual, dual) */
	QSPI_TRANS_MODE_2_2_2,	   /* (dual, dual, dual)*/
	QSPI_TRANS_MODE_1_1_4,	   /* (std, std, quad)  */
	QSPI_TRANS_MODE_1_4_4,	   /* (std, quad, quad) */
	QSPI_TRANS_MODE_4_4_4,	   /* (quad, quad, quad */
	QSPI_TRANS_MODE_MAX,
};

struct qspi_config {
	uint32_t freq;
	uint8_t clk_pol; /* clock polarity */
	uint8_t clk_pha; /* clock phase */
	uint8_t spi_frf; /* spi frame format */
};

struct qspi_flash_info {
	char *name;
	uint32_t flash_size;
	uint32_t erase_sect_size;
};

struct qspi_rdid_resp {
	uint8_t mgr_id;
	uint8_t dev_id;
	uint8_t cap;
};

#endif
