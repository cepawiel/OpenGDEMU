/*****************************************************************************
 *
 * \file
 *
 * \brief Management of the AT45DBX data flash controller through SPI.
 *
 * This file manages the accesses to the AT45DBX data flash components.
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 ******************************************************************************/
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */


//_____  I N C L U D E S ___________________________________________________

#include "conf_access.h"


#if AT45DBX_MEM == ENABLE

#include "compiler.h"
#include "board.h"
#include "gpio.h"
#include "spi.h"
#include "conf_at45dbx.h"
#include "at45dbx.h"


#if AT45DBX_MEM_CNT > 4
  #error AT45DBX_MEM_CNT must not exceed 4
#endif


//_____ D E F I N I T I O N S ______________________________________________

/*! \name AT45DBX Group A Commands
 */
//! @{
#define AT45DBX_CMDA_RD_PAGE              0xD2        //!< Main Memory Page Read (Serial/8-bit Mode).
#define AT45DBX_CMDA_RD_ARRAY_LEG         0xE8        //!< Continuous Array Read, Legacy Command (Serial/8-bit Mode).
#define AT45DBX_CMDA_RD_ARRAY_LF_SM       0x03        //!< Continuous Array Read, Low-Frequency Mode (Serial Mode).
#define AT45DBX_CMDA_RD_ARRAY_AF_SM       0x0B        //!< Continuous Array Read, Any-Frequency Mode (Serial Mode).
#define AT45DBX_CMDA_RD_SECTOR_PROT_REG   0x32        //!< Read Sector Protection Register (Serial/8-bit Mode).
#define AT45DBX_CMDA_RD_SECTOR_LKDN_REG   0x35        //!< Read Sector Lockdown Register (Serial/8-bit Mode).
#define AT45DBX_CMDA_RD_SECURITY_REG      0x77        //!< Read Security Register (Serial/8-bit Mode).
//! @}

/*! \name AT45DBX Group B Commands
 */
//! @{
#define AT45DBX_CMDB_ER_PAGE              0x81        //!< Page Erase (Serial/8-bit Mode).
#define AT45DBX_CMDB_ER_BLOCK             0x50        //!< Block Erase (Serial/8-bit Mode).
#define AT45DBX_CMDB_ER_SECTOR            0x7C        //!< Sector Erase (Serial/8-bit Mode).
#define AT45DBX_CMDB_ER_CHIP              0xC794809A  //!< Chip Erase (Serial/8-bit Mode).
#define AT45DBX_CMDB_XFR_PAGE_TO_BUF1     0x53        //!< Main Memory Page to Buffer 1 Transfer (Serial/8-bit Mode).
#define AT45DBX_CMDB_XFR_PAGE_TO_BUF2     0x55        //!< Main Memory Page to Buffer 2 Transfer (Serial/8-bit Mode).
#define AT45DBX_CMDB_CMP_PAGE_TO_BUF1     0x60        //!< Main Memory Page to Buffer 1 Compare (Serial/8-bit Mode).
#define AT45DBX_CMDB_CMP_PAGE_TO_BUF2     0x61        //!< Main Memory Page to Buffer 2 Compare (Serial/8-bit Mode).
#define AT45DBX_CMDB_PR_BUF1_TO_PAGE_ER   0x83        //!< Buffer 1 to Main Memory Page Program with Built-in Erase (Serial/8-bit Mode).
#define AT45DBX_CMDB_PR_BUF2_TO_PAGE_ER   0x86        //!< Buffer 2 to Main Memory Page Program with Built-in Erase (Serial/8-bit Mode).
#define AT45DBX_CMDB_PR_BUF1_TO_PAGE      0x88        //!< Buffer 1 to Main Memory Page Program without Built-in Erase (Serial/8-bit Mode).
#define AT45DBX_CMDB_PR_BUF2_TO_PAGE      0x89        //!< Buffer 2 to Main Memory Page Program without Built-in Erase (Serial/8-bit Mode).
#define AT45DBX_CMDB_PR_PAGE_TH_BUF1      0x82        //!< Main Memory Page Program through Buffer 1 (Serial/8-bit Mode).
#define AT45DBX_CMDB_PR_PAGE_TH_BUF2      0x85        //!< Main Memory Page Program through Buffer 2 (Serial/8-bit Mode).
#define AT45DBX_CMDB_RWR_PAGE_TH_BUF1     0x58        //!< Auto Page Rewrite through Buffer 1 (Serial/8-bit Mode).
#define AT45DBX_CMDB_RWR_PAGE_TH_BUF2     0x59        //!< Auto Page Rewrite through Buffer 2 (Serial/8-bit Mode).
//! @}

/*! \name AT45DBX Group C Commands
 */
//! @{
#define AT45DBX_CMDC_RD_BUF1_LF_SM        0xD1        //!< Buffer 1 Read, Low-Frequency Mode (Serial Mode).
#define AT45DBX_CMDC_RD_BUF2_LF_SM        0xD3        //!< Buffer 2 Read, Low-Frequency Mode (Serial Mode).
#define AT45DBX_CMDC_RD_BUF1_AF_SM        0xD4        //!< Buffer 1 Read, Any-Frequency Mode (Serial Mode).
#define AT45DBX_CMDC_RD_BUF2_AF_SM        0xD6        //!< Buffer 2 Read, Any-Frequency Mode (Serial Mode).
#define AT45DBX_CMDC_RD_BUF1_AF_8M        0x54        //!< Buffer 1 Read, Any-Frequency Mode (8-bit Mode).
#define AT45DBX_CMDC_RD_BUF2_AF_8M        0x56        //!< Buffer 2 Read, Any-Frequency Mode (8-bit Mode).
#define AT45DBX_CMDC_WR_BUF1              0x84        //!< Buffer 1 Write (Serial/8-bit Mode).
#define AT45DBX_CMDC_WR_BUF2              0x87        //!< Buffer 2 Write (Serial/8-bit Mode).
#define AT45DBX_CMDC_RD_STATUS_REG        0xD7        //!< Status Register Read (Serial/8-bit Mode).
#define AT45DBX_CMDC_RD_MNFCT_DEV_ID_SM   0x9F        //!< Manufacturer and Device ID Read (Serial Mode).
//! @}

/*! \name AT45DBX Group D Commands
 */
//! @{
#define AT45DBX_CMDD_EN_SECTOR_PROT       0x3D2A7FA9  //!< Enable Sector Protection (Serial/8-bit Mode).
#define AT45DBX_CMDD_DIS_SECTOR_PROT      0x3D2A7F9A  //!< Disable Sector Protection (Serial/8-bit Mode).
#define AT45DBX_CMDD_ER_SECTOR_PROT_REG   0x3D2A7FCF  //!< Erase Sector Protection Register (Serial/8-bit Mode).
#define AT45DBX_CMDD_PR_SECTOR_PROT_REG   0x3D2A7FFC  //!< Program Sector Protection Register (Serial/8-bit Mode).
#define AT45DBX_CMDD_LKDN_SECTOR          0x3D2A7F30  //!< Sector Lockdown (Serial/8-bit Mode).
#define AT45DBX_CMDD_PR_SECURITY_REG      0x9B000000  //!< Program Security Register (Serial/8-bit Mode).
#define AT45DBX_CMDD_PR_CONF_REG          0x3D2A80A6  //!< Program Configuration Register (Serial/8-bit Mode).
#define AT45DBX_CMDD_DEEP_PWR_DN          0xB9        //!< Deep Power-down (Serial/8-bit Mode).
#define AT45DBX_CMDD_RSM_DEEP_PWR_DN      0xAB        //!< Resume from Deep Power-down (Serial/8-bit Mode).
//! @}


/*! \name Bit-Masks and Values for the Status Register
 */
//! @{
#define AT45DBX_MSK_BUSY                  0x80        //!< Busy status bit-mask.
#define AT45DBX_BUSY                      0x00        //!< Busy status value (0x00 when busy, 0x80 when ready).
#define AT45DBX_MSK_DENSITY               0x3C        //!< Device density bit-mask.
//! @}

#if AT45DBX_MEM_SIZE == AT45DBX_2MB

/*! \name AT45DB161 Memories
 */
//! @{
#define AT45DBX_DENSITY                   0x2C        //!< Device density value.
#define AT45DBX_BYTE_ADDR_BITS            10          //!< Address bits for byte position within buffer.
//! @}

#elif AT45DBX_MEM_SIZE == AT45DBX_4MB

/*! \name AT45DB321 Memories
 */
//! @{
#define AT45DBX_DENSITY                   0x34        //!< Device density value.
#define AT45DBX_BYTE_ADDR_BITS            10          //!< Address bits for byte position within buffer.
//! @}

#elif AT45DBX_MEM_SIZE == AT45DBX_8MB

/*! \name AT45DB642D or AT45DB641E Memories
 */
//! @{
#define AT45DBX_DENSITY                   0x3C        //!< Device density value.
#ifdef AT45DB641E
#define AT45DBX_BYTE_ADDR_BITS            9          //!< Address bits for byte position within buffer.
#else
#define AT45DBX_BYTE_ADDR_BITS            11          //!< Address bits for byte position within buffer.
#endif
//! @}

#else
  #error AT45DBX_MEM_SIZE is not defined to a supported value
#endif

//! Address bits for page selection.
#define AT45DBX_PAGE_ADDR_BITS            (AT45DBX_MEM_SIZE - AT45DBX_PAGE_BITS)

//! Number of bits for addresses within pages.
#define AT45DBX_PAGE_BITS                 (AT45DBX_BYTE_ADDR_BITS - 1)

//! Page size in bytes.
#define AT45DBX_PAGE_SIZE                 (1 << AT45DBX_PAGE_BITS)

//! Bit-mask for byte position within buffer in \ref gl_ptr_mem.
#define AT45DBX_MSK_PTR_BYTE              ((1 << AT45DBX_PAGE_BITS) - 1)

//! Bit-mask for page selection in \ref gl_ptr_mem.
#define AT45DBX_MSK_PTR_PAGE              (((1 << AT45DBX_PAGE_ADDR_BITS) - 1) << AT45DBX_PAGE_BITS)

//! Bit-mask for byte position within sector in \ref gl_ptr_mem.
#define AT45DBX_MSK_PTR_SECTOR            ((1 << AT45DBX_SECTOR_BITS) - 1)


/*! \brief Sends a dummy byte through SPI.
 */
#define spi_write_dummy()                 spi_write(AT45DBX_SPI, 0xFF)


//! Boolean indicating whether memory is in busy state.
static bool at45dbx_busy;

//! Memory data pointer.
static U32 gl_ptr_mem;

//! Sector buffer.
static U8 sector_buf[AT45DBX_SECTOR_SIZE];


/*! \name Control Functions
 */
//! @{


bool at45dbx_init(spi_options_t spiOptions, unsigned int pba_hz)
{
  // Setup SPI registers according to spiOptions.
  for (spiOptions.reg = AT45DBX_SPI_FIRST_NPCS;
       spiOptions.reg < AT45DBX_SPI_FIRST_NPCS + AT45DBX_MEM_CNT;
       spiOptions.reg++)
  {
    if (spi_setupChipReg(AT45DBX_SPI, &spiOptions, pba_hz) != SPI_OK) return false;
  }

  // Memory ready.
  at45dbx_busy = false;

  return true;
}


/*! \brief Selects or unselects a DF memory.
 *
 * \param memidx  Memory ID of DF to select or unselect.
 * \param bSelect Boolean indicating whether the DF memory has to be selected.
 */
static void at45dbx_chipselect_df(U8 memidx, bool bSelect)
{
  if (bSelect)
  {
    // Select SPI chip.
    spi_selectChip(AT45DBX_SPI, AT45DBX_SPI_FIRST_NPCS + memidx);
  }
  else
  {
    // Unselect SPI chip.
    spi_unselectChip(AT45DBX_SPI, AT45DBX_SPI_FIRST_NPCS + memidx);
  }
}


bool at45dbx_mem_check(void)
{
  U8 df;
  U16 status = 0;

  // DF memory check.
  for (df = 0; df < AT45DBX_MEM_CNT; df++)
  {
    // Select the DF memory to check.
    at45dbx_chipselect_df(df, true);

    // Send the Status Register Read command.
    spi_write(AT45DBX_SPI, AT45DBX_CMDC_RD_STATUS_REG);

    // Send a dummy byte to read the status register.
    spi_write_dummy();
    spi_read(AT45DBX_SPI, &status);

    // Unselect the checked DF memory.
    at45dbx_chipselect_df(df, false);

    // Unexpected device density value.
    if ((status & AT45DBX_MSK_DENSITY) < AT45DBX_DENSITY) return false;
  }

  return true;
}


/*! \brief Waits until the DF is ready.
 */
static void at45dbx_wait_ready(void)
{
  U16 status;

  // Select the DF memory gl_ptr_mem points to.
  at45dbx_chipselect_df(gl_ptr_mem >> AT45DBX_MEM_SIZE, true);

  // Send the Status Register Read command.
  spi_write(AT45DBX_SPI, AT45DBX_CMDC_RD_STATUS_REG);

  // Read the status register until the DF is ready.
  do
  {
    // Send a dummy byte to read the status register.
    spi_write_dummy();
    spi_read(AT45DBX_SPI, &status);
  } while ((status & AT45DBX_MSK_BUSY) == AT45DBX_BUSY);

  // Unselect the DF memory gl_ptr_mem points to.
  at45dbx_chipselect_df(gl_ptr_mem >> AT45DBX_MEM_SIZE, false);
}


bool at45dbx_read_open(U32 sector)
{
  U32 addr;

  // Set the global memory pointer to a byte address.
  gl_ptr_mem = sector << AT45DBX_SECTOR_BITS; // gl_ptr_mem = sector * AT45DBX_SECTOR_SIZE.

  // If the DF memory is busy, wait until it's ready.
  if (at45dbx_busy) at45dbx_wait_ready();
  at45dbx_busy = false;

  // Select the DF memory gl_ptr_mem points to.
  at45dbx_chipselect_df(gl_ptr_mem >> AT45DBX_MEM_SIZE, true);

  // Initiate a page read at a given sector.

  // Send the Main Memory Page Read command.
  spi_write(AT45DBX_SPI, AT45DBX_CMDA_RD_PAGE);

  // Send the three address bytes, which comprise:
  //  - (24 - (AT45DBX_PAGE_ADDR_BITS + AT45DBX_BYTE_ADDR_BITS)) reserved bits;
  //  - then AT45DBX_PAGE_ADDR_BITS bits specifying the page in main memory to be read;
  //  - then AT45DBX_BYTE_ADDR_BITS bits specifying the starting byte address within that page.
  // NOTE: The bits of gl_ptr_mem above the AT45DBX_MEM_SIZE bits are useless for the local
  // DF addressing. They are used for DF discrimination when there are several DFs.
  addr = (Rd_bitfield(gl_ptr_mem, AT45DBX_MSK_PTR_PAGE) << AT45DBX_BYTE_ADDR_BITS) |
         Rd_bitfield(gl_ptr_mem, AT45DBX_MSK_PTR_BYTE);
  spi_write(AT45DBX_SPI, LSB2W(addr));
  spi_write(AT45DBX_SPI, LSB1W(addr));
  spi_write(AT45DBX_SPI, LSB0W(addr));

  // Send 32 don't care clock cycles to initialize the read operation.
  spi_write_dummy();
  spi_write_dummy();
  spi_write_dummy();
  spi_write_dummy();

  return true;
}


void at45dbx_read_close(void)
{
  // Unselect the DF memory gl_ptr_mem points to.
  at45dbx_chipselect_df(gl_ptr_mem >> AT45DBX_MEM_SIZE, false);

  // Memory ready.
  at45dbx_busy = false;
}


bool at45dbx_write_open(U32 sector)
{
  U32 addr;

  // Set the global memory pointer to a byte address.
  gl_ptr_mem = sector << AT45DBX_SECTOR_BITS; // gl_ptr_mem = sector * AT45DBX_SECTOR_SIZE.

  // If the DF memory is busy, wait until it's ready.
  if (at45dbx_busy) at45dbx_wait_ready();
  at45dbx_busy = false;

#if AT45DBX_PAGE_SIZE > AT45DBX_SECTOR_SIZE
  // Select the DF memory gl_ptr_mem points to.
  at45dbx_chipselect_df(gl_ptr_mem >> AT45DBX_MEM_SIZE, true);

  // Transfer the content of the current page to buffer 1.

  // Send the Main Memory Page to Buffer 1 Transfer command.
  spi_write(AT45DBX_SPI, AT45DBX_CMDB_XFR_PAGE_TO_BUF1);

  // Send the three address bytes, which comprise:
  //  - (24 - (AT45DBX_PAGE_ADDR_BITS + AT45DBX_BYTE_ADDR_BITS)) reserved bits;
  //  - then AT45DBX_PAGE_ADDR_BITS bits specifying the page in main memory to be read;
  //  - then AT45DBX_BYTE_ADDR_BITS don't care bits.
  // NOTE: The bits of gl_ptr_mem above the AT45DBX_MEM_SIZE bits are useless for the local
  // DF addressing. They are used for DF discrimination when there are several DFs.
  addr = Rd_bitfield(gl_ptr_mem, AT45DBX_MSK_PTR_PAGE) << AT45DBX_BYTE_ADDR_BITS;
  spi_write(AT45DBX_SPI, LSB2W(addr));
  spi_write(AT45DBX_SPI, LSB1W(addr));
  spi_write(AT45DBX_SPI, LSB0W(addr));

  // Unselect the DF memory gl_ptr_mem points to.
  at45dbx_chipselect_df(gl_ptr_mem >> AT45DBX_MEM_SIZE, false);

  // Wait for end of page transfer.
  at45dbx_wait_ready();
#endif

  // Select the DF memory gl_ptr_mem points to.
  at45dbx_chipselect_df(gl_ptr_mem >> AT45DBX_MEM_SIZE, true);

  // Initiate a page write at a given sector.

  // Send the Main Memory Page Program through Buffer 1 command.
  spi_write(AT45DBX_SPI, AT45DBX_CMDB_PR_PAGE_TH_BUF1);

  // Send the three address bytes, which comprise:
  //  - (24 - (AT45DBX_PAGE_ADDR_BITS + AT45DBX_BYTE_ADDR_BITS)) reserved bits;
  //  - then AT45DBX_PAGE_ADDR_BITS bits specifying the page in main memory to be written;
  //  - then AT45DBX_BYTE_ADDR_BITS bits specifying the starting byte address within that page.
  // NOTE: The bits of gl_ptr_mem above the AT45DBX_MEM_SIZE bits are useless for the local
  // DF addressing. They are used for DF discrimination when there are several DFs.
  addr = (Rd_bitfield(gl_ptr_mem, AT45DBX_MSK_PTR_PAGE) << AT45DBX_BYTE_ADDR_BITS) |
         Rd_bitfield(gl_ptr_mem, AT45DBX_MSK_PTR_BYTE);
  spi_write(AT45DBX_SPI, LSB2W(addr));
  spi_write(AT45DBX_SPI, LSB1W(addr));
  spi_write(AT45DBX_SPI, LSB0W(addr));

  return true;
}


void at45dbx_write_close(void)
{
  // While end of logical sector not reached, zero-fill remaining memory bytes.
  while (Rd_bitfield(gl_ptr_mem, AT45DBX_MSK_PTR_SECTOR))
  {
    spi_write(AT45DBX_SPI, 0x00);
    gl_ptr_mem++;
  }

  // Unselect the DF memory gl_ptr_mem points to.
  at45dbx_chipselect_df(gl_ptr_mem >> AT45DBX_MEM_SIZE, false);

  // Memory busy.
  at45dbx_busy = true;
}


//! @}


/*! \name Single-Byte Access Functions
 */
//! @{


U8 at45dbx_read_byte(void)
{
  U16 data;

  // Memory busy.
  if (at45dbx_busy)
  {
    // Being here, we know that we previously finished a page read.
    // => We have to access the next page.

    // Memory ready.
    at45dbx_busy = false;

    // Eventually select the next DF and open the next page.
    // NOTE: at45dbx_read_open input parameter is a sector.
    at45dbx_read_open(gl_ptr_mem >> AT45DBX_SECTOR_BITS); // gl_ptr_mem / AT45DBX_SECTOR_SIZE.
  }

  // Send a dummy byte to read the next data byte.
  spi_write_dummy();
  spi_read(AT45DBX_SPI, &data);
  gl_ptr_mem++;

  // If end of page reached,
  if (!Rd_bitfield(gl_ptr_mem, AT45DBX_MSK_PTR_BYTE))
  {
    // unselect the DF memory gl_ptr_mem points to.
    at45dbx_chipselect_df(gl_ptr_mem >> AT45DBX_MEM_SIZE, false);

    // Memory busy.
    at45dbx_busy = true;
  }

  return data;
}


bool at45dbx_write_byte(U8 b)
{
  // Memory busy.
  if (at45dbx_busy)
  {
    // Being here, we know that we previously launched a page programming.
    // => We have to access the next page.

    // Eventually select the next DF and open the next page.
    // NOTE: at45dbx_write_open input parameter is a sector.
    at45dbx_write_open(gl_ptr_mem >> AT45DBX_SECTOR_BITS); // gl_ptr_mem / AT45DBX_SECTOR_SIZE.
  }

  // Write the next data byte.
  spi_write(AT45DBX_SPI, b);
  gl_ptr_mem++;

  // If end of page reached,
  if (!Rd_bitfield(gl_ptr_mem, AT45DBX_MSK_PTR_BYTE))
  {
    // unselect the DF memory gl_ptr_mem points to in order to program the page.
    at45dbx_chipselect_df(gl_ptr_mem >> AT45DBX_MEM_SIZE, false);

    // Memory busy.
    at45dbx_busy = true;
  }

  return true;
}


//! @}


/*! \name Multiple-Sector Access Functions
 */
//! @{


bool at45dbx_read_multiple_sector(U16 nb_sector)
{
  while (nb_sector--)
  {
    // Read the next sector.
    at45dbx_read_sector_2_ram(sector_buf);
    at45dbx_read_multiple_sector_callback(sector_buf);
  }

  return true;
}


bool at45dbx_write_multiple_sector(U16 nb_sector)
{
  while (nb_sector--)
  {
    // Write the next sector.
    at45dbx_write_multiple_sector_callback(sector_buf);
    at45dbx_write_sector_from_ram(sector_buf);
  }

  return true;
}


//! @}


/*! \name Single-Sector Access Functions
 */
//! @{


bool at45dbx_read_sector_2_ram(void *ram)
{
  U8 *_ram = ram;
  U16 i;
  U16 data;

  // Memory busy.
  if (at45dbx_busy)
  {
    // Being here, we know that we previously finished a page read.
    // => We have to access the next page.

    // Memory ready.
    at45dbx_busy = false;

    // Eventually select the next DF and open the next page.
    // NOTE: at45dbx_read_open input parameter is a sector.
    at45dbx_read_open(gl_ptr_mem >> AT45DBX_SECTOR_BITS); // gl_ptr_mem / AT45DBX_SECTOR_SIZE.
  }

  // Read the next sector.
  for (i = AT45DBX_SECTOR_SIZE; i; i--)
  {
    // Send a dummy byte to read the next data byte.
    spi_write_dummy();
    spi_read(AT45DBX_SPI, &data);
    *_ram++ = data;
  }

  // Update the memory pointer.
  gl_ptr_mem += AT45DBX_SECTOR_SIZE;

#if AT45DBX_PAGE_SIZE > AT45DBX_SECTOR_SIZE
  // If end of page reached,
  if (!Rd_bitfield(gl_ptr_mem, AT45DBX_MSK_PTR_BYTE))
#endif
  {
    // unselect the DF memory gl_ptr_mem points to.
    at45dbx_chipselect_df(gl_ptr_mem >> AT45DBX_MEM_SIZE, false);

    // Memory busy.
    at45dbx_busy = true;
  }

  return true;
}


bool at45dbx_write_sector_from_ram(const void *ram)
{
  const U8 *_ram = ram;
  U16 i;

  // Memory busy.
  if (at45dbx_busy)
  {
    // Being here, we know that we previously launched a page programming.
    // => We have to access the next page.

    // Eventually select the next DF and open the next page.
    // NOTE: at45dbx_write_open input parameter is a sector.
    at45dbx_write_open(gl_ptr_mem >> AT45DBX_SECTOR_BITS); // gl_ptr_mem / AT45DBX_SECTOR_SIZE.
  }

  // Write the next sector.
  for (i = AT45DBX_SECTOR_SIZE; i; i--)
  {
    // Write the next data byte.
    spi_write(AT45DBX_SPI, *_ram++);
  }

  // Update the memory pointer.
  gl_ptr_mem += AT45DBX_SECTOR_SIZE;

#if AT45DBX_PAGE_SIZE > AT45DBX_SECTOR_SIZE
  // If end of page reached,
  if (!Rd_bitfield(gl_ptr_mem, AT45DBX_MSK_PTR_BYTE))
#endif
  {
    // unselect the DF memory gl_ptr_mem points to in order to program the page.
    at45dbx_chipselect_df(gl_ptr_mem >> AT45DBX_MEM_SIZE, false);

    // Memory busy.
    at45dbx_busy = true;
  }

  return true;
}


//! @}


#endif  // AT45DBX_MEM == ENABLE
