/**
* \file  unittest.c
*
* \brief Implementation of unit test cases.
*		
*
* Copyright (c) 2020 Microchip Technology Inc. and its subsidiaries. 
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
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, 
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
*/
/*
* Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
*/
/******************************************************************************
                   Includes section
******************************************************************************/
#include <types.h>
#include <bootStructure.h>
#include <abstractSerializer.h>
#include <hardwareInit.h>
#include <flashLoader.h>
#ifdef SAM0
#include <app_info.h>
#include <boot_version.h>
#endif
#include <suite.h>

#include <extMemReader.h>
#include <spiMemInterface.h>


/******************************************************************************
                   defines section
******************************************************************************/
#define TEST_NVM_ADDRESS 0x2000
#define INT_MEM_BLOCK 1024  
#ifdef EXT_MEMORY
#define EXT_MEM_ADDRESS_START 0x2000
#define EXT_MEM_ADDRESS_END 0x4000
#define EXT_MEM_BLOCK PAGE_SIZE
#endif

/**
 * \brief Generates byte pattern 1 for a given address
 */
#define BYTE_PATTERN1(x) ((uint8_t)(((x) * 5 + 1) & 0xff))
#define PATTERN_AA  0xAA
#define PATTERN_55  0x55


/******************************************************************************
                   Implementations section
******************************************************************************/
static void nvm_read_and_write_test(const struct test_case *test);
#ifdef EXT_MEMORY
static void int2ext_mem_copy (const struct test_case *test);
static void ext_mem_pattern_write (const struct test_case *test);
static void ext_mem_setup (const struct test_case *test); 
static void ext_mem_clr (const struct test_case *test); 
#endif

/**************************************************************************//**
\brief State machine in the main
******************************************************************************/


int main(void)
{
  
  lowLevelInit();
//  hwStopWdt();
 
#if USE_USART0
  hwInitUsart0();
#endif
  
#if USE_USART1
//  hwInitUsart1();
#endif
  
  
  // Internal flash tests
  DEFINE_TEST_CASE(nvminittest,
                   NULL,
                   nvm_read_and_write_test,
                   NULL,
                   "Testing nvm");
  DEFINE_TEST_ARRAY(nvm_tests) = {
		&nvminittest,
		
	};
  DEFINE_TEST_SUITE(nvm_test_suite, nvm_tests,
			"SAM Flashloader test suite");
  test_suite_run(&nvm_test_suite);
  
 
  // External flash tests
#ifdef EXT_MEMORY  
  DEFINE_TEST_CASE(int2extCopyTest,
                   ext_mem_setup,
                   int2ext_mem_copy,
                   ext_mem_clr,
                   "Testing Int2Ext memcpy");
 
  
  DEFINE_TEST_CASE(extMemPatternTest,
                   ext_mem_setup,
                   ext_mem_pattern_write,
                   ext_mem_clr,
                   "Testing mem pattern writes");
  
  
  DEFINE_TEST_ARRAY(ext_mem_tests) = {
    //
    // Enabling only one test due to buffer allocation issues (space limitation) 
    //
     &extMemPatternTest
		// &int2extCopyTest
	};
  
  DEFINE_TEST_SUITE(ext_mem_test_suite, ext_mem_tests,
			"SAM external mem test suite");
  
  test_suite_run(&ext_mem_test_suite);
#endif 
  
  return 1;
}


static void nvm_read_and_write_test(const struct test_case *test)
{

  uint32_t nvm_addr= TEST_NVM_ADDRESS;  
  uint8_t buffer[INT_MEM_BLOCK];
  unsigned int i;
  
  
  /* Fill the buffer with Pattern 1 */
   for (i = 0; i < INT_MEM_BLOCK; i++) {
     buffer[i] = BYTE_PATTERN1(i);
   }
   
  halFlashWrite(TEST_NVM_ADDRESS, buffer, INT_MEM_BLOCK);
  /* Flush the buffer */
  for (i = 0; i < INT_MEM_BLOCK; i++) {
    buffer[i] = 0;
  }
  memcpy(buffer,(uint32_t *)nvm_addr, INT_MEM_BLOCK);
  /* Check the integrity of data in NVM */
  for (i = 0; i < INT_MEM_BLOCK; i++) {
	   test_assert_true(test, buffer[i] == BYTE_PATTERN1(i),
		"Value not expected @ byte %d (read: 0x%02x, expected: 0x%02x)", 
      i, buffer[i], BYTE_PATTERN1(i));
	}
}

#ifdef EXT_MEMORY
static void int2ext_mem_copy (const struct test_case *test) 
{
  
  uint8_t   buf[INT_MEM_BLOCK];
  bool      status = true;
  uint32_t  destOffset;
  uint8_t*  srcPtr;
  bool      passed = true;
    
 
  // check availability of the external flash
  if (!memCheckMem()) {
    LOG_STRING(MemCheckStr,"\r\n No ext mem found...");
    appSnprintf(MemCheckStr);
    
    test_assert_true(test, 0, "\r\n");
  }
  
  
  destOffset = EXT_MEM_ADDRESS_START;
  srcPtr = (uint8_t*) TEST_NVM_ADDRESS;
  
//  memcpy(srcBuf, srcPtr, EXT_MEM_BLOCK);
  
  // Erase check
#if EXTERNAL_MEMORY != AT45DB041  
  memPreEraseImageArea (destOffset, INT_MEM_BLOCK);
  memReadData (destOffset, buf, INT_MEM_BLOCK);
  /* Check the integrity of data in NVM */
  for (uint32_t i = 0; i < INT_MEM_BLOCK; i++) {    
    if (buf[i] != 0xFF) {
       status = false;
       break;
    }
	}
#endif  
 
   
  if (!status) {
    LOG_STRING(EraseStr,"\n\r Unable to erase...");
    appSnprintf(EraseStr);
    
    test_assert_true(test, 0, "\n\r");
  }
  else {
    LOG_STRING(EraseStr,"\n\r Erase check ok...");
    appSnprintf(EraseStr);
  }
  
                     
//  memWriteData (destOffset, srcPtr, INT_MEM_BLOCK);

  for (destOffset = EXT_MEM_ADDRESS_START; destOffset < EXT_MEM_ADDRESS_END; destOffset+=INT_MEM_BLOCK, srcPtr+=INT_MEM_BLOCK) {
    memWriteData (destOffset, srcPtr, INT_MEM_BLOCK);
    memReadData (destOffset, buf, INT_MEM_BLOCK);
  
    status = !memcmp (buf, srcPtr, INT_MEM_BLOCK);
      
    if (!status) {
      LOG_STRING(McopyStr,"\r\n Copied data not matching source...at address: %X + 0-64");
      appSnprintf(McopyStr, destOffset);
      passed = false;
      
    } 
  }  
  
  if (!passed) {      
      test_assert_true(test, 0, "\r\n");
  } 
          
}  

static void ext_mem_pattern_write (const struct test_case *test)
{
  
  uint8_t buffer[EXT_MEM_BLOCK];
  unsigned int i;
  bool      status = true;
   
   
  // check availability of the external flash     
  if (!memCheckMem()) {
    LOG_STRING(MemCheckStr,"\r\n No ext mem found...");
    appSnprintf(MemCheckStr);
    
    test_assert_true(test, 0, "\r\n");
  }
  
#if EXTERNAL_MEMORY != AT45DB041   
   // Check erase
  memPreEraseImageArea (EXT_MEM_ADDRESS_START, EXT_MEM_BLOCK);
  memReadData (EXT_MEM_ADDRESS_START, buffer, sizeof(buffer));
  /* Check the integrity of data in NVM */
  for (i = 0; i < EXT_MEM_BLOCK; i++) {    
    if (buffer[i] != 0xFF) {
       status = false;
       break;
    }
	}
#endif  
  
  if (!status) {
    LOG_STRING(EraseStr,"\n\r Unable to erase...");
    appSnprintf(EraseStr);
    
    test_assert_true(test, 0, "\r\n");
  }
  else {
    LOG_STRING(EraseStr,"\n\r Erase check ok...");
    appSnprintf(EraseStr);
  }
  
  
  
  // Program ext mem 
  
  /* Fill the buffer with Pattern 101010... */
   for (i = 0; i < EXT_MEM_BLOCK; i++) {
     buffer[i] = 0xAA;
   }
  
  memWriteData(EXT_MEM_ADDRESS_START,buffer,EXT_MEM_BLOCK);
  
  /* Flush the buffer */
  for (i = 0; i < EXT_MEM_BLOCK; i++) {
    buffer[i] = 0;
  }
  
  // Read back into the buffer
  memReadData (EXT_MEM_ADDRESS_START, buffer, EXT_MEM_BLOCK);
  
  /* Check the integrity of data in NVM */
  for (i = 0; i < EXT_MEM_BLOCK; i++) {    
    if (buffer[i] != 0xAA) {
       if (status) {
         LOG_STRING(Pat1Str,"\r\n Write Pattern 0x55 failing ...");
         appSnprintf(Pat1Str); 
         status = false;
       }
       LOG_STRING(Pat2Str,"\r\n Value not expected within 0x%x @ offset %d: read: 0x%x, expected: 0x%x...");
       appSnprintf(Pat2Str, EXT_MEM_ADDRESS_START, i, buffer[i], 0x55);
    }
	}
  
  if (!status) {  
    test_assert_true(test, 0, "\r\n");
  } else {
    LOG_STRING(Pat1Str,"\r\n Write Pattern 0xAA ok...");
    appSnprintf(Pat1Str);
  } 
   
  status = true;
  
#if EXTERNAL_MEMORY != AT45DB041     
  // Program ext mem
  memPreEraseImageArea (EXT_MEM_ADDRESS_START, EXT_MEM_BLOCK);
#endif
  
  /* Fill the buffer with Pattern 010101... */
   for (i = 0; i < EXT_MEM_BLOCK; i++) {
     buffer[i] = 0x55;
   }
  
  memWriteData(EXT_MEM_ADDRESS_START,buffer,EXT_MEM_BLOCK);
  
  /* Flush the buffer */
  for (i = 0; i < EXT_MEM_BLOCK; i++) {
    buffer[i] = 0;
  }
  
  // Read back into the buffer
  memReadData (EXT_MEM_ADDRESS_START, buffer, EXT_MEM_BLOCK);
  
  /* Check the integrity of data in NVM */
  for (i = 0; i < EXT_MEM_BLOCK; i++) {    
    if (buffer[i] != 0x55) {
      if (status) {
        LOG_STRING(Pat1Str,"\r\n Write Pattern 0x55 failing ...");
        appSnprintf(Pat1Str); 
        status = false;
      }
      LOG_STRING(Pat2Str,"\r\n Value not expected within 0x%x @ offset %d: read: 0x%x, expected: 0x%x...");
      appSnprintf(Pat2Str, EXT_MEM_ADDRESS_START, i, buffer[i], 0x55);
    }    
	}
   
  if (!status) { 
    test_assert_true(test, 0, "\r\n");
  } else {
    LOG_STRING(Pat1Str,"\r\n Write Pattern 0x55 ok...");
    appSnprintf(Pat1Str);
  }    
}

static void ext_mem_setup (const struct test_case *test)
{
   // init SPI interface
  spiMemInit();
}
  
static void ext_mem_clr (const struct test_case *test)
{
   // clear spi interface
  spiMemUnInit();
}

#endif

// eof bootloader.c
