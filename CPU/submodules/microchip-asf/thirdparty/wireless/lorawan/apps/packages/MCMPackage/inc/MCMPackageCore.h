/*  _        _   _ _ _ _
   / \   ___| |_(_) (_) |_ _   _
  / _ \ / __| __| | | | __| | | |
 / ___ \ (__| |_| | | | |_| |_| |
/_/   \_\___|\__|_|_|_|\__|\__, |
                           |___/
    (C)2017 Actility
License: Revised BSD License, see LICENCE_ACTILITY.TXT file include in the project & LICENCE_SEMTECH.txt
Description: MCM Core Package definitions

*/

/*
* Licence files location: thirdparty/wireless/lorawan/apps/packages/LibFUOTA/sam0/gcc
*/

#ifndef __MCMPACKAGECORE_H
#define __MCMPACKAGECORE_H

#include "lorawan.h"
#include "DMPackageCore.h"

//-----------------------------------------------------------------------------

#define LORAWAN_MCMPACKAGE_PORT     200

#define LORAWAN_MCMPACKAGE_ID       2 //PackageIdentifier uniquely identifies the package. For the “multicast control package” this identifier is 2.
#define MC_PKGVERSION_VALUE         (1) //corresponds to the version of the package specification implemented by the end-device

#define MC_PKGVERSION_REQ           0x00
#define MC_PKGVERSION_REQ_LEN       (1)
#define MC_PKGVERSION_ANS           0x00
#define MC_PKGVERSION_ANS_LEN       (3)

#define MC_GROUP_STATUS_REQ         0x01
#define MC_GROUP_STATUS_REQ_LEN     (2)
#define MC_GROUP_STATUS_ANS         0x01

#define MC_GROUP_SETUP_REQ          0x02
#define MC_GROUP_SETUP_REQ_LEN      (30)
#define MC_GROUP_SETUP_ANS          0x02

#define MC_GROUP_DELETE_REQ         0x03
#define MC_GROUP_DELETE_REQ_LEN     (2)
#define MC_GROUP_DELETE_ANS         0x03

#define MC_CLASSC_SESSION_REQ       0x04
#define MC_CLASSC_SESSION_ANS       0x04

#define MC_CLASSB_SESSION_REQ       0x05
#define MC_CLASSB_SESSION_ANS       0x05

#define MC_GROUP_CTX_MAX            (4)

#define STATUS_ERROR 1
#define STATUS_OK 0

#define MC_GROUP_STATUS_REQ_RFU_MASK                (0xF0) /* 1st byte has RFU bits */
#define MC_GROUP_SETUP_REQ_RFU_MASK                 (0xFC) /* McGroupIDHeader byte has RFU bits */
#define MC_GROUP_DELETE_REQ_RFU_MASK                (0xFC) /* McGroupIDHeader byte has RFU bits */
#define MC_CLASSC_SESSION_REQ_RFU_MASK_1            (0xFC) /* McGroupIDHeader byte has RFU bits */
#define MC_CLASSC_SESSION_REQ_RFU_MASK_2            (0xF0) /* SessionTimeout byte has RFU bits */
#define MC_CLASSB_SESSION_REQ_RFU_MASK              (0xFC) /* McGroupIDHeader byte has RFU bits */
#define  MCM_RFU_BITS_NOT_ZEROS(bitField, rfuMask)  ((bool)(bitField & rfuMask))

/*!
 * Global  McClassCSession parameters
 */

#define MC_CLASSC_SESSION_REQ_LENGTH (11)
#define MC_CLASSC_SESSION_ANS_LENGTH (5)

#define MC_CLASSB_SESSION_REQ_LENGTH (11)
#define MC_CLASSB_SESSION_ANS_LENGTH (5)

//-----------------------------------------------------------------------------

typedef struct __attribute__((packed)) McClassSessionParams_s
{
  	/*!
     * is the identifier of the multicast  group being used.
     */
    uint8_t McGroupIDHeader;

    uint32_t SessionTime;
	
    uint8_t TimeOut;
	
    struct {
      /*!
       * reception frequency 
       */
      uint32_t DLFrequency :24;
      
       /*!
       * datarate of the current class c session
       */
      uint32_t DataRate    :8;
    };
} McClassSessionParams_t;

typedef struct __attribute__((packed)) McGroupSetupParams_s {
  
  union {
    struct {
      uint8_t   McGroupID :2;
      uint8_t   rfu       :6;
    };
    uint8_t   McGroupIDHeader;
  };
  uint32_t  McAddr;
  uint8_t   mckey_e[16];
  uint32_t  minMcFcnt;
  uint32_t  maxMcFcnt;
} McGroupSetupParams_t;

//-----------------------------------------------------------------------------
extern uint64_t session_endtime;
extern uint32_t session_timeout;

//-----------------------------------------------------------------------------

/*
* \brief Function that processes command targeted to Remote Multicast
*        port (Fport: 200). It will also generate the reply and send it.
*
* \param[in] *appdata - Pointer to appCbParams_t indicating a downlink
*/
void MCMPackageCore( appCbParams_t * appdata );

/*
* \brief Initializes the resources required by MCMPackage
*/
void MCMPackageCreateTimer(void);

/*
* \brief Returns the remaining timeout yet to be elapsed by MCMPackage.
*
* \return uint32_t - remaining timeout
*/
uint32_t MCMPackageNextTimeoutDuration(void);

/*
* \brief Return the last group's parameters that set in device
*
* \return Pointer to McGroupSetupParams_t structure
*/
McGroupSetupParams_t* MCM_get_group_pars(void);

/*
* \brief Function to set the GenAppKey to be used by MCMPackage for derivation
*
* \param[in] *genAppKey - Pointer to genAppKey
*/
void MCMPackageSetGenAppKey(uint8_t *genAppKey);

//-----------------------------------------------------------------------------
/*
* \brief Function will perform a switch to the given class. This function
*        SHALL be used by FUOTA package only and user MUST not call it.
*
* \param[in] class_req - target class to switch, A and C are supported
*/
void ClassSwitch( EdClass_t class_req );

#endif
