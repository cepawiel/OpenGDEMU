/*  _        _   _ _ _ _
   / \   ___| |_(_) (_) |_ _   _
  / _ \ / __| __| | | | __| | | |
 / ___ \ (__| |_| | | | |_| |_| |
/_/   \_\___|\__|_|_|_|\__|\__, |
                           |___/
    (C)2017 Actility
License: Revised BSD License, see LICENCE_ACTILITY.TXT file include in the project & LICENCE_SEMTECH.txt
Description: Device Management Package Support

*/

/*
* Licence files location: thirdparty/wireless/lorawan/apps/packages/LibFUOTA/sam0/gcc
*/

#ifndef __DMPACKAGECORE_H__
#define __DMPACKAGECORE_H__

#include "lorawan.h"
#include "msg.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LORAWAN_DMPACKAGE_PORT          (203)
#define LORAWAN_DMPACKAGE_INDEX         (0)

#define DEV_VERSION_REQ                 (0x01)  /**< Asks an end-device its hardware revision, currently running firmware version */
#define DEV_VERSION_ANS                 (0x01)  /**< Conveys answer to the DevVersionReq request */
  
#define DEV_VERSION_REQ_LENGTH          (1)
#define DEV_VERSION_ANS_LENGTH          (9)

//--------------------------------------

/*
* \brief Sets the HW and FW version information to be used by
*        DMPackage. This will be used to reply for DEV_VERSION_REQ
*
* \param[in] fwVersion - firmware version
* \param[in] hwVersion - hardware version
*
* \return None
*/
void DMPackage_SetVersionInfo(uint32_t fwVersion, uint32_t hwVersion);

/*
* \brief Function that processes a single command targeted to Device Management
*        port (Fport: 203). It will also generate the reply and send it.
*
* \param[in] *appdata - Pointer to appCbParams_t indicating a downlink
*/
void DMPackageCore(appCbParams_t *appdata);

#ifdef __cplusplus
}
#endif

#endif /* __DMPACKAGECORE_H__ */
