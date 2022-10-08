/**
 *
 * \file
 *
 * \brief WINC3400 IoT OTA Interface.
 *
 * Copyright (c) 2017-2019 Microchip Technology Inc. and its subsidiaries.
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
 */

/**@defgroup OTAAPI OTA
   @brief
        The WINC supports OTA (Over-The-Air) updates. Using the APIs described in this module,
        it is possible to request an ATWINC15x0 to update its firmware, or safely rollback to
        the previous firmware version.\n There are also APIs to download files and store them in
        the WINC's Flash (supported by ATWINC1510 only), which can be used for Host MCU OTA
        updates or accessing information stored remotely.
    @{
        @defgroup   OTACALLBACKS    Callbacks
        @brief
            Lists the different callbacks that can be used during OTA updates.\n
            Callbacks of type @ref tpfOtaNotifCb and @ref tpfOtaUpdateCb should be passed
            onto @ref m2m_ota_init at system initialization. Other callbacks are provided
            to handle the various steps of Host File Download.

        @defgroup   OTADEFINE       Defines
        @brief
            Specifies the macros and defines used by the OTA APIs.

        @defgroup   OTATYPEDEF      Enumerations and Typedefs
        @brief
            Specifies the enums and Data Structures used by the OTA APIs.

        @defgroup   OTAFUNCTIONS    Functions
        @brief
            Lists the full set of available APIs to manage OTA updates and Host File Downloads.
    @}
*/

#ifndef __M2M_OTA_H__
#define __M2M_OTA_H__

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

#include "common/include/nm_common.h"
#include "driver/include/m2m_types.h"
#include "driver/source/nmdrv.h"

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
MACROS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

/**@addtogroup OTACALLBACKS
 * @{
 */
/*!
@typedef void (*tpfOtaNotifCb) (tstrOtaUpdateInfo *pstrOtaUpdateInfo);

@brief
    A callback to get notification about a potential OTA update.

@param[in] pstrOtaUpdateInfo
    A structure to provide notification payload.

@sa
    tstrOtaUpdateInfo

@warning
    The notification is not supported (Not implemented yet)
*/
typedef void (*tpfOtaNotifCb)(tstrOtaUpdateInfo *pstrOtaUpdateInfo);

/*!
@typedef void (*tpfOtaUpdateCb)(uint8 u8OtaUpdateStatusType, uint8 u8OtaUpdateStatus);

@brief
   A callback to get OTA status update, the callback provides the status type and its status.\n
   The OTA callback provides the download status, the switch to the downloaded firmware status
   and roll-back status.

@param[in] u8OtaUpdateStatusType
    Possible values are listed in @ref tenuOtaUpdateStatusType.

@param[in] u8OtaUpdateStatus
    Possible values are listed as enumerated by @ref tenuOtaUpdateStatus.

@note
    Executes other callbacks passed to the OTA module.

@see
    tenuOtaUpdateStatusType
    tenuOtaUpdateStatus
 */
typedef void (*tpfOtaUpdateCb)(uint8 u8OtaUpdateStatusType, uint8 u8OtaUpdateStatus);
/**@}*/     // OTACALLBACKS

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
FUNCTION PROTOTYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

#ifdef __cplusplus
extern "C" {
#endif

/*!
@ingroup OTAFUNCTIONS
@fn \
    sint8  m2m_ota_init(tpfOtaUpdateCb pfOtaUpdateCb, tpfOtaNotifCb pfOtaNotifCb);

@brief
    Synchronous initialization function for the OTA layer by registering the update callback.\n
    The notification callback is not supported at the current version. Calling this API is a
    MUST for all the OTA API's.

@param[in]  pfOtaUpdateCb
    OTA Update callback function.

@param[in]  pfOtaNotifCb
    OTA Notify callback function.

@return
    The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
 */
NMI_API sint8 m2m_ota_init(tpfOtaUpdateCb pfOtaUpdateCb, tpfOtaNotifCb pfOtaNotifCb);

/*!
@ingroup OTAFUNCTIONS
@fn \
    sint8  m2m_ota_notif_set_url(uint8 *u8Url);

@brief
    Set the OTA notification server URL, the function needs to be called before any check for update.\n
    This functionality is not supported by WINC firmware.

@param[in]  u8Url
    Set the OTA notification server URL, the function needs to be called before any check for update.

@pre
    Prior calling of @ref m2m_ota_init is required.

@warning
    Notification Server is not supported in the current version (function is not implemented).

@see
    m2m_ota_init

@return
    The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
 */
NMI_API sint8  m2m_ota_notif_set_url(uint8 *u8Url);

/*!
@ingroup OTAFUNCTIONS
@fn \
    sint8  m2m_ota_notif_check_for_update(void);

@brief
    Synchronous function to check for the OTA update using the Notification Server URL.\n
    Function is not implemented (not supported at the current version).

@warning
    Function is not implemented (not supported at the current version).

@sa
    m2m_ota_init
    m2m_ota_notif_set_url

@return
    The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
 */
NMI_API sint8  m2m_ota_notif_check_for_update(void);

/*!
@ingroup OTAFUNCTIONS
@fn \
    sint8 m2m_ota_notif_sched(uint32 u32Period);

@brief
    Schedule OTA notification Server check for update request after specific number of days.\n
    Function is not implemented (not supported at the current version).

@param[in]  u32Period
    Period in days

@warning
    Function is not implemented (not supported at the current version).

@sa
    m2m_ota_init
    m2m_ota_notif_check_for_update
    m2m_ota_notif_set_url

@return
    The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
 */
NMI_API sint8 m2m_ota_notif_sched(uint32 u32Period);

/*!
@ingroup OTAFUNCTIONS
@fn \
    sint8 m2m_ota_start_update(unsigned char * pcDownloadUrl);

@brief
    Request OTA start update using the download URL. The OTA module will download the OTA image, ensure integrity of the image
    and update the validity of the image in the control structure. On completion, a callback of type @ref tpfOtaUpdateCb is called
    (callback previously provided via @ref m2m_ota_init). Switching to the updated image additionally requires completion of
    @ref m2m_ota_switch_firmware, followed by a WINC reset.

@param[in]  pcDownloadUrl
    The download firmware URL, according to the application server.

@warning
    Calling this API does not guarantee OTA WINC image update, it depends on the connection with the
    download server and the validity of the image.\n
    Calling this API invalidates any previous valid rollback image, irrespective of the result, but when
    the OTA succeeds, the current image will become the rollback image after @ref m2m_ota_switch_firmware.

@pre
    @ref m2m_ota_init is a prerequisite and must have been called before using @ref m2m_ota_start_update.\n
    Switching to the newly downloaded image requires calling @ref m2m_ota_switch_firmware API.

@sa
    @ref m2m_ota_init
    @ref m2m_ota_switch_firmware
    @ref tpfOtaUpdateCb

@return
    The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
    Note that successful operation in this context means the OTA update request has reached the firmware OTA module.
    It does not indicate whether or not the image update succeeded.

@section OTAExample Example
        This example shows how an OTA image update and switch is carried out.
        It demonstrates use of the following OTA APIs:
            - @ref m2m_ota_init
            - @ref tpfOtaUpdateCb
            - @ref m2m_ota_start_update
            - @ref m2m_ota_switch_firmware
            - @ref m2m_ota_rollback

        It also makes use of @ref m2m_wifi_check_ota_rb in order to inform OTA decisions.
@code
static void OtaUpdateCb(uint8 u8OtaUpdateStatusType, uint8 u8OtaUpdateStatus)
{
    sint8 s8tmp;
    tstrM2mRev strtmp;
    M2M_INFO("%d %d\n", u8OtaUpdateStatusType, u8OtaUpdateStatus);
    switch(u8OtaUpdateStatusType)
    {
    case DL_STATUS:
        if(u8OtaUpdateStatus == OTA_STATUS_SUCCESS)
        {
            M2M_INFO("OTA download succeeded\n");
            s8tmp = m2m_wifi_check_ota_rb();
            if(s8tmp == M2M_ERR_FW_VER_MISMATCH)
            {
                //  In this case the application SHOULD update the host driver before calling
                //  @ref m2m_ota_switch_firmware(). Switching firmware image and resetting without updating host
                //  driver would lead to severely limited functionality (i.e. OTA rollback only).
            }
            else if(s8tmp == M2M_SUCCESS)
            {
                //  In this case the application MAY WANT TO update the host driver before calling
                // @ref m2m_ota_switch_firmware(). Switching firmware image and resetting without
                // updating host driver may lead to suboptimal functionality.
            }
            else
            {
                M2M_INFO("Cannot recognize downloaded image\n");
                //  In this case the application MUST NOT update the host driver if such an update would change the
                //  driver HIF Major field. Firmware switch @ref using m2m_ota_switch_firmware() is blocked.
                break;
            }
            // Switch to the upgraded firmware
            M2M_INFO("Now switching active partition...\n");
            s8tmp = m2m_ota_switch_firmware();
        }
        break;
    case SW_STATUS:
    case RB_STATUS:
        if(u8OtaUpdateStatus == OTA_STATUS_SUCCESS)
        {
            M2M_INFO("Switch/Rollback succeeded\n");

            // Start the host SW upgrade if required, then system reset is required (Reinitialize the driver)

            M2M_INFO("Now resetting the system...\n");
            system_reset();
        }
        break;
    }
}

static void wifi_event_cb(uint8 u8WiFiEvent, void *pvMsg)
{
    // ...
    case M2M_WIFI_REQ_DHCP_CONF:
    {
        // After successful connection, start the OTA upgrade
        m2m_ota_start_update(OTA_URL);
    }
    break;
    default:
    break;
    // ...
}

int main(void)
{
    tstrWifiInitParam   param;
    sint8               s8Ret             = M2M_SUCCESS;
    bool                rollback_required = FALSE;

    // System init, etc should be here...

    m2m_memset((uint8 *)&param, 0, sizeof(param));
    param.pfAppWifiCb = wifi_event_cb;

    // Initialize the WINC Driver
    s8Ret = m2m_wifi_init(&param);
    if(s8Ret == M2M_ERR_FW_VER_MISMATCH)
    {
        M2M_ERR("Firmware version mismatch\n");
        s8Ret = m2m_wifi_check_ota_rb();
        if(s8Ret == M2M_SUCCESS)
        {
            //  In this case the image in the inactive partition has compatible HIF. We will switch/rollback to it
            //  after initializing the OTA module.
            rollback_required = TRUE;
        }
    }
    if(M2M_SUCCESS != s8Ret)
    {
        M2M_ERR("Driver Init Failed <%d>\n", s8Ret);
        while(1);
    }
    // Initialize the OTA module
    m2m_ota_init(OtaUpdateCb, NULL);
    if(rollback_required)
    {
        //  We need to call either @ref m2m_ota_rollback() or @ref m2m_ota_switch_firmware() (functionally equivalent).
        m2m_ota_rollback();
    }
    else
    {
        //  Connect to AP that provides connection to the OTA server
        m2m_wifi_default_connect();
    }
    while(1)
    {
        //  Handle the app state machine plus the WINC event handler
        while(m2m_wifi_handle_events(NULL) != M2M_SUCCESS) {
        }
    }
}
@endcode
 */
NMI_API sint8 m2m_ota_start_update(unsigned char *pcDownloadUrl);

/*!
@ingroup OTAFUNCTIONS
@fn \
    sint8 m2m_ota_rollback(void);

@brief
    Request OTA Roll-back to the old (inactive) WINC image, the WINC firmware will check the validity of the inactive image
    and activate it if valid. On completion, a callback of type @ref tpfOtaUpdateCb is called (application must previously have
    provided the callback via @ref m2m_ota_init). If the callback indicates successful activation, the newly-activated image
    will start running after next system reset.

@warning
    If rollback requires a host driver update in order to maintain HIF compatibility (HIF
    major value change), then it is recommended to update the host driver prior to calling this API.\n
    In the event of system reset with incompatible driver/firmware, compatibility can be
    recovered by calling @ref m2m_ota_rollback or @ref m2m_ota_switch_firmware. See @ref OTAExample.

@sa
    m2m_ota_init
    m2m_ota_start_update

@return
    The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
 */
NMI_API sint8 m2m_ota_rollback(void);

/*!
@ingroup OTAFUNCTIONS
@fn \
    sint8 m2m_ota_abort(void);

@brief
    Request the WINC to abort an OTA in progress.\n
    If no download is in progress, the API will respond with failure.

@sa
    m2m_ota_init
    m2m_ota_start_update

@return
    The function returns @ref M2M_SUCCESS for a successful operation and a negative value otherwise.
 */
NMI_API sint8 m2m_ota_abort(void);

/*!
@ingroup OTAFUNCTIONS
@fn \
    sint8 m2m_ota_switch_firmware(void);

@brief
    Request switch to the updated WINC image. The WINC firmware will check the validity of the
    inactive image and activate it if valid. On completion, a callback of type @ref tpfOtaUpdateCb
    is called (application must previously have provided the callback via @ref m2m_ota_init).
    If the callback indicates successful activation, the newly-activated image will start running
    after next system reset.

@warning
    If switch requires a host driver update in order to maintain HIF compatibility (HIF
    major value change), then it is recommended to update the host driver prior to calling this API.\n
    In the event of system reset with incompatible driver/firmware, compatibility can be
    recovered by calling @ref m2m_ota_rollback or @ref m2m_ota_switch_firmware. See @ref OTAExample.

@sa
    m2m_ota_init
    m2m_ota_start_update

@return
    The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
 */
NMI_API sint8 m2m_ota_switch_firmware(void);

#if 0
NMI_API sint8 m2m_ota_test(void);
#endif

#ifdef __cplusplus
}
#endif
#endif /* __M2M_OTA_H__ */
