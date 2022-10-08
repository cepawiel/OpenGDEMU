/*  _        _   _ _ _ _
   / \   ___| |_(_) (_) |_ _   _
  / _ \ / __| __| | | | __| | | |
 / ___ \ (__| |_| | | | |_| |_| |
/_/   \_\___|\__|_|_|_|\__|\__, |
                           |___/
    (C)2017 Actility
License: Revised BSD License, see LICENCE_ACTILITY.TXT file include in the project & LICENCE_SEMTECH.txt
Description: FTM Package Core definitions

*/

/*
* Licence files location: thirdparty/wireless/lorawan/apps/packages/LibFUOTA/sam0/gcc
*/

#ifndef __FTMPACKAGECORE_H__
#define __FTMPACKAGECORE_H__

#include "lorawan.h"

#define LORAWAN_FTMPACKAGE_PORT     201

#define LORAWAN_FTMPACKAGE_ID       3 //uniquely identifies the package. For the “fragmentation transport package” this identifier is 3.
#define FRAG_PKGVERSION_VALUE       (1)

#define FRAG_PKGVERSION_REQ         0x00
#define FRAG_PKGVERSION_REQ_LEN     (1)
#define FRAG_PKGVERSION_ANS         0x00
#define FRAG_PKGVERSION_ANS_LEN     (3)

#define FRAG_SESSION_STATUS_REQ     0x01
#define FRAG_SESSION_STATUS_REQ_LEN (2)
#define FRAG_SESSION_STATUS_ANS     0x01
#define FRAG_SESSION_STATUS_ANS_LEN (5)

#define FRAG_SESSION_SETUP_REQ      0x02
#define FRAG_SESSION_SETUP_REQ_LEN  (11)
#define FRAG_SESSION_SETUP_ANS      0x02
#define FRAG_SESSION_SETUP_ANS_LEN  (2)

#define FRAG_SESSION_DELETE_REQ     0x03
#define FRAG_SESSION_DELETE_REQ_LEN 2
#define FRAG_SESSION_DELETE_ANS     0x03
#define FRAG_SESSION_DELETE_ANS_LEN 2

#define FRAG_DATA                   0x08

#define FRAG_SESSION_ENCUNSUPPORTED   0x01
#define FRAG_SESSION_NOTENOUGHMEMORY  0x02
#define FRAG_SESSION_ALREADYEXIST     0x10
#define FRAG_SESSION_INDEXUSUPPORTED  0x04
#define FRAG_SESSION_DESCUNSUPPORTED  0x08



#define FRAG_SESSION_SETUP_REQ_PARAM_IDX       (2)

#define FRAG_STATUS_REQ_PARAM_IDX              (2)
#define FRAG_STATUS_REQ_PARAM_FRAGINDEX(x)     (((x) >> 1) & 0x03)
#define FRAG_STATUS_REQ_PARAM_PARTICIPANTS(x)  ((x) & 0x01)

#define FRAG_SESSION_DELETE_REQ_PARAM_IDX      (2)
#define FRAG_SESSION_DELETE_REQ_FRAG_INDEX(x)  ((x) & 0x03)

#define FRAG_DATA_PARAM_IDX                    (2)

//=============================================================================

typedef void (*FTMImageReceivedCb_t)(void);

//=============================================================================
typedef struct __attribute__((packed)) FragSessionSetupReq_s {
  uint8_t   session;
  uint16_t  fragments;
  uint8_t   fsize;
  union {
    struct {
      uint8_t blk_ack_dly :3;
      uint8_t fmatrix     :3;
      uint8_t rfu         :2;
    };
    uint8_t   byte;
  } encoding;
  uint8_t   padding;
  uint32_t  decsriptor;
} FragSessionSetupReq_t;

/*!
 * Global  FTMPackage parameters
 */
typedef struct sFTMPackageParams {
    /*
     * [0 to 3] identifies one of the 4 simultaneously possible fragmentation session.
     */
    uint8_t FragIndex ;
    /*!
     *
     * specifies which multicast group addresses are allowed as input to this defragmentation session
     */
    uint8_t McGroupBitMask ;

    /*
     * identifies the fragmentation session and contains the following fields
     */
    uint8_t FragSession ;
    /*
     * specifies the total number of fragments of the data block to be transported
       * during the coming multicast fragmentation session
     */
    uint16_t NbFrag;

    /*!
     * is the size in byte of each fragment.
     */
    uint8_t FragSize;

    uint8_t Encoding;
    /*!
     * Specifies random delay between request and device reply for multicast requests
     * in seconds
     */
    uint16_t BlockAckDelay;
    /*!
    * specifies the total number of Redundancy fragments of the data block to be transported
    * during the coming multicast fragmentation session
    */
    uint16_t Redundancy;
    /*!
      * specifies number of padding bytes
      */
    uint8_t Padding;
    /*!
      * 32bit number describing the file that is going to be transported through the fragmentation session.
      * The encoding of this field is application specific.
      */
    uint32_t Descriptor;
} FTMPackageParams_t;

//=============================================================================
/*
* \brief Function that processes command targeted to Fragmented Data Block 
*        port (Fport: 201). It will also generate the reply and send it.
*
* \param[in] *appdata - Pointer to appCbParams_t indicating a downlink
*/
void FTMPackageCore(appCbParams_t *appdata);

/*
* \brief Sets the callback to be invoked when complete valid image is received
*
* \param cb - Pointer to callback function to be invoked by FTMPackage
*/
void FTMPackageSetImageReceivedCallback(FTMImageReceivedCb_t cb);

/*
* \brief Sets the custom FUOTA descriptor
*
* \param descriptor - Custom FUOTA descriptor value
*/
void FTMPackageSetFuotaDescriptor(uint32_t descriptor);

//=============================================================================

#endif //__FTMPACKAGECORE_H__

// eof FTMPackageCore.h
