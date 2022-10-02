/*
 * chap_ms.c - Microsoft MS-CHAP compatible implementation.
 *
 * Copyright (c) 1995 Eric Rosenquist.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * 3. The name(s) of the authors of this software must not be used to
 *    endorse or promote products derived from this software without
 *    prior written permission.
 *
 * THE AUTHORS OF THIS SOFTWARE DISCLAIM ALL WARRANTIES WITH REGARD TO
 * THIS SOFTWARE, INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS, IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY
 * SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN
 * AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING
 * OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * Modifications by Lauri Pesonen / lpesonen@clinet.fi, april 1997
 *
 *   Implemented LANManager type password response to MS-CHAP challenges.
 *   Now pppd provides both NT style and LANMan style blocks, and the
 *   prefered is set by option "ms-lanman". Default is to use NT.
 *   The hash text (StdText) was taken from Win95 RASAPI32.DLL.
 *
 *   You should also use DOMAIN\\USERNAME as described in README.MSCHAP80
 */

/*
 * Modifications by Frank Cusack, frank@google.com, March 2002.
 *
 *   Implemented MS-CHAPv2 functionality, heavily based on sample
 *   implementation in RFC 2759.  Implemented MPPE functionality,
 *   heavily based on sample implementation in RFC 3079.
 *
 * Copyright (c) 2002 Google, Inc.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * 3. The name(s) of the authors of this software must not be used to
 *    endorse or promote products derived from this software without
 *    prior written permission.
 *
 * THE AUTHORS OF THIS SOFTWARE DISCLAIM ALL WARRANTIES WITH REGARD TO
 * THIS SOFTWARE, INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS, IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY
 * SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN
 * AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING
 * OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include "lwip/opt.h"
#if PPP_SUPPORT && MSCHAP_SUPPORT  /* don't build if not configured for use in lwipopts.h */

#if 0 /* UNUSED */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#endif /* UNUSED */

#include "netif/ppp/ppp_impl.h"

#include "netif/ppp/chap-new.h"
#include "netif/ppp/chap_ms.h"
#include "netif/ppp/pppcrypt.h"
#include "netif/ppp/magic.h"

#if LWIP_INCLUDED_POLARSSL_MD4
#include "netif/ppp/polarssl/md4.h"
#else
#include "polarssl/md4.h"
#endif

#if LWIP_INCLUDED_POLARSSL_SHA1
#include "netif/ppp/polarssl/sha1.h"
#else
#include "polarssl/sha1.h"
#endif

#if LWIP_INCLUDED_POLARSSL_DES
#include "netif/ppp/polarssl/des.h"
#else
#include "polarssl/des.h"
#endif

#define SHA1_SIGNATURE_SIZE 20

static void	ascii2unicode (char[], int, u_char[]);
static void	NTPasswordHash (u_char *, int, u_char[MD4_SIGNATURE_SIZE]);
static void	ChallengeResponse (u_char *, u_char *, u_char[24]);
static void	ChapMS_NT (u_char *, char *, int, u_char[24]);
static void	ChapMS2_NT (u_char *, u_char[16], char *, char *, int,
				u_char[24]);
static void	GenerateAuthenticatorResponsePlain
			(char*, int, u_char[24], u_char[16], u_char *,
			     char *, u_char[41]);
#ifdef MSLANMAN
static void	ChapMS_LANMan (u_char *, char *, int, u_char *);
#endif

#ifdef MPPE
static void	Set_Start_Key (u_char *, char *, int);
static void	SetMasterKeys (char *, int, u_char[24], int);
#endif

#ifdef MSLANMAN
bool	ms_lanman = 0;    	/* Use LanMan password instead of NT */
			  	/* Has meaning only with MS-CHAP challenges */
#endif

#ifdef MPPE
u_char mppe_send_key[MPPE_MAX_KEY_LEN];
u_char mppe_recv_key[MPPE_MAX_KEY_LEN];
int mppe_keys_set = 0;		/* Have the MPPE keys been set? */

#ifdef DEBUGMPPEKEY
/* For MPPE debug */
/* Use "[]|}{?/><,`!2&&(" (sans quotes) for RFC 3079 MS-CHAPv2 test value */
static char *mschap_challenge = NULL;
/* Use "!@\#$%^&*()_+:3|~" (sans quotes, backslash is to escape #) for ... */
static char *mschap2_peer_challenge = NULL;
#endif

#include "netif/ppp/fsm.h"		/* Need to poke MPPE options */
#include "netif/ppp/ccp.h"
#include <net/ppp-comp.h>
#endif

#if PPP_OPTIONS
/*
 * Command-line options.
 */
static option_t chapms_option_list[] = {
#ifdef MSLANMAN
	{ "ms-lanman", o_bool, &ms_lanman,
	  "Use LanMan passwd when using MS-CHAP", 1 },
#endif
#ifdef DEBUGMPPEKEY
	{ "mschap-challenge", o_string, &mschap_challenge,
	  "specify CHAP challenge" },
	{ "mschap2-peer-challenge", o_string, &mschap2_peer_challenge,
	  "specify CHAP peer challenge" },
#endif
	{ NULL }
};
#endif /* PPP_OPTIONS */

#if PPP_SERVER
/*
 * chapms_generate_challenge - generate a challenge for MS-CHAP.
 * For MS-CHAP the challenge length is fixed at 8 bytes.
 * The length goes in challenge[0] and the actual challenge starts
 * at challenge[1].
 */
static void chapms_generate_challenge(unsigned char *challenge) {
	*challenge++ = 8;
#ifdef DEBUGMPPEKEY
	if (mschap_challenge && strlen(mschap_challenge) == 8)
		memcpy(challenge, mschap_challenge, 8);
	else
#endif
		random_bytes(challenge, 8);
}

static void chapms2_generate_challenge(unsigned char *challenge) {
	*challenge++ = 16;
#ifdef DEBUGMPPEKEY
	if (mschap_challenge && strlen(mschap_challenge) == 16)
		memcpy(challenge, mschap_challenge, 16);
	else
#endif
		random_bytes(challenge, 16);
}

static int chapms_verify_response(int id, const char *name,
		       const unsigned char *secret, int secret_len,
		       const unsigned char *challenge, const unsigned char *response,
		       char *message, int message_space) {
	unsigned char md[MS_CHAP_RESPONSE_LEN];
	int diff;
	int challenge_len, response_len;
	LWIP_UNUSED_ARG(id);
	LWIP_UNUSED_ARG(name);

	challenge_len = *challenge++;	/* skip length, is 8 */
	response_len = *response++;
	if (response_len != MS_CHAP_RESPONSE_LEN)
		goto bad;

#ifndef MSLANMAN
	if (!response[MS_CHAP_USENT]) {
		/* Should really propagate this into the error packet. */
		ppp_notice("Peer request for LANMAN auth not supported");
		goto bad;
	}
#endif

	/* Generate the expected response. */
	ChapMS((u_char *)challenge, (char *)secret, secret_len, md);

#ifdef MSLANMAN
	/* Determine which part of response to verify against */
	if (!response[MS_CHAP_USENT])
		diff = memcmp(&response[MS_CHAP_LANMANRESP],
			      &md[MS_CHAP_LANMANRESP], MS_CHAP_LANMANRESP_LEN);
	else
#endif
		diff = memcmp(&response[MS_CHAP_NTRESP], &md[MS_CHAP_NTRESP],
			      MS_CHAP_NTRESP_LEN);

	if (diff == 0) {
		ppp_slprintf(message, message_space, "Access granted");
		return 1;
	}

 bad:
	/* See comments below for MS-CHAP V2 */
	ppp_slprintf(message, message_space, "E=691 R=1 C=%0.*B V=0",
		 challenge_len, challenge);
	return 0;
}

static int chapms2_verify_response(int id, const char *name,
			const unsigned char *secret, int secret_len,
			const unsigned char *challenge, const unsigned char *response,
			char *message, int message_space) {
	unsigned char md[MS_CHAP2_RESPONSE_LEN];
	char saresponse[MS_AUTH_RESPONSE_LENGTH+1];
	int challenge_len, response_len;
	LWIP_UNUSED_ARG(id);

	challenge_len = *challenge++;	/* skip length, is 16 */
	response_len = *response++;
	if (response_len != MS_CHAP2_RESPONSE_LEN)
		goto bad;	/* not even the right length */

	/* Generate the expected response and our mutual auth. */
	ChapMS2((u_char*)challenge, (u_char*)&response[MS_CHAP2_PEER_CHALLENGE], (char*)name,
		(char *)secret, secret_len, md,
		(unsigned char *)saresponse, MS_CHAP2_AUTHENTICATOR);

	/* compare MDs and send the appropriate status */
	/*
	 * Per RFC 2759, success message must be formatted as
	 *     "S=<auth_string> M=<message>"
	 * where
	 *     <auth_string> is the Authenticator Response (mutual auth)
	 *     <message> is a text message
	 *
	 * However, some versions of Windows (win98 tested) do not know
	 * about the M=<message> part (required per RFC 2759) and flag
	 * it as an error (reported incorrectly as an encryption error
	 * to the user).  Since the RFC requires it, and it can be
	 * useful information, we supply it if the peer is a conforming
	 * system.  Luckily (?), win98 sets the Flags field to 0x04
	 * (contrary to RFC requirements) so we can use that to
	 * distinguish between conforming and non-conforming systems.
	 *
	 * Special thanks to Alex Swiridov <say@real.kharkov.ua> for
	 * help debugging this.
	 */
	if (memcmp(&md[MS_CHAP2_NTRESP], &response[MS_CHAP2_NTRESP],
		   MS_CHAP2_NTRESP_LEN) == 0) {
		if (response[MS_CHAP2_FLAGS])
			ppp_slprintf(message, message_space, "S=%s", saresponse);
		else
			ppp_slprintf(message, message_space, "S=%s M=%s",
				 saresponse, "Access granted");
		return 1;
	}

 bad:
	/*
	 * Failure message must be formatted as
	 *     "E=e R=r C=c V=v M=m"
	 * where
	 *     e = error code (we use 691, ERROR_AUTHENTICATION_FAILURE)
	 *     r = retry (we use 1, ok to retry)
	 *     c = challenge to use for next response, we reuse previous
	 *     v = Change Password version supported, we use 0
	 *     m = text message
	 *
	 * The M=m part is only for MS-CHAPv2.  Neither win2k nor
	 * win98 (others untested) display the message to the user anyway.
	 * They also both ignore the E=e code.
	 *
	 * Note that it's safe to reuse the same challenge as we don't
	 * actually accept another response based on the error message
	 * (and no clients try to resend a response anyway).
	 *
	 * Basically, this whole bit is useless code, even the small
	 * implementation here is only because of overspecification.
	 */
	ppp_slprintf(message, message_space, "E=691 R=1 C=%0.*B V=0 M=%s",
		 challenge_len, challenge, "Access denied");
	return 0;
}
#endif /* PPP_SERVER */

static void chapms_make_response(unsigned char *response, int id, const char *our_name,
		     const unsigned char *challenge, const char *secret, int secret_len,
		     const unsigned char *private_) {
	LWIP_UNUSED_ARG(id);
	LWIP_UNUSED_ARG(our_name);
	LWIP_UNUSED_ARG(private_);
	challenge++;	/* skip length, should be 8 */
	*response++ = MS_CHAP_RESPONSE_LEN;
	ChapMS((u_char*)challenge, (char*)secret, secret_len, response);
}

static void chapms2_make_response(unsigned char *response, int id, const char *our_name,
		      const unsigned char *challenge, const char *secret, int secret_len,
		      const unsigned char *private_) {
	LWIP_UNUSED_ARG(id);
	challenge++;	/* skip length, should be 16 */
	*response++ = MS_CHAP2_RESPONSE_LEN;
	ChapMS2((u_char*)challenge,
#ifdef DEBUGMPPEKEY
		mschap2_peer_challenge,
#else
		NULL,
#endif
		(char*)our_name, (char*)secret, secret_len, response, (u_char*)private_,
		MS_CHAP2_AUTHENTICATEE);
}

static int chapms2_check_success(unsigned char *msg, int len, unsigned char *private_) {
	if ((len < MS_AUTH_RESPONSE_LENGTH + 2) ||
	    strncmp((char *)msg, "S=", 2) != 0) {
		/* Packet does not start with "S=" */
		ppp_error("MS-CHAPv2 Success packet is badly formed.");
		return 0;
	}
	msg += 2;
	len -= 2;
	if (len < MS_AUTH_RESPONSE_LENGTH
	    || memcmp(msg, private_, MS_AUTH_RESPONSE_LENGTH)) {
		/* Authenticator Response did not match expected. */
		ppp_error("MS-CHAPv2 mutual authentication failed.");
		return 0;
	}
	/* Authenticator Response matches. */
	msg += MS_AUTH_RESPONSE_LENGTH; /* Eat it */
	len -= MS_AUTH_RESPONSE_LENGTH;
	if ((len >= 3) && !strncmp((char *)msg, " M=", 3)) {
		msg += 3; /* Eat the delimiter */
	} else if (len) {
		/* Packet has extra text which does not begin " M=" */
		ppp_error("MS-CHAPv2 Success packet is badly formed.");
		return 0;
	}
	return 1;
}

static void chapms_handle_failure(unsigned char *inp, int len) {
	int err;
	const char *p;
	char msg[64];

	/* We want a null-terminated string for strxxx(). */
	len = LWIP_MIN(len, 63);
	MEMCPY(msg, inp, len);
	msg[len] = 0;
	p = msg;

	/*
	 * Deal with MS-CHAP formatted failure messages; just print the
	 * M=<message> part (if any).  For MS-CHAP we're not really supposed
	 * to use M=<message>, but it shouldn't hurt.  See
	 * chapms[2]_verify_response.
	 */
	if (!strncmp(p, "E=", 2))
		err = strtol(p+2, NULL, 10); /* Remember the error code. */
	else
		goto print_msg; /* Message is badly formatted. */

	if (len && ((p = strstr(p, " M=")) != NULL)) {
		/* M=<message> field found. */
		p += 3;
	} else {
		/* No M=<message>; use the error code. */
		switch (err) {
		case MS_CHAP_ERROR_RESTRICTED_LOGON_HOURS:
			p = "E=646 Restricted logon hours";
			break;

		case MS_CHAP_ERROR_ACCT_DISABLED:
			p = "E=647 Account disabled";
			break;

		case MS_CHAP_ERROR_PASSWD_EXPIRED:
			p = "E=648 Password expired";
			break;

		case MS_CHAP_ERROR_NO_DIALIN_PERMISSION:
			p = "E=649 No dialin permission";
			break;

		case MS_CHAP_ERROR_AUTHENTICATION_FAILURE:
			p = "E=691 Authentication failure";
			break;

		case MS_CHAP_ERROR_CHANGING_PASSWORD:
			/* Should never see this, we don't support Change Password. */
			p = "E=709 Error changing password";
			break;

		default:
			ppp_error("Unknown MS-CHAP authentication failure: %.*v",
			      len, inp);
			return;
		}
	}
print_msg:
	if (p != NULL)
		ppp_error("MS-CHAP authentication failed: %v", p);
}

static void ChallengeResponse(u_char *challenge,
		  u_char PasswordHash[MD4_SIGNATURE_SIZE],
		  u_char response[24]) {
    u_char    ZPasswordHash[21];
    des_context des;
    u_char des_key[8];

    BZERO(ZPasswordHash, sizeof(ZPasswordHash));
    MEMCPY(ZPasswordHash, PasswordHash, MD4_SIGNATURE_SIZE);

#if 0
    dbglog("ChallengeResponse - ZPasswordHash %.*B",
	   sizeof(ZPasswordHash), ZPasswordHash);
#endif

    pppcrypt_56_to_64_bit_key(ZPasswordHash + 0, des_key);
    des_setkey_enc(&des, des_key);
    des_crypt_ecb(&des, challenge, response +0);

    pppcrypt_56_to_64_bit_key(ZPasswordHash + 7, des_key);
    des_setkey_enc(&des, des_key);
    des_crypt_ecb(&des, challenge, response +8);

    pppcrypt_56_to_64_bit_key(ZPasswordHash + 14, des_key);
    des_setkey_enc(&des, des_key);
    des_crypt_ecb(&des, challenge, response +16);

#if 0
    dbglog("ChallengeResponse - response %.24B", response);
#endif
}

void ChallengeHash(u_char PeerChallenge[16], u_char *rchallenge,
	      char *username, u_char Challenge[8]) {
    sha1_context	sha1Context;
    u_char	sha1Hash[SHA1_SIGNATURE_SIZE];
    char	*user;

    /* remove domain from "domain\username" */
    if ((user = strrchr(username, '\\')) != NULL)
	++user;
    else
	user = username;

    sha1_starts(&sha1Context);
    sha1_update(&sha1Context, PeerChallenge, 16);
    sha1_update(&sha1Context, rchallenge, 16);
    sha1_update(&sha1Context, (unsigned char *)user, strlen(user));
    sha1_finish(&sha1Context, sha1Hash);

    MEMCPY(Challenge, sha1Hash, 8);
}

/*
 * Convert the ASCII version of the password to Unicode.
 * This implicitly supports 8-bit ISO8859/1 characters.
 * This gives us the little-endian representation, which
 * is assumed by all M$ CHAP RFCs.  (Unicode byte ordering
 * is machine-dependent.)
 */
static void ascii2unicode(char ascii[], int ascii_len, u_char unicode[]) {
    int i;

    BZERO(unicode, ascii_len * 2);
    for (i = 0; i < ascii_len; i++)
	unicode[i * 2] = (u_char) ascii[i];
}

static void NTPasswordHash(u_char *secret, int secret_len, u_char hash[MD4_SIGNATURE_SIZE]) {
    md4_context		md4Context;

    md4_starts(&md4Context);
    md4_update(&md4Context, secret, secret_len);
    md4_finish(&md4Context, hash);
}

static void ChapMS_NT(u_char *rchallenge, char *secret, int secret_len,
	  u_char NTResponse[24]) {
    u_char	unicodePassword[MAX_NT_PASSWORD * 2];
    u_char	PasswordHash[MD4_SIGNATURE_SIZE];

    /* Hash the Unicode version of the secret (== password). */
    ascii2unicode(secret, secret_len, unicodePassword);
    NTPasswordHash(unicodePassword, secret_len * 2, PasswordHash);

    ChallengeResponse(rchallenge, PasswordHash, NTResponse);
}

static void ChapMS2_NT(u_char *rchallenge, u_char PeerChallenge[16], char *username,
	   char *secret, int secret_len, u_char NTResponse[24]) {
    u_char	unicodePassword[MAX_NT_PASSWORD * 2];
    u_char	PasswordHash[MD4_SIGNATURE_SIZE];
    u_char	Challenge[8];

    ChallengeHash(PeerChallenge, rchallenge, username, Challenge);

    /* Hash the Unicode version of the secret (== password). */
    ascii2unicode(secret, secret_len, unicodePassword);
    NTPasswordHash(unicodePassword, secret_len * 2, PasswordHash);

    ChallengeResponse(Challenge, PasswordHash, NTResponse);
}

#ifdef MSLANMAN
static u_char *StdText = (u_char *)"KGS!@#$%"; /* key from rasapi32.dll */

static void ChapMS_LANMan(u_char *rchallenge, char *secret, int secret_len,
	      unsigned char *response) {
    int			i;
    u_char		UcasePassword[MAX_NT_PASSWORD]; /* max is actually 14 */
    u_char		PasswordHash[MD4_SIGNATURE_SIZE];
    des_context des;
    u_char des_key[8];

    /* LANMan password is case insensitive */
    BZERO(UcasePassword, sizeof(UcasePassword));
    for (i = 0; i < secret_len; i++)
       UcasePassword[i] = (u_char)toupper(secret[i]);

    pppcrypt_56_to_64_bit_key(UcasePassword +0, des_key);
    des_setkey_enc(&des, des_key);
    des_crypt_ecb(&des, StdText, PasswordHash +0);

    pppcrypt_56_to_64_bit_key(UcasePassword +7, des_key);
    des_setkey_enc(&des, des_key);
    des_crypt_ecb(&des, StdText, PasswordHash +8);

    ChallengeResponse(rchallenge, PasswordHash, &response[MS_CHAP_LANMANRESP]);
}
#endif


void GenerateAuthenticatorResponse(u_char PasswordHashHash[MD4_SIGNATURE_SIZE],
			      u_char NTResponse[24], u_char PeerChallenge[16],
			      u_char *rchallenge, char *username,
			      u_char authResponse[MS_AUTH_RESPONSE_LENGTH+1]) {
    /*
     * "Magic" constants used in response generation, from RFC 2759.
     */
    u_char Magic1[39] = /* "Magic server to client signing constant" */
	{ 0x4D, 0x61, 0x67, 0x69, 0x63, 0x20, 0x73, 0x65, 0x72, 0x76,
	  0x65, 0x72, 0x20, 0x74, 0x6F, 0x20, 0x63, 0x6C, 0x69, 0x65,
	  0x6E, 0x74, 0x20, 0x73, 0x69, 0x67, 0x6E, 0x69, 0x6E, 0x67,
	  0x20, 0x63, 0x6F, 0x6E, 0x73, 0x74, 0x61, 0x6E, 0x74 };
    u_char Magic2[41] = /* "Pad to make it do more than one iteration" */
	{ 0x50, 0x61, 0x64, 0x20, 0x74, 0x6F, 0x20, 0x6D, 0x61, 0x6B,
	  0x65, 0x20, 0x69, 0x74, 0x20, 0x64, 0x6F, 0x20, 0x6D, 0x6F,
	  0x72, 0x65, 0x20, 0x74, 0x68, 0x61, 0x6E, 0x20, 0x6F, 0x6E,
	  0x65, 0x20, 0x69, 0x74, 0x65, 0x72, 0x61, 0x74, 0x69, 0x6F,
	  0x6E };

    int		i;
    sha1_context	sha1Context;
    u_char	Digest[SHA1_SIGNATURE_SIZE];
    u_char	Challenge[8];

    sha1_starts(&sha1Context);
    sha1_update(&sha1Context, PasswordHashHash, MD4_SIGNATURE_SIZE);
    sha1_update(&sha1Context, NTResponse, 24);
    sha1_update(&sha1Context, Magic1, sizeof(Magic1));
    sha1_finish(&sha1Context, Digest);

    ChallengeHash(PeerChallenge, rchallenge, username, Challenge);

    sha1_starts(&sha1Context);
    sha1_update(&sha1Context, Digest, sizeof(Digest));
    sha1_update(&sha1Context, Challenge, sizeof(Challenge));
    sha1_update(&sha1Context, Magic2, sizeof(Magic2));
    sha1_finish(&sha1Context, Digest);

    /* Convert to ASCII hex string. */
    for (i = 0; i < LWIP_MAX((MS_AUTH_RESPONSE_LENGTH / 2), (int)sizeof(Digest)); i++)
	sprintf((char *)&authResponse[i * 2], "%02X", Digest[i]);
}


static void GenerateAuthenticatorResponsePlain
		(char *secret, int secret_len,
		 u_char NTResponse[24], u_char PeerChallenge[16],
		 u_char *rchallenge, char *username,
		 u_char authResponse[MS_AUTH_RESPONSE_LENGTH+1]) {
    u_char	unicodePassword[MAX_NT_PASSWORD * 2];
    u_char	PasswordHash[MD4_SIGNATURE_SIZE];
    u_char	PasswordHashHash[MD4_SIGNATURE_SIZE];

    /* Hash (x2) the Unicode version of the secret (== password). */
    ascii2unicode(secret, secret_len, unicodePassword);
    NTPasswordHash(unicodePassword, secret_len * 2, PasswordHash);
    NTPasswordHash(PasswordHash, sizeof(PasswordHash),
		   PasswordHashHash);

    GenerateAuthenticatorResponse(PasswordHashHash, NTResponse, PeerChallenge,
				  rchallenge, username, authResponse);
}


#ifdef MPPE
/*
 * Set mppe_xxxx_key from the NTPasswordHashHash.
 * RFC 2548 (RADIUS support) requires us to export this function (ugh).
 */
void mppe_set_keys(u_char *rchallenge, u_char PasswordHashHash[MD4_SIGNATURE_SIZE]) {
    sha1_context	sha1Context;
    u_char	Digest[SHA1_SIGNATURE_SIZE];	/* >= MPPE_MAX_KEY_LEN */

    sha1_starts(&sha1Context);
    sha1_update(&sha1Context, PasswordHashHash, MD4_SIGNATURE_SIZE);
    sha1_update(&sha1Context, PasswordHashHash, MD4_SIGNATURE_SIZE);
    sha1_update(&sha1Context, rchallenge, 8);
    sha1_finish(&sha1Context, Digest);

    /* Same key in both directions. */
    MEMCPY(mppe_send_key, Digest, sizeof(mppe_send_key));
    MEMCPY(mppe_recv_key, Digest, sizeof(mppe_recv_key));

    mppe_keys_set = 1;
}

/*
 * Set mppe_xxxx_key from MS-CHAP credentials. (see RFC 3079)
 */
static void Set_Start_Key(u_char *rchallenge, char *secret, int secret_len) {
    u_char	unicodePassword[MAX_NT_PASSWORD * 2];
    u_char	PasswordHash[MD4_SIGNATURE_SIZE];
    u_char	PasswordHashHash[MD4_SIGNATURE_SIZE];

    /* Hash (x2) the Unicode version of the secret (== password). */
    ascii2unicode(secret, secret_len, unicodePassword);
    NTPasswordHash(unicodePassword, secret_len * 2, PasswordHash);
    NTPasswordHash(PasswordHash, sizeof(PasswordHash), PasswordHashHash);

    mppe_set_keys(rchallenge, PasswordHashHash);
}

/*
 * Set mppe_xxxx_key from MS-CHAPv2 credentials. (see RFC 3079)
 *
 * This helper function used in the Winbind module, which gets the
 * NTHashHash from the server.
 */
void mppe_set_keys2(u_char PasswordHashHash[MD4_SIGNATURE_SIZE],
	       u_char NTResponse[24], int IsServer) {
    sha1_context	sha1Context;
    u_char	MasterKey[SHA1_SIGNATURE_SIZE];	/* >= MPPE_MAX_KEY_LEN */
    u_char	Digest[SHA1_SIGNATURE_SIZE];	/* >= MPPE_MAX_KEY_LEN */

    u_char SHApad1[40] =
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    u_char SHApad2[40] =
	{ 0xf2, 0xf2, 0xf2, 0xf2, 0xf2, 0xf2, 0xf2, 0xf2, 0xf2, 0xf2,
	  0xf2, 0xf2, 0xf2, 0xf2, 0xf2, 0xf2, 0xf2, 0xf2, 0xf2, 0xf2,
	  0xf2, 0xf2, 0xf2, 0xf2, 0xf2, 0xf2, 0xf2, 0xf2, 0xf2, 0xf2,
	  0xf2, 0xf2, 0xf2, 0xf2, 0xf2, 0xf2, 0xf2, 0xf2, 0xf2, 0xf2 };

    /* "This is the MPPE Master Key" */
    u_char Magic1[27] =
	{ 0x54, 0x68, 0x69, 0x73, 0x20, 0x69, 0x73, 0x20, 0x74,
	  0x68, 0x65, 0x20, 0x4d, 0x50, 0x50, 0x45, 0x20, 0x4d,
	  0x61, 0x73, 0x74, 0x65, 0x72, 0x20, 0x4b, 0x65, 0x79 };
    /* "On the client side, this is the send key; "
       "on the server side, it is the receive key." */
    u_char Magic2[84] =
	{ 0x4f, 0x6e, 0x20, 0x74, 0x68, 0x65, 0x20, 0x63, 0x6c, 0x69,
	  0x65, 0x6e, 0x74, 0x20, 0x73, 0x69, 0x64, 0x65, 0x2c, 0x20,
	  0x74, 0x68, 0x69, 0x73, 0x20, 0x69, 0x73, 0x20, 0x74, 0x68,
	  0x65, 0x20, 0x73, 0x65, 0x6e, 0x64, 0x20, 0x6b, 0x65, 0x79,
	  0x3b, 0x20, 0x6f, 0x6e, 0x20, 0x74, 0x68, 0x65, 0x20, 0x73,
	  0x65, 0x72, 0x76, 0x65, 0x72, 0x20, 0x73, 0x69, 0x64, 0x65,
	  0x2c, 0x20, 0x69, 0x74, 0x20, 0x69, 0x73, 0x20, 0x74, 0x68,
	  0x65, 0x20, 0x72, 0x65, 0x63, 0x65, 0x69, 0x76, 0x65, 0x20,
	  0x6b, 0x65, 0x79, 0x2e };
    /* "On the client side, this is the receive key; "
       "on the server side, it is the send key." */
    u_char Magic3[84] =
	{ 0x4f, 0x6e, 0x20, 0x74, 0x68, 0x65, 0x20, 0x63, 0x6c, 0x69,
	  0x65, 0x6e, 0x74, 0x20, 0x73, 0x69, 0x64, 0x65, 0x2c, 0x20,
	  0x74, 0x68, 0x69, 0x73, 0x20, 0x69, 0x73, 0x20, 0x74, 0x68,
	  0x65, 0x20, 0x72, 0x65, 0x63, 0x65, 0x69, 0x76, 0x65, 0x20,
	  0x6b, 0x65, 0x79, 0x3b, 0x20, 0x6f, 0x6e, 0x20, 0x74, 0x68,
	  0x65, 0x20, 0x73, 0x65, 0x72, 0x76, 0x65, 0x72, 0x20, 0x73,
	  0x69, 0x64, 0x65, 0x2c, 0x20, 0x69, 0x74, 0x20, 0x69, 0x73,
	  0x20, 0x74, 0x68, 0x65, 0x20, 0x73, 0x65, 0x6e, 0x64, 0x20,
	  0x6b, 0x65, 0x79, 0x2e };
    u_char *s;

    sha1_starts(&sha1Context);
    sha1_update(&sha1Context, PasswordHashHash, MD4_SIGNATURE_SIZE);
    sha1_update(&sha1Context, NTResponse, 24);
    sha1_update(&sha1Context, Magic1, sizeof(Magic1));
    sha1_finish(&sha1Context, MasterKey);

    /*
     * generate send key
     */
    if (IsServer)
	s = Magic3;
    else
	s = Magic2;
    sha1_starts(&sha1Context);
    sha1_update(&sha1Context, MasterKey, 16);
    sha1_update(&sha1Context, SHApad1, sizeof(SHApad1));
    sha1_update(&sha1Context, s, 84);
    sha1_update(&sha1Context, SHApad2, sizeof(SHApad2));
    sha1_finish(&sha1Context, Digest);

    MEMCPY(mppe_send_key, Digest, sizeof(mppe_send_key));

    /*
     * generate recv key
     */
    if (IsServer)
	s = Magic2;
    else
	s = Magic3;
    sha1_starts(&sha1Context);
    sha1_update(&sha1Context, MasterKey, 16);
    sha1_update(&sha1Context, SHApad1, sizeof(SHApad1));
    sha1_update(&sha1Context, s, 84);
    sha1_update(&sha1Context, SHApad2, sizeof(SHApad2));
    sha1_finish(&sha1Context, Digest);

    MEMCPY(mppe_recv_key, Digest, sizeof(mppe_recv_key));

    mppe_keys_set = 1;
}

/*
 * Set mppe_xxxx_key from MS-CHAPv2 credentials. (see RFC 3079)
 */
static void SetMasterKeys(char *secret, int secret_len, u_char NTResponse[24], int IsServer) {
    u_char	unicodePassword[MAX_NT_PASSWORD * 2];
    u_char	PasswordHash[MD4_SIGNATURE_SIZE];
    u_char	PasswordHashHash[MD4_SIGNATURE_SIZE];
    /* Hash (x2) the Unicode version of the secret (== password). */
    ascii2unicode(secret, secret_len, unicodePassword);
    NTPasswordHash(unicodePassword, secret_len * 2, PasswordHash);
    NTPasswordHash(PasswordHash, sizeof(PasswordHash), PasswordHashHash);
    mppe_set_keys2(PasswordHashHash, NTResponse, IsServer);
}

#endif /* MPPE */


void ChapMS(u_char *rchallenge, char *secret, int secret_len,
       unsigned char *response) {
    BZERO(response, MS_CHAP_RESPONSE_LEN);

    ChapMS_NT(rchallenge, secret, secret_len, &response[MS_CHAP_NTRESP]);

#ifdef MSLANMAN
    ChapMS_LANMan(rchallenge, secret, secret_len,
		  &response[MS_CHAP_LANMANRESP]);

    /* preferred method is set by option  */
    response[MS_CHAP_USENT] = !ms_lanman;
#else
    response[MS_CHAP_USENT] = 1;
#endif

#ifdef MPPE
    Set_Start_Key(rchallenge, secret, secret_len);
#endif
}


/*
 * If PeerChallenge is NULL, one is generated and the PeerChallenge
 * field of response is filled in.  Call this way when generating a response.
 * If PeerChallenge is supplied, it is copied into the PeerChallenge field.
 * Call this way when verifying a response (or debugging).
 * Do not call with PeerChallenge = response.
 *
 * The PeerChallenge field of response is then used for calculation of the
 * Authenticator Response.
 */
void ChapMS2(u_char *rchallenge, u_char *PeerChallenge,
	char *user, char *secret, int secret_len, unsigned char *response,
	u_char authResponse[], int authenticator) {
    /* ARGSUSED */
    u_char *p = &response[MS_CHAP2_PEER_CHALLENGE];
    int i;
    LWIP_UNUSED_ARG(authenticator);

    BZERO(response, MS_CHAP2_RESPONSE_LEN);

    /* Generate the Peer-Challenge if requested, or copy it if supplied. */
    if (!PeerChallenge)
	for (i = 0; i < MS_CHAP2_PEER_CHAL_LEN; i++)
	    *p++ = (u_char) (drand48() * 0xff);
    else
	MEMCPY(&response[MS_CHAP2_PEER_CHALLENGE], PeerChallenge,
	      MS_CHAP2_PEER_CHAL_LEN);

    /* Generate the NT-Response */
    ChapMS2_NT(rchallenge, &response[MS_CHAP2_PEER_CHALLENGE], user,
	       secret, secret_len, &response[MS_CHAP2_NTRESP]);

    /* Generate the Authenticator Response. */
    GenerateAuthenticatorResponsePlain(secret, secret_len,
				       &response[MS_CHAP2_NTRESP],
				       &response[MS_CHAP2_PEER_CHALLENGE],
				       rchallenge, user, authResponse);

#ifdef MPPE
    SetMasterKeys(secret, secret_len,
		  &response[MS_CHAP2_NTRESP], authenticator);
#endif
}

#ifdef MPPE
/*
 * Set MPPE options from plugins.
 */
void set_mppe_enc_types(int policy, int types) {
    /* Early exit for unknown policies. */
    if (policy != MPPE_ENC_POL_ENC_ALLOWED ||
	policy != MPPE_ENC_POL_ENC_REQUIRED)
	return;

    /* Don't modify MPPE if it's optional and wasn't already configured. */
    if (policy == MPPE_ENC_POL_ENC_ALLOWED && !ccp_wantoptions[0].mppe)
	return;

    /*
     * Disable undesirable encryption types.  Note that we don't ENABLE
     * any encryption types, to avoid overriding manual configuration.
     */
    switch(types) {
	case MPPE_ENC_TYPES_RC4_40:
	    ccp_wantoptions[0].mppe &= ~MPPE_OPT_128;	/* disable 128-bit */
	    break;
	case MPPE_ENC_TYPES_RC4_128:
	    ccp_wantoptions[0].mppe &= ~MPPE_OPT_40;	/* disable 40-bit */
	    break;
	default:
	    break;
    }
}
#endif /* MPPE */

const struct chap_digest_type chapms_digest = {
	CHAP_MICROSOFT,		/* code */
#if PPP_SERVER
	chapms_generate_challenge,
	chapms_verify_response,
#endif /* PPP_SERVER */
	chapms_make_response,
	NULL,			/* check_success */
	chapms_handle_failure,
};

const struct chap_digest_type chapms2_digest = {
	CHAP_MICROSOFT_V2,	/* code */
#if PPP_SERVER
	chapms2_generate_challenge,
	chapms2_verify_response,
#endif /* PPP_SERVER */
	chapms2_make_response,
	chapms2_check_success,
	chapms_handle_failure,
};

#endif /* PPP_SUPPORT && MSCHAP_SUPPORT */
