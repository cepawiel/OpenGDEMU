/*
 *  Diffie-Hellman-Merkle key exchange
 *
 *  Copyright (C) 2006-2010, Brainspark B.V.
 *
 *  This file is part of PolarSSL (http://www.polarssl.org)
 *  Lead Maintainer: Paul Bakker <polarssl_maintainer at polarssl.org>
 *
 *  All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
/*
 *  Reference:
 *
 *  http://www.cacr.math.uwaterloo.ca/hac/ (chapter 12)
 */

#include "polarssl/config.h"

#if defined(POLARSSL_DHM_C)

#include "polarssl/dhm.h"

#include <string.h>

/*
 * helper to validate the mpi size and import it
 */
static int dhm_read_bignum( mpi *X,
                            unsigned char **p,
                            const unsigned char *end )
{
    int ret, n;

    if( end - *p < 2 )
        return( POLARSSL_ERR_DHM_BAD_INPUT_DATA );

    n = ( (*p)[0] << 8 ) | (*p)[1];
    (*p) += 2;

    if( (int)( end - *p ) < n )
        return( POLARSSL_ERR_DHM_BAD_INPUT_DATA );

    if( ( ret = mpi_read_binary( X, *p, n ) ) != 0 )
        return( POLARSSL_ERR_DHM_READ_PARAMS_FAILED | ret );

    (*p) += n;

    return( 0 );
}

/*
 * Parse the ServerKeyExchange parameters
 */
int dhm_read_params( dhm_context *ctx,
                     unsigned char **p,
                     const unsigned char *end )
{
    int ret, n;

    memset( ctx, 0, sizeof( dhm_context ) );

    if( ( ret = dhm_read_bignum( &ctx->P,  p, end ) ) != 0 ||
        ( ret = dhm_read_bignum( &ctx->G,  p, end ) ) != 0 ||
        ( ret = dhm_read_bignum( &ctx->GY, p, end ) ) != 0 )
        return( ret );

    ctx->len = mpi_size( &ctx->P );

    if( end - *p < 2 )
        return( POLARSSL_ERR_DHM_BAD_INPUT_DATA );

    n = ( (*p)[0] << 8 ) | (*p)[1];
    (*p) += 2;

    if( end != *p + n )
        return( POLARSSL_ERR_DHM_BAD_INPUT_DATA );

    return( 0 );
}

/*
 * Setup and write the ServerKeyExchange parameters
 */
int dhm_make_params( dhm_context *ctx, int x_size,
                     unsigned char *output, int *olen,
                     int (*f_rng)(void *), void *p_rng )
{
    int i, ret, n, n1, n2, n3;
    unsigned char *p;

    /*
     * Generate X as large as possible ( < P )
     */
    n = x_size / sizeof( t_int );
    MPI_CHK( mpi_grow( &ctx->X, n ) );
    MPI_CHK( mpi_lset( &ctx->X, 0 ) );

    p = (unsigned char *) ctx->X.p;
    for( i = 0; i < x_size - 1; i++ )
        *p++ = (unsigned char) f_rng( p_rng );

    while( mpi_cmp_mpi( &ctx->X, &ctx->P ) >= 0 )
           mpi_shift_r( &ctx->X, 1 );

    /*
     * Calculate GX = G^X mod P
     */
    MPI_CHK( mpi_exp_mod( &ctx->GX, &ctx->G, &ctx->X,
                          &ctx->P , &ctx->RP ) );

    /*
     * export P, G, GX
     */
#define DHM_MPI_EXPORT(X,n)                     \
    MPI_CHK( mpi_write_binary( X, p + 2, n ) ); \
    *p++ = (unsigned char)( n >> 8 );           \
    *p++ = (unsigned char)( n      ); p += n;

    n1 = mpi_size( &ctx->P  );
    n2 = mpi_size( &ctx->G  );
    n3 = mpi_size( &ctx->GX );

    p = output;
    DHM_MPI_EXPORT( &ctx->P , n1 );
    DHM_MPI_EXPORT( &ctx->G , n2 );
    DHM_MPI_EXPORT( &ctx->GX, n3 );

    *olen  = p - output;

    ctx->len = n1;

cleanup:

    if( ret != 0 )
        return( ret | POLARSSL_ERR_DHM_MAKE_PARAMS_FAILED );

    return( 0 );
}

/*
 * Import the peer's public value G^Y
 */
int dhm_read_public( dhm_context *ctx,
                     const unsigned char *input, int ilen )
{
    int ret;

    if( ctx == NULL || ilen < 1 || ilen > ctx->len )
        return( POLARSSL_ERR_DHM_BAD_INPUT_DATA );

    if( ( ret = mpi_read_binary( &ctx->GY, input, ilen ) ) != 0 )
        return( POLARSSL_ERR_DHM_READ_PUBLIC_FAILED | ret );

    return( 0 );
}

/*
 * Create own private value X and export G^X
 */
int dhm_make_public( dhm_context *ctx, int x_size,
                     unsigned char *output, int olen,
                     int (*f_rng)(void *), void *p_rng )
{
    int ret, i, n;
    unsigned char *p;

    if( ctx == NULL || olen < 1 || olen > ctx->len )
        return( POLARSSL_ERR_DHM_BAD_INPUT_DATA );

    /*
     * generate X and calculate GX = G^X mod P
     */
    n = x_size / sizeof( t_int );
    MPI_CHK( mpi_grow( &ctx->X, n ) );
    MPI_CHK( mpi_lset( &ctx->X, 0 ) );

    n = x_size - 1;
    p = (unsigned char *) ctx->X.p;
    for( i = 0; i < n; i++ )
        *p++ = (unsigned char) f_rng( p_rng );

    while( mpi_cmp_mpi( &ctx->X, &ctx->P ) >= 0 )
           mpi_shift_r( &ctx->X, 1 );

    MPI_CHK( mpi_exp_mod( &ctx->GX, &ctx->G, &ctx->X,
                          &ctx->P , &ctx->RP ) );

    MPI_CHK( mpi_write_binary( &ctx->GX, output, olen ) );

cleanup:

    if( ret != 0 )
        return( POLARSSL_ERR_DHM_MAKE_PUBLIC_FAILED | ret );

    return( 0 );
}

/*
 * Derive and export the shared secret (G^Y)^X mod P
 */
int dhm_calc_secret( dhm_context *ctx,
                     unsigned char *output, int *olen )
{
    int ret;

    if( ctx == NULL || *olen < ctx->len )
        return( POLARSSL_ERR_DHM_BAD_INPUT_DATA );

    MPI_CHK( mpi_exp_mod( &ctx->K, &ctx->GY, &ctx->X,
                          &ctx->P, &ctx->RP ) );

    *olen = mpi_size( &ctx->K );

    MPI_CHK( mpi_write_binary( &ctx->K, output, *olen ) );

cleanup:

    if( ret != 0 )
        return( POLARSSL_ERR_DHM_CALC_SECRET_FAILED | ret );

    return( 0 );
}

/*
 * Free the components of a DHM key
 */
void dhm_free( dhm_context *ctx )
{
    mpi_free( &ctx->RP, &ctx->K, &ctx->GY,
              &ctx->GX, &ctx->X, &ctx->G,
              &ctx->P, NULL );
}

#if defined(POLARSSL_SELF_TEST)

/*
 * Checkup routine
 */
int dhm_self_test( int verbose )
{
    return( verbose++ );
}

#endif

#endif
