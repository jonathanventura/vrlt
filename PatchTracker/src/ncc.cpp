/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 * and the terms and conditions of the GNU Lesser General Public License Version 2.1.
 *
 * File: ncc.cpp
 * Author: Jonathan Ventura
 * Last Modified: 12.2.2012
 */

#include <PatchTracker/ncc.h>

#ifdef __ARM_NEON__
#include <arm_neon.h>
#endif

#include <cstdio>

namespace vrlt {
    
    void memcpy8( unsigned char *output, unsigned char *input )
    {
#ifdef __ARM_NEON__
        uint8x8_t v0 = vld1_u8( input );
        vst1_u8( output, v0 );
#else
        for ( int i = 0; i < 8; i++ ) output[i] = input[i];
#endif
    }
    
    unsigned int computeSum( unsigned char *x, int rowstep )
    {
#ifdef __ARM_NEON__
        // load values
        uint8x8_t v[8];
        unsigned char *ptr = x;
        for ( int i = 0; i < 8; i++,ptr += rowstep ) v[i] = vld1_u8( ptr );
        
        // pairwise adds (64 uchar -> 4 ushort)
        uint16x4_t sum4 = vpaddl_u8( v[0] );
        sum4 = vpadal_u8( sum4, v[1] );
        sum4 = vpadal_u8( sum4, v[2] );
        sum4 = vpadal_u8( sum4, v[3] );
        sum4 = vpadal_u8( sum4, v[4] );
        sum4 = vpadal_u8( sum4, v[5] );
        sum4 = vpadal_u8( sum4, v[6] );
        sum4 = vpadal_u8( sum4, v[7] );
        
        // pairwise add ( 4 ushort -> 2 uint )
        uint32x2_t sum2 = vpaddl_u16( sum4 );
        
        // pairwise add ( 2 uint -> 1 ulong )
        uint64x1_t sum1 = vpaddl_u32( sum2 );
        
        // get 1 ulong
        uint64_t sum;
        vst1_u64( &sum, sum1 );
        
        return (unsigned int) sum;
#else
        unsigned int result = 0;
        unsigned char *ptr = x;
        for ( int i = 0; i < 8; i++,ptr+=rowstep )
            for ( int j = 0; j < 8; j++ ) result += ptr[j];
        return result;
#endif
    }
    
    unsigned int computeSum( unsigned char *x )
    {
#ifdef __ARM_NEON__
        // load values
        uint8x16x4_t v = vld4q_u8( x );
        
        // pairwise adds (64 uchar -> 4 ushort)
        uint16x8_t sum8 = vpaddlq_u8( v.val[0] );
        sum8 = vpadalq_u8( sum8, v.val[1] );
        sum8 = vpadalq_u8( sum8, v.val[2] );
        sum8 = vpadalq_u8( sum8, v.val[3] );
        
        // pairwise add (8 ushort -> 4 uint )
        uint32x4_t sum4 = vpaddlq_u16( sum8 );
        
        // pairwise add (4 uint -> 2 ulong )
        uint64x2_t sum2 = vpaddlq_u32( sum4 );
        
        // get 2 ulong
        uint64_t sums[2];
        vst1q_u64( sums, sum2 );
        
        return (unsigned int) (sums[0] + sums[1]);
#else
        unsigned int result = 0;
        for ( int i = 0; i < 64; i++ ) result += x[i];
        return result;
#endif
    }
    
    unsigned int computeSumSq( unsigned char *x, int rowstep )
    {
#ifdef __ARM_NEON__
        // load values
        uint8x8x4_t v0;
        uint8x8x4_t v1;
        unsigned char *ptr = x;
        for ( int i = 0; i < 4; i++,ptr+=rowstep ) v0.val[i] = vld1_u8( ptr );
        for ( int i = 0; i < 4; i++,ptr+=rowstep ) v1.val[i] = vld1_u8( ptr );
        
        // multiply (8 uchar -> 8 ushort)
        // pairwise add and accumulate (8x8 ushort -> 4 uint)
        uint16x8_t prod8 = vmull_u8( v0.val[0], v0.val[0] );
        uint32x4_t sum4 = vpaddlq_u16( prod8 );
        prod8 = vmull_u8( v0.val[1], v0.val[1] );
        sum4 = vpadalq_u16( sum4, prod8 );
        prod8 = vmull_u8( v0.val[2], v0.val[2] );
        sum4 = vpadalq_u16( sum4, prod8 );
        prod8 = vmull_u8( v0.val[3], v0.val[3] );
        sum4 = vpadalq_u16( sum4, prod8 );
        prod8 = vmull_u8( v1.val[0], v1.val[0] );
        sum4 = vpadalq_u16( sum4, prod8 );
        prod8 = vmull_u8( v1.val[1], v1.val[1] );
        sum4 = vpadalq_u16( sum4, prod8 );
        prod8 = vmull_u8( v1.val[2], v1.val[2] );
        sum4 = vpadalq_u16( sum4, prod8 );
        prod8 = vmull_u8( v1.val[3], v1.val[3] );
        sum4 = vpadalq_u16( sum4, prod8 );
        
        // pairwise add (4 uint -> 2 ulong )
        uint64x2_t sum2 = vpaddlq_u32( sum4 );
        
        // get 2 ulong
        uint64_t sums[2];
        vst1q_u64( sums, sum2 );
        
        return (unsigned int) (sums[0] + sums[1]);
#else
        unsigned int result = 0;
        unsigned char *ptr = x;
        for ( int i = 0; i < 8; i++,ptr+=rowstep )
            for ( int j = 0; j < 8; j++ ) result += (unsigned int)ptr[j]*ptr[j];
        return result;
#endif
    }
    
    
    unsigned int computeSumSq( unsigned char *x )
    {
#ifdef __ARM_NEON__
        // load values
        uint8x8x4_t v0 = vld4_u8( x );
        uint8x8x4_t v1 = vld4_u8( x + 32 );
        
        // multiply (8 uchar -> 8 ushort)
        // pairwise add and accumulate (8x8 ushort -> 4 uint)
        uint16x8_t prod8 = vmull_u8( v0.val[0], v0.val[0] );
        uint32x4_t sum4 = vpaddlq_u16( prod8 );
        prod8 = vmull_u8( v0.val[1], v0.val[1] );
        sum4 = vpadalq_u16( sum4, prod8 );
        prod8 = vmull_u8( v0.val[2], v0.val[2] );
        sum4 = vpadalq_u16( sum4, prod8 );
        prod8 = vmull_u8( v0.val[3], v0.val[3] );
        sum4 = vpadalq_u16( sum4, prod8 );
        prod8 = vmull_u8( v1.val[0], v1.val[0] );
        sum4 = vpadalq_u16( sum4, prod8 );
        prod8 = vmull_u8( v1.val[1], v1.val[1] );
        sum4 = vpadalq_u16( sum4, prod8 );
        prod8 = vmull_u8( v1.val[2], v1.val[2] );
        sum4 = vpadalq_u16( sum4, prod8 );
        prod8 = vmull_u8( v1.val[3], v1.val[3] );
        sum4 = vpadalq_u16( sum4, prod8 );
        
        // pairwise add (4 uint -> 2 ulong )
        uint64x2_t sum2 = vpaddlq_u32( sum4 );
        
        // get 2 ulong
        uint64_t sums[2];
        vst1q_u64( sums, sum2 );
        
        return (unsigned int) (sums[0] + sums[1]);
#else
        unsigned int result = 0;
        for ( int i = 0; i < 64; i++ ) result += (unsigned int)x[i]*x[i];
        return result;
#endif
    }
    
    unsigned int computeDotProduct( unsigned char *x, int rowstep, unsigned char *y )
    {
#ifdef __ARM_NEON__
        // load values
        uint8x8x4_t vx;
        unsigned char *ptr = x;
        vx.val[0] = vld1_u8( ptr );     ptr += rowstep;
        vx.val[1] = vld1_u8( ptr );     ptr += rowstep;
        vx.val[2] = vld1_u8( ptr );     ptr += rowstep;
        vx.val[3] = vld1_u8( ptr );     ptr += rowstep;
        uint8x8x4_t vy;
        vy.val[0] = vld1_u8( y );
        vy.val[1] = vld1_u8( y + 8 );
        vy.val[2] = vld1_u8( y + 16 );
        vy.val[3] = vld1_u8( y + 24 );
        
        // multiply (8 uchar -> 8 ushort)
        // pairwise add and accumulate (8x8 ushort -> 4 uint)
        uint16x8_t prod8 = vmull_u8( vx.val[0], vy.val[0] );
        uint32x4_t sum4 = vpaddlq_u16( prod8 );
        prod8 = vmull_u8( vx.val[1], vy.val[1] );
        sum4 = vpadalq_u16( sum4, prod8 );
        prod8 = vmull_u8( vx.val[2], vy.val[2] );
        sum4 = vpadalq_u16( sum4, prod8 );
        prod8 = vmull_u8( vx.val[3], vy.val[3] );
        sum4 = vpadalq_u16( sum4, prod8 );
        
        // load values
        vx.val[0] = vld1_u8( ptr );     ptr += rowstep;
        vx.val[1] = vld1_u8( ptr );     ptr += rowstep;
        vx.val[2] = vld1_u8( ptr );     ptr += rowstep;
        vx.val[3] = vld1_u8( ptr );     ptr += rowstep;
        vy.val[0] = vld1_u8( y + 32 );
        vy.val[1] = vld1_u8( y + 40 );
        vy.val[2] = vld1_u8( y + 48 );
        vy.val[3] = vld1_u8( y + 56 );
        
        // pairwise add and accumulate
        prod8 = vmull_u8( vx.val[0], vy.val[0] );
        sum4 = vpadalq_u16( sum4, prod8 );
        prod8 = vmull_u8( vx.val[1], vy.val[1] );
        sum4 = vpadalq_u16( sum4, prod8 );
        prod8 = vmull_u8( vx.val[2], vy.val[2] );
        sum4 = vpadalq_u16( sum4, prod8 );
        prod8 = vmull_u8( vx.val[3], vy.val[3] );
        sum4 = vpadalq_u16( sum4, prod8 );
        
        // pairwise add (4 uint -> 2 ulong )
        uint64x2_t sum2 = vpaddlq_u32( sum4 );
        
        // get 2 ulong
        uint64_t sums[2];
        vst1q_u64( sums, sum2 );
        
        return (unsigned int) (sums[0] + sums[1]);
#else
        unsigned int result = 0;
        unsigned char *xptr = x;
        unsigned char *yptr = y;
        for ( int i = 0; i < 8; i++,xptr+=rowstep,yptr+=8 )
            for ( int j = 0; j < 8; j++ ) result += (unsigned int)xptr[j]*yptr[j];
        return result;
#endif
    }
    
    unsigned int computeDotProduct( unsigned char *x, unsigned char *y )
    {
#ifdef __ARM_NEON__
        // load values
        uint8x8x4_t vx = vld4_u8( x );
        uint8x8x4_t vy = vld4_u8( y );
        
        // multiply (8 uchar -> 8 ushort)
        // pairwise add and accumulate (8x8 ushort -> 4 uint)
        uint16x8_t prod8 = vmull_u8( vx.val[0], vy.val[0] );
        uint32x4_t sum4 = vpaddlq_u16( prod8 );
        prod8 = vmull_u8( vx.val[1], vy.val[1] );
        sum4 = vpadalq_u16( sum4, prod8 );
        prod8 = vmull_u8( vx.val[2], vy.val[2] );
        sum4 = vpadalq_u16( sum4, prod8 );
        prod8 = vmull_u8( vx.val[3], vy.val[3] );
        sum4 = vpadalq_u16( sum4, prod8 );
        
        // load values
        vx = vld4_u8( x + 32 );
        vy = vld4_u8( y + 32 );
        
        // pairwise add and accumulate
        prod8 = vmull_u8( vx.val[0], vy.val[0] );
        sum4 = vpadalq_u16( sum4, prod8 );
        prod8 = vmull_u8( vx.val[1], vy.val[1] );
        sum4 = vpadalq_u16( sum4, prod8 );
        prod8 = vmull_u8( vx.val[2], vy.val[2] );
        sum4 = vpadalq_u16( sum4, prod8 );
        prod8 = vmull_u8( vx.val[3], vy.val[3] );
        sum4 = vpadalq_u16( sum4, prod8 );
        
        // pairwise add (4 uint -> 2 ulong )
        uint64x2_t sum2 = vpaddlq_u32( sum4 );
        
        // get 2 ulong
        uint64_t sums[2];
        vst1q_u64( sums, sum2 );
        
        return (unsigned int) (sums[0] + sums[1]);
#else
        unsigned int result = 0;
        for ( int i = 0; i < 64; i++ ) result += (unsigned int)x[i]*y[i];
        return result;
#endif
    }
}
