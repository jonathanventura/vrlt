/*
 * Copyright (c) 2012. The Regents of the University of California. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: bruteforce.cpp
 * Author: Jonathan Ventura
 * Last Modified: 11.17.2012
 */

#include <FeatureMatcher/bruteforce.h>

#include <cstdio>
#include <cstdlib>

//#define BUFFERED

namespace vrlt {

static const char *distanceKernel =
"__kernel void\n"
"distanceKernel(\n"
"global const uchar16 *data,\n"
"global const uchar16 *query,\n"
"global uint *stored_distsq\n"
")\n"
"{\n"
"int i = get_global_id(0);\n"
"global const uchar16 *ptr = data + 8*i;\n"
"uint distsq = 0;\n"
"for ( int j = 0; j < 8; j++ )\n"
"{\n"
"    uchar16 diff = abs_diff( query[j], ptr[j] );\n"
"    distsq += diff.s0 * diff.s0;\n"
"    distsq += diff.s1 * diff.s1;\n"
"    distsq += diff.s2 * diff.s2;\n"
"    distsq += diff.s3 * diff.s3;\n"
"    distsq += diff.s4 * diff.s4;\n"
"    distsq += diff.s5 * diff.s5;\n"
"    distsq += diff.s6 * diff.s6;\n"
"    distsq += diff.s7 * diff.s7;\n"
"    distsq += diff.s8 * diff.s8;\n"
"    distsq += diff.s9 * diff.s9;\n"
"    distsq += diff.sa * diff.sa;\n"
"    distsq += diff.sb * diff.sb;\n"
"    distsq += diff.sc * diff.sc;\n"
"    distsq += diff.sd * diff.sd;\n"
"    distsq += diff.se * diff.se;\n"
"    distsq += diff.sf * diff.sf;\n"
"}\n"
"stored_distsq[i] = distsq;\n"
"}\n";

//struct DistancesKernelData
//{
//    unsigned char *data;
//    unsigned char *queries;
//    unsigned int *stored_distsq;
//    int N;
//};
//
//static void distancesKernelFn( void *context, size_t index )
//{
//    DistancesKernelData *d = (DistancesKernelData*)context;
//    int k = index/d->N;
//    int i = index%d->N;
//    
//    unsigned int distsq = 0;
//    for ( int j = 0; j < 128; j++ ) {
//        
//}



#ifndef BUFFERED
static const char *distancesKernel =
"__kernel void\n"
"distancesKernel(\n"
//"global uchar *data,\n"
//"global uchar *queries,\n"
"global float4 *data,\n"
"global float4 *queries,\n"
"global uint *stored_distsq,\n"
"const int N,\n"
"const int ncomponents\n"
")\n"
"{\n"
"uint index = get_global_id(0);\n"
"uint k = index/N;\n"
"uint i = index%N;\n"
"float distsq = 0;\n"
"int nj = ncomponents/4;\n"
"for ( int j = 0; j < nj; j++ ) {\n"
"    float4 diff = queries[k*32+j] - data[i*32+j];\n"
"    distsq += dot(diff,diff);\n"
//"    if ( distsq > 25.f*25.f*128.f ) break;\n"
"}\n"
"stored_distsq[index] = (uint)distsq;\n"
//"uint distsq = 0;\n"
//"for ( int j = 0; j < 128; j++ ) {\n"
//"   int queryval = queries[k*128+j];\n"
//"   int dataval = data[i*128+j];\n"
//"   int diff = queryval-dataval;\n"
//"   distsq += diff*diff;\n"
//"}\n"
//"stored_distsq[index] = distsq;\n"
//"for ( int j = 0; j < 8; j++ ) {\n"
//"    uchar16 diff = abs_diff( queries[k*8+j], data[i*8+j] );\n"
//"    distsq += diff.s0 * diff.s0;\n"
//"    distsq += diff.s1 * diff.s1;\n"
//"    distsq += diff.s2 * diff.s2;\n"
//"    distsq += diff.s3 * diff.s3;\n"
//"    distsq += diff.s4 * diff.s4;\n"
//"    distsq += diff.s5 * diff.s5;\n"
//"    distsq += diff.s6 * diff.s6;\n"
//"    distsq += diff.s7 * diff.s7;\n"
//"    distsq += diff.s8 * diff.s8;\n"
//"    distsq += diff.s9 * diff.s9;\n"
//"    distsq += diff.sa * diff.sa;\n"
//"    distsq += diff.sb * diff.sb;\n"
//"    distsq += diff.sc * diff.sc;\n"
//"    distsq += diff.sd * diff.sd;\n"
//"    distsq += diff.se * diff.se;\n"
//"    distsq += diff.sf * diff.sf;\n"
//"}\n"
//"stored_distsq[index] = distsq;\n"
"}\n";
#else
static const char *distancesKernel =
"__kernel void\n"
"distancesKernel(\n"
"global float4 *data,\n"
"global float4 *queries,\n"
"global uint *stored_distsq,\n"
"const int N,\n"
"const int total_size,\n"
"const int offset\n"
")\n"
"{\n"
"uint id = get_global_id(0);\n"
"uint index = id + offset;\n"
"if ( index >= total_size ) return;\n"
"uint k = index/N;\n"
"uint i = index%N;\n"
"float distsq = 0;\n"
"for ( int j = 0; j < 32; j++ ) {\n"
"    float4 diff = queries[k*32+j] - data[i*32+j];\n"
"    distsq += dot(diff,diff);\n"
"}\n"
"stored_distsq[id] = (uint)distsq;\n"
"}\n";
#endif

//static const char *distancesKernel =
//"#pragma OPENCL EXTENSION cl_khr_global_int32_base_atomics : enable\n"
//"__kernel void\n"
//"distancesKernel(\n"
//"global const uchar16 *data,\n"
//"global const uchar16 *queries,\n"
//"global uint *stored_distsq,\n"
//"const uint N\n"
//")\n"
//"{\n"
//"int j = get_global_id(0);\n"
//"int k = get_global_id(1);\n"
//"int i = get_global_id(2);\n"
//"uchar16 dataval = data[8*i+j];\n"
//"uchar16 queryval = queries[8*k+j];\n"
//"uchar16 diff = abs_diff( dataval, queryval );\n"
//"uint distsq;\n"
//"distsq = diff.s0 * diff.s0;\n"
//"distsq += diff.s1 * diff.s1;\n"
//"distsq += diff.s2 * diff.s2;\n"
//"distsq += diff.s3 * diff.s3;\n"
//"distsq += diff.s4 * diff.s4;\n"
//"distsq += diff.s5 * diff.s5;\n"
//"distsq += diff.s6 * diff.s6;\n"
//"distsq += diff.s7 * diff.s7;\n"
//"distsq += diff.s8 * diff.s8;\n"
//"distsq += diff.s9 * diff.s9;\n"
//"distsq += diff.sa * diff.sa;\n"
//"distsq += diff.sb * diff.sb;\n"
//"distsq += diff.sc * diff.sc;\n"
//"distsq += diff.sd * diff.sd;\n"
//"distsq += diff.se * diff.se;\n"
//"distsq += diff.sf * diff.sf;\n"
//"atom_add( stored_distsq + N*k + i, distsq );\n"
//"}\n";

void BruteForceNN::compileProgram( cl_program prog )
{
    int err = clBuildProgram( prog, 0, NULL, NULL, NULL, NULL );
    
    if ( err != CL_SUCCESS ) {
        size_t len;
        char buffer[2048];
        
        fprintf(stderr,"Error: Failed to build program executable!\n" );
        clGetProgramBuildInfo(prog, device_id, CL_PROGRAM_BUILD_LOG, sizeof(buffer), buffer, &len);
        fprintf(stderr,"%s\n", buffer);
        exit(0);
    }	
}

BruteForceNN::BruteForceNN() : N(0), data(NULL), float_data(NULL), data_mem( NULL )
{
    int err;

#ifndef BUFFERED
    err = clGetDeviceIDs(NULL, CL_DEVICE_TYPE_CPU, 1, &device_id, NULL);
#else
    err = clGetDeviceIDs(NULL, CL_DEVICE_TYPE_GPU, 1, &device_id, NULL);
#endif
    context = clCreateContext(0, 1, &device_id, NULL, NULL, &err);	
    
    dists_program = clCreateProgramWithSource( context, 1, &distancesKernel, NULL, &err );
    compileProgram( dists_program );
    
#ifdef BUFFERED
    response_buf_size = 4096;
    response_buf_mem = clCreateBuffer( context, CL_MEM_WRITE_ONLY, sizeof(unsigned int)*response_buf_size, NULL, NULL );
#endif
}

void BruteForceNN::setData( int _N, unsigned char *_data )
{  
    N = _N;
    data = _data;

//    data_mem = clCreateBuffer( context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(unsigned char)*128*N, data, NULL );

    delete [] float_data;
    float_data = new float[N*128];
    for ( int i = 0; i < N*128; i++ ) float_data[i] = data[i];
    
    if ( data_mem ) clReleaseMemObject( data_mem );
    data_mem = clCreateBuffer( context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(float)*128*N, float_data, NULL );
}

BruteForceNN::~BruteForceNN()
{
#ifdef BUFFERED
    clReleaseMemObject( response_buf_mem );
#endif
    
    delete [] float_data;
    clReleaseMemObject( data_mem );
    
    clReleaseProgram( dists_program );
    
    clReleaseContext( context );
    
}
/*
 void compute_distances( unsigned char *query )
 {
 unsigned char *ptr = data;
 for ( int i = 0; i < N; i++ )
 {
 unsigned int distsq = 0;
 unsigned char *queryptr = query;
 for ( int j = 0; j < 128; j++,queryptr++,ptr++ )
 {
 unsigned char dataval = *ptr;
 unsigned char queryval = *queryptr;
 unsigned int absdiff = (dataval>queryval)?dataval-queryval:queryval-dataval;
 distsq += absdiff*absdiff;
 }
 stored_distsq[i] = distsq;
 }
 }
 */
/*
 void compute_distances_CL( unsigned char *query )
 {
 int err;
 
 cl_mem query_mem = clCreateBuffer( context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(unsigned char)*128, query, NULL );
 
 size_t global_offset = 0;
 size_t global_size = N;
 
 err = CL_SUCCESS;
 err |= clSetKernelArg( dist_kernel, 0, sizeof(cl_mem), &data_mem );
 err |= clSetKernelArg( dist_kernel, 1, sizeof(cl_mem), &query_mem );
 err |= clSetKernelArg( dist_kernel, 2, sizeof(cl_mem), &response_mem );
 
 err = clEnqueueNDRangeKernel( commands, dist_kernel, 1, &global_offset, &global_size, NULL, 0, NULL, NULL );
 
 err = clEnqueueReadBuffer( commands, response_mem, CL_TRUE, 0, sizeof(unsigned int)*N, stored_distsq, 0, NULL, NULL );
 
 clReleaseMemObject( query_mem );
 }
 */

/*
 void compute_distances( size_t num_queries, unsigned char *queries, unsigned int *stored_distsqs )
 {
 unsigned char *query = queries;
 for ( int k = 0; k < num_queries; k++,query+=128 )
 {
 unsigned char *ptr = data;
 for ( int i = 0; i < N; i++ )
 {
 unsigned int distsq = 0;
 unsigned char *queryptr = query;
 for ( int j = 0; j < 128; j++,queryptr++,ptr++ )
 {
 unsigned char dataval = *ptr;
 unsigned char queryval = *queryptr;
 unsigned int absdiff = (dataval>queryval)?dataval-queryval:queryval-dataval;
 distsq += absdiff*absdiff;
 }
 stored_distsqs[N*k+i] = distsq;
 }
 }
 }
 */

#ifndef BUFFERED
void BruteForceNN::getDistances( size_t num_queries, unsigned char *queries, unsigned int *stored_distsqs )
{
    int err;

    cl_command_queue commands = clCreateCommandQueue( context, device_id, 0, &err );
    cl_kernel dists_kernel = clCreateKernel( dists_program, "distancesKernel", &err );
    
//    cl_mem queries_mem = clCreateBuffer( context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(unsigned char)*128*num_queries, queries, NULL );
    float *float_queries = new float[128*num_queries];
    for ( int i = 0; i < 128*num_queries; i++ ) float_queries[i] = queries[i];
    
    cl_mem queries_mem = clCreateBuffer( context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(float)*128*num_queries, float_queries, NULL );
    cl_mem responses_mem = clCreateBuffer( context, CL_MEM_WRITE_ONLY | CL_MEM_USE_HOST_PTR, sizeof(unsigned int)*N*num_queries, stored_distsqs, NULL );
    
    size_t global_size[1] = { num_queries*N };
    
    int ncomponents = 128;
    
    err = CL_SUCCESS;
    err |= clSetKernelArg( dists_kernel, 0, sizeof(cl_mem), &data_mem );
    err |= clSetKernelArg( dists_kernel, 1, sizeof(cl_mem), &queries_mem );
    err |= clSetKernelArg( dists_kernel, 2, sizeof(cl_mem), &responses_mem );
    err |= clSetKernelArg( dists_kernel, 3, sizeof(cl_uint), &N );
    err |= clSetKernelArg( dists_kernel, 4, sizeof(cl_int), &ncomponents );
    
    err = clEnqueueNDRangeKernel( commands, dists_kernel, 1, NULL, global_size, NULL, 0, NULL, NULL );
    
    err = clEnqueueReadBuffer( commands, responses_mem, CL_TRUE, 0, sizeof(unsigned int)*N*num_queries, stored_distsqs, 0, NULL, NULL );
    
    clReleaseMemObject( queries_mem );
    clReleaseMemObject( responses_mem );

    clReleaseKernel( dists_kernel );
    clReleaseCommandQueue( commands );
    
    delete [] float_queries;
}
#else
void BruteForceNN::getDistances( size_t num_queries, unsigned char *queries, unsigned int *stored_distsqs )
{
    int err;
    
//    cl_mem queries_mem = clCreateBuffer( context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(unsigned char)*128*num_queries, queries, NULL );
    
    float *float_queries = new float[128*num_queries];
    for ( int i = 0; i < 128*num_queries; i++ ) float_queries[i] = queries[i];
    
    cl_mem queries_mem = clCreateBuffer( context, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, sizeof(float)*128*num_queries, float_queries, NULL );
    
    int total_size = num_queries*N;

    err = CL_SUCCESS;
    err |= clSetKernelArg( dists_kernel, 0, sizeof(cl_mem), &data_mem );
    err |= clSetKernelArg( dists_kernel, 1, sizeof(cl_mem), &queries_mem );
    err |= clSetKernelArg( dists_kernel, 2, sizeof(cl_mem), &response_buf_mem );
    err |= clSetKernelArg( dists_kernel, 3, sizeof(cl_int), &N );
    err |= clSetKernelArg( dists_kernel, 4, sizeof(cl_int), &total_size );

    for ( int offset = 0; offset < total_size; offset+=response_buf_size )
    {
        size_t global_size[1] = { response_buf_size };
        
        err = clSetKernelArg( dists_kernel, 5, sizeof(cl_int), &offset );
        
        err = clEnqueueNDRangeKernel( commands, dists_kernel, 1, NULL, global_size, NULL, 0, NULL, NULL );
    
        int read_size = response_buf_size;
        if ( offset + read_size > total_size ) read_size = total_size - offset;
        
//        printf( "range: %d to %d\n", offset, offset+read_size-1 );
        
        err = clEnqueueReadBuffer( commands, response_buf_mem, CL_TRUE, 0, sizeof(unsigned int)*read_size, stored_distsqs+offset, 0, NULL, NULL );
    }
    
    clReleaseMemObject( queries_mem );
    delete [] float_queries;
}
#endif

/*
 void findnn( unsigned char *query, size_t &neighbor, unsigned int &distance_sq )
 {
 compute_distances_CL( query );
 
 neighbor = 0;
 unsigned int best_distsq = stored_distsq[0];
 
 for ( int i = 1; i < N; i++ )
 {
 unsigned int distsq = stored_distsq[i];
 if ( distsq < best_distsq ) {
 best_distsq = distsq;
 neighbor = i;
 }
 }
 
 distance_sq = best_distsq;
 }
 */


void BruteForceNN::findconsistentnn( int num_queries, unsigned char *queries, int *neighbors, unsigned int *distances_sq )
{
    unsigned int *stored_distsqs = new unsigned int[num_queries*N];
    
    getDistances( num_queries, queries, stored_distsqs );
    
    int *back_neighbors = new int[N];

    for ( int i = 0; i < N; i++ ) {
        int neighbor = 0;
        unsigned int best_distsq = 255*255*128+1;
        
        for ( int k = 0; k < num_queries; k++ )
        {
            unsigned int distsq = stored_distsqs[k*N+i];
            if ( distsq < best_distsq ) {
                best_distsq = distsq;
                neighbor = k;
            }
        }
        
        back_neighbors[i] = neighbor;
    }

    for ( int k = 0; k < num_queries; k++ ) {
        int neighbor = 0;
        unsigned int best_distsq = 255*255*128+1;
        
        for ( int i = 0; i < N; i++ )
        {
            unsigned int distsq = stored_distsqs[k*N+i];
            if ( distsq < best_distsq ) {
                best_distsq = distsq;
                neighbor = i;
            }
        }
        
        if ( back_neighbors[neighbor] == k ) {
            distances_sq[k] = best_distsq;
            neighbors[k] = neighbor;
        } else {
            neighbors[k] = -1;
        }
    }
    
    delete [] back_neighbors;
    delete [] stored_distsqs;
}


void BruteForceNN::findnn( int num_queries, unsigned char *queries, int *neighbors, unsigned int *distances_sq )
{
    unsigned int *stored_distsqs = new unsigned int[num_queries*N];
    
    getDistances( num_queries, queries, stored_distsqs );
    
    unsigned int *stored_distsq_ptr = stored_distsqs;
    for ( int k = 0; k < num_queries; k++,stored_distsq_ptr+=N ) {
        int neighbor = 0;
        unsigned int best_distsq = 255*255*128+1;
        
        for ( int i = 0; i < N; i++ )
        {
            unsigned int distsq = stored_distsq_ptr[i];
            if ( distsq < best_distsq ) {
                best_distsq = distsq;
                neighbor = i;
            }
        }
        
        distances_sq[k] = best_distsq;
        neighbors[k] = neighbor;
    }
    
    delete [] stored_distsqs;
}

/*
 int findknn( unsigned char *query, int k, size_t *neighbors, unsigned int *distances_sq )
 {
 compute_distances_CL( query );
 
 for ( int i = 0; i < k; i++ )
 {
 neighbors[i] = 0;
 distances_sq[i] = 255*255*128;
 }
 
 for ( int i = 0; i < N; i++ )
 {
 unsigned int distsq = stored_distsq[i];
 
 for ( int j = 0; j < k; j++ )
 {
 if ( distsq < distances_sq[j] ) {
 
 for ( int K = k-1; K > j; K-- )
 {
 distances_sq[K] = distances_sq[K-1];
 neighbors[K] = neighbors[K-1];
 }
 
 distances_sq[j] = distsq;
 neighbors[j] = i;
 
 break;
 }
 }
 }
 
 return k;
 }
 */

void BruteForceNN::findknn( int num_queries, unsigned char *queries, int k, int *neighbors, unsigned int *distances_sq )
{
    unsigned int *stored_distsqs = new unsigned int[num_queries*N];
    
    //compute_distances( num_queries, queries, stored_distsqs );
    getDistances( num_queries, queries, stored_distsqs );
    
    int *neighbors_ptr = neighbors;
    unsigned int *distances_ptr = distances_sq;
    unsigned int *stored_distsq_ptr = stored_distsqs;
    
    for ( int m = 0; m < num_queries; m++,neighbors_ptr+=k,distances_ptr+=k,stored_distsq_ptr+=N )
    {
        for ( int j = 0; j < k; j++ )
        {
            neighbors_ptr[j] = 0;
            distances_ptr[j] = 255*255*128+1;
        }
        
        for ( int i = 0; i < N; i++ )
        {
            unsigned int distsq = stored_distsq_ptr[i];
            
            for ( int j = 0; j < k; j++ )
            {
                if ( distsq < distances_ptr[j] ) {
                    
                    for ( int K = k-1; K > j; K-- )
                    {
                        distances_ptr[K] = distances_ptr[K-1];
                        neighbors_ptr[K] = neighbors_ptr[K-1];
                    }
                    
                    distances_ptr[j] = distsq;
                    neighbors_ptr[j] = i;
                    
                    break;
                }
            }
        }
    }
    
    delete [] stored_distsqs;
}
    
}
