// Copyright(c) 2017, Intel Corporation
//
// Redistribution  and  use  in source  and  binary  forms,  with  or  without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of  source code  must retain the  above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name  of Intel Corporation  nor the names of its contributors
//   may be used to  endorse or promote  products derived  from this  software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,  BUT NOT LIMITED TO,  THE
// IMPLIED WARRANTIES OF  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED.  IN NO EVENT  SHALL THE COPYRIGHT OWNER  OR CONTRIBUTORS BE
// LIABLE  FOR  ANY  DIRECT,  INDIRECT,  INCIDENTAL,  SPECIAL,  EXEMPLARY,  OR
// CONSEQUENTIAL  DAMAGES  (INCLUDING,  BUT  NOT LIMITED  TO,  PROCUREMENT  OF
// SUBSTITUTE GOODS OR SERVICES;  LOSS OF USE,  DATA, OR PROFITS;  OR BUSINESS
// INTERRUPTION)  HOWEVER CAUSED  AND ON ANY THEORY  OF LIABILITY,  WHETHER IN
// CONTRACT,  STRICT LIABILITY,  OR TORT  (INCLUDING NEGLIGENCE  OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,  EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <string.h>
#include <uuid/uuid.h>
#include <opae/fpga.h>
#include <time.h>
#include <sys/mman.h>
#include <stdbool.h>
#ifndef USE_ASE
#include <hwloc.h>
#endif
#include "fpga_dma.h"
#include <unistd.h>
#include <ctype.h>

/**
 * \fpga_dma_test.c
 * \brief User-mode DMA test
 */

#include <stdlib.h>
#include <assert.h>

#define HELLO_AFU_ID              "331DB30C-9885-41EA-9081-F88B8F655CAA"
#define TEST_BUF_SIZE 		(10*1024*1024)
#define ASE_TEST_BUF_SIZE	(1024*1024*1024)
uint64_t MAX_SIZE =			2147483648;
#define SW_VERSION			1.31
#define FPGA_VERSION		0x200B8

#ifdef CHECK_DELAYS
extern double poll_wait_count;
extern double buf_full_count;
char cbuf[2048];
#endif

static char *verify_buf = NULL;
static uint64_t verify_buf_size = 0;

static int err_cnt = 0;

// Options determining various optimization attempts
bool use_malloc = true;
bool use_memcpy = true;
bool use_advise = false;
bool do_not_verify = false;
bool cpu_affinity = true;
bool memory_affinity = true;

#define FPGA_MEM_ADDR_DATA0		0x100000000
//#define DATA_SIZE			8
#define DATA_BYTE_LENGTH	2

int HostToFPGA = 1;
int FPGAToHost = 1;
int FPGAToFPGA = 0;
uint64_t TestSize = 4*1024;
uint64_t DMAChunkSize = 4*1024;
uint64_t ErrFreq = 0;
int DMAMaxBuff = 8;
int Poll_Intr = true;
int Magic = true;
uint32_t use_ase;

int num_of_test = 0;
uint64_t testSizeArr[22] = {0x100, 0x400, 0x800, 
							0x1000, 0x4000, 0x8000,
							0x10000, 0x40000, 0x80000,
							0x100000, 0x400000, 0x800000,
							0x1000000, 0x4000000, 0x8000000,
							0x10000000, 0x40000000, 0x80000000,
							0x100000000, 0x400000000, 0x800000000,
							0x0	};
uint64_t ErrFreqArr[22] = {	0x1, 0x1, 0x1,
							0x1, 0x1, 0x1,
							0x1, 0x1, 0x1,
							0xF, 0xF, 0xF,
							0xFF, 0xFF, 0xFF,
							0xFFFF, 0xFFFF, 0xFFFF,
							0xFFFF, 0xFFFF, 0xFFFF,
							0x0	};

typedef struct 
{
	uint64_t tSize;
	uint64_t tSizeRec;
	uint64_t errFreq;
	bool res;
	char msg[50];
}testResult;
								
/*
 * macro for checking return codes
 */
#define ON_ERR_GOTO(res, label, desc)\
  do {\
    if ((res) != FPGA_OK) {\
      err_cnt++;\
      fprintf(stderr, "Error %s: %s\n", (desc), fpgaErrStr(res));\
      goto label;\
    }\
  } while (0)

// Aligned malloc
static inline void *malloc_aligned(uint64_t align, size_t size)
{
	assert(align && ((align & (align - 1)) == 0));	// Must be power of 2 and not 0
	assert(align >= 2 * sizeof(void *));
	void *blk = NULL;
	if (use_malloc) {
		blk = malloc(size + align + 2 * sizeof(void *));
	} else {
		align = getpagesize();
		blk =
		    mmap(NULL, size + align + 2 * sizeof(void *),
			 PROT_READ | PROT_WRITE,
			 MAP_SHARED | MAP_ANONYMOUS | MAP_POPULATE, 0, 0);
	}
	void **aptr =
	    (void **)(((uint64_t) blk + 2 * sizeof(void *) + (align - 1)) &
		      ~(align - 1));
	aptr[-1] = blk;
	aptr[-2] = (void *)(size + align + 2 * sizeof(void *));
	return aptr;
}

// Aligned free
static inline void free_aligned(void *ptr)
{
	void **aptr = (void **)ptr;
	if (use_malloc) {
		free(aptr[-1]);
	} else {
		munmap(aptr[-1], (size_t) aptr[-2]);
	}
	return;
}

static inline void fill_buffer(char *buf, size_t size)
{
	if (do_not_verify)
		return;
	size_t i = 0;

	if (verify_buf_size < size) {
		free(verify_buf);
		verify_buf = (char *)malloc(size);
		verify_buf_size = size;
		char *buf = verify_buf;

		// use a deterministic seed to generate pseudo-random numbers
		srand(99);

		for (i = 0; i < size; i++) {
			*buf = rand() % 256;
			buf++;
		}
	}

	memcpy(buf, verify_buf, size);
}

static inline fpga_result verify_buffer(char *buf, size_t size)
{
	if (do_not_verify)
		return FPGA_OK;

	assert(NULL != verify_buf);

	if (!memcmp(buf, verify_buf, size)) {
		printf("Buffer Verification Success!\n");
	} else {
		size_t i, rnum = 0;
		srand(99);

		for (i = 0; i < size; i++) {
			rnum = rand() % 256;
			if ((*buf & 0xFF) != rnum) {
				printf
				    ("Invalid data at %zx Expected = %zx Actual = %x\n",
				     i, rnum, (*buf & 0xFF));
				return FPGA_INVALID_PARAM;
			}
			buf++;
		}
	}

	return FPGA_OK;
}

fpga_result verify_buffer2(char *buf, size_t size) {
   size_t i, rnum=0;
   srand(99);
   int err_cnt = 0;
   for(i=0; i<size/4; i++) {
	  //printf("Value at %d : %x \n", (int)(i), *buf);
      // rnum = rand()%256;
      // rnum = i%256;
      rnum = (i>>0)%256;
      if((*buf&0xFF) != rnum) {
         //printf("Invalid data at %zx Expected = %zx Actual = %x\n",i,rnum,(*buf&0xFF));
         //return FPGA_INVALID_PARAM;
		 err_cnt++;
      }
      buf++;
      rnum = (i>>8)%256;
      if((*buf&0xFF) != rnum)
		 err_cnt++;
      buf++;
      rnum = (i>>16)%256;
      if((*buf&0xFF) != rnum)
		 err_cnt++;
      buf++;
      rnum = (i>>24)%256;
      if((*buf&0xFF) != rnum)
		 err_cnt++;
      buf++;
   }
   if ( err_cnt == 0 )
	  printf("\nBuffer Verification Success!\n");
   else
	  printf("\nHost side error count is : %d\n", err_cnt);
   return FPGA_OK;
}

static inline void clear_buffer(char *buf, size_t size)
{
	if (do_not_verify)
		return;
	memset(buf, 0, size);
}

static inline char *showDelays(char *buf)
{
#ifdef CHECK_DELAYS
	sprintf(buf,
		"Avg per iteration: Poll delays: %g, Descriptor buffer full delays: %g",
		poll_wait_count, buf_full_count);
#else
	buf[0] = '\0';
#endif
	return buf;
}

static inline void report_bandwidth(size_t size, double seconds)
{
	char buf[2048];
	double throughput = (double)size / ((double)seconds * 1000 * 1000);
	printf("\rMeasured bandwidth = %lf Megabytes/sec %s\n", throughput,
	       showDelays(buf));

#ifdef CHECK_DELAYS
	poll_wait_count = 0;
	buf_full_count = 0;
#endif
}

// return elapsed time
static inline double getTime(struct timespec start, struct timespec end)
{
	uint64_t diff =
	    1000000000L * (end.tv_sec - start.tv_sec) + end.tv_nsec -
	    start.tv_nsec;
	return (double)diff / (double)1000000000L;
}

fpga_result ddr_sweep(fpga_dma_handle dma_h, uint64_t ptr_align,
		      uint64_t siz_align)
{
	int res;
	struct timespec start, end;

	ssize_t total_mem_size =
	    (uint64_t) (4 * 1024) * (uint64_t) (1024 * 1024);

	uint64_t *dma_buf_ptr = malloc_aligned(getpagesize(), total_mem_size);
	if (dma_buf_ptr == NULL) {
		printf("Unable to allocate %ld bytes of memory",
		       total_mem_size);
		return FPGA_NO_MEMORY;
	}

	if (use_advise) {
		if (0 != madvise(dma_buf_ptr, total_mem_size, MADV_SEQUENTIAL))
			perror("Warning: madvise returned error");
	}

	uint64_t *buf_to_free_ptr = dma_buf_ptr;
	dma_buf_ptr = (uint64_t *) ((uint64_t) dma_buf_ptr + ptr_align);
	total_mem_size -= ptr_align + siz_align;

	printf("Buffer pointer = %p, size = 0x%lx (%p through %p)\n",
	       dma_buf_ptr, total_mem_size, dma_buf_ptr,
	       (uint64_t *) ((uint64_t) dma_buf_ptr + total_mem_size));

	printf("Allocated test buffer\n");
	printf("Fill test buffer\n");
	fill_buffer((char *)dma_buf_ptr, total_mem_size);

	uint64_t src = (uint64_t) dma_buf_ptr;
	uint64_t dst = 0x0;

	double tot_time = 0.0;
	int i;

	printf("DDR Sweep Host to FPGA\n");

#define ITERS 32

#ifdef CHECK_DELAYS
	poll_wait_count = 0;
	buf_full_count = 0;
#endif

	for (i = 0; i < ITERS; i++) {
		clock_gettime(CLOCK_MONOTONIC, &start);
		res =
		    fpgaDmaTransferSync(dma_h, dst, src, total_mem_size,
					HOST_TO_FPGA_MM);
		clock_gettime(CLOCK_MONOTONIC, &end);
		if (res != FPGA_OK) {
			printf
			    (" fpgaDmaTransferSync Host to FPGA failed with error %s",
			     fpgaErrStr(res));
			free_aligned(buf_to_free_ptr);
			return FPGA_EXCEPTION;
		}
		tot_time += getTime(start, end);
	}

#ifdef CHECK_DELAYS
	poll_wait_count /= (double)ITERS;
	buf_full_count /= (double)ITERS;
#endif

	report_bandwidth(total_mem_size * ITERS, tot_time);
	tot_time = 0.0;

	printf("\rClear buffer\n");
	clear_buffer((char *)dma_buf_ptr, total_mem_size);

	src = 0x0;
	dst = (uint64_t) dma_buf_ptr;

	printf("DDR Sweep FPGA to Host\n");

#ifdef CHECK_DELAYS
	poll_wait_count = 0;
	buf_full_count = 0;
#endif

	for (i = 0; i < ITERS; i++) {
		clock_gettime(CLOCK_MONOTONIC, &start);
		res =
		    fpgaDmaTransferSync(dma_h, dst, src, total_mem_size,
					FPGA_TO_HOST_MM);
		clock_gettime(CLOCK_MONOTONIC, &end);

		if (res != FPGA_OK) {
			printf
			    (" fpgaDmaTransferSync FPGA to Host failed with error %s",
			     fpgaErrStr(res));
			free_aligned(buf_to_free_ptr);
			return FPGA_EXCEPTION;
		}
		tot_time += getTime(start, end);
	}

#ifdef CHECK_DELAYS
	poll_wait_count /= (double)ITERS;
	buf_full_count /= (double)ITERS;
#endif

	report_bandwidth(total_mem_size * ITERS, tot_time);
	tot_time = 0.0;

	printf("Verifying buffer..\n");
	verify_buffer((char *)dma_buf_ptr, total_mem_size);

	free_aligned(buf_to_free_ptr);
	return FPGA_OK;
}

static void usage(void)
{
	printf
	    ("Usage: fpga_dma_test <use_ase = 1 (simulation only), 0 (hardware)> [options]\n");
	printf("Options are:\n");
	printf("\t-m\tUse malloc (default)\n");
	printf("\t-p\tUse mmap (Incompatible with -m)\n");
	printf("\t-c\tUse builtin memcpy (default)\n");
	printf("\t-2\tUse SSE2 memcpy (Incompatible with -c)\n");
	printf("\t-n\tDo not provide OS advice (default)\n");
	printf("\t-a\tUse madvise (Incompatible with -n)\n");
	printf
	    ("\t-y\tDo not verify buffer contents - faster (default is to verify)\n");
	printf
	    ("\t-C\tDo not restrict process to CPUs attached to DCP NUMA node\n");
	printf
	    ("\t-M\tDo not restrict process memory allocation to DCP NUMA node\n");
}

int main(int argc, char *argv[])
{
	fpga_result res = FPGA_OK;
	fpga_dma_handle dma_h;
	uint64_t count;
	fpga_properties filter = NULL;
	fpga_token afc_token;
	fpga_handle afc_h;
	fpga_guid guid;
	uint32_t num_matches;
	volatile uint64_t *mmio_ptr = NULL;
	uint64_t *dma_buf_ptr = NULL;
	//uint32_t use_ase;
	int param;
	use_ase = 0;
	debugPrint = 0;

	if (argc < 2) {
		usage();
		return 1;
	}

	if (!isdigit(*argv[1])) {
		usage();
		return 1;
	}

	param = atoi(argv[1]);
	if ( param == 0 )
	{
		use_ase = 0;
		debugPrint = 0;
	}
	else if ( param == 1 )
	{
		use_ase = 1;
		debugPrint = 0;
	}
	else if ( param == 2 )
	{
		use_ase = 0;
		debugPrint = 1;
	}
	else if ( param == 3 )
	{
		use_ase = 1;
		debugPrint = 1;
	}
	
	if (use_ase) {
		printf("Running test in ASE mode\n");
	} else {
		printf("Running test in HW mode\n");
	}

	int x;
	for (x = 2; x < argc; x++) {
		char *str = argv[x];
		if (str[0] != '-') {
			usage();
			return 1;
		}

		switch (str[1]) {
		case 'm':
			use_malloc = true;
			break;
		case 'p':
			use_malloc = false;
			break;
		case 'c':
			use_memcpy = true;
			break;
		case '2':
			use_memcpy = false;
			break;
		case 'n':
			use_advise = false;
			break;
		case 'a':
			use_advise = true;
			break;
		case 'y':
			do_not_verify = true;
			break;
		case 'C':
			cpu_affinity = true;
			break;
		case 'M':
			memory_affinity = true;
			break;
		default:
			return 1;
		}
	}

	// enumerate the afc
	if (uuid_parse(HELLO_AFU_ID, guid) < 0) {
		return 1;
	}

	res = fpgaGetProperties(NULL, &filter);
	ON_ERR_GOTO(res, out, "fpgaGetProperties");

	res = fpgaPropertiesSetObjectType(filter, FPGA_ACCELERATOR);
	ON_ERR_GOTO(res, out_destroy_prop, "fpgaPropertiesSetObjectType");

	res = fpgaPropertiesSetGUID(filter, guid);
	ON_ERR_GOTO(res, out_destroy_prop, "fpgaPropertiesSetGUID");

	res = fpgaEnumerate(&filter, 1, &afc_token, 1, &num_matches);
	ON_ERR_GOTO(res, out_destroy_prop, "fpgaEnumerate");

	if (num_matches < 1) {
		printf("Error: Number of matches < 1");
		ON_ERR_GOTO(FPGA_INVALID_PARAM, out_destroy_prop,
			    "num_matches<1");
	}
	// open the AFC
	res = fpgaOpen(afc_token, &afc_h, 0);
	ON_ERR_GOTO(res, out_destroy_tok, "fpgaOpen");

#ifndef USE_ASE
	// Set up proper affinity if requested
	if (cpu_affinity || memory_affinity) {
		unsigned dom = 0, bus = 0, dev = 0, func = 0;
		fpga_properties props;
		int retval;
#ifdef FPGA_DMA_DEBUG
		char str[4096];
#endif
		res = fpgaGetProperties(afc_token, &props);
		ON_ERR_GOTO(res, out_destroy_tok, "fpgaGetProperties");
		res = fpgaPropertiesGetBus(props, (uint8_t *) & bus);
		ON_ERR_GOTO(res, out_destroy_tok, "fpgaPropertiesGetBus");
		res = fpgaPropertiesGetDevice(props, (uint8_t *) & dev);
		ON_ERR_GOTO(res, out_destroy_tok, "fpgaPropertiesGetDevice");
		res = fpgaPropertiesGetFunction(props, (uint8_t *) & func);
		ON_ERR_GOTO(res, out_destroy_tok, "fpgaPropertiesGetFunction");

		// Find the device from the topology
		hwloc_topology_t topology;
		hwloc_topology_init(&topology);
		hwloc_topology_set_flags(topology,
					 HWLOC_TOPOLOGY_FLAG_IO_DEVICES);
		hwloc_topology_load(topology);
		hwloc_obj_t obj =
		    hwloc_get_pcidev_by_busid(topology, dom, bus, dev, func);
		hwloc_obj_t obj2 = hwloc_get_non_io_ancestor_obj(topology, obj);
#ifdef FPGA_DMA_DEBUG
		hwloc_obj_type_snprintf(str, 4096, obj2, 1);
		printf("%s\n", str);
		hwloc_obj_attr_snprintf(str, 4096, obj2, " :: ", 1);
		printf("%s\n", str);
		hwloc_bitmap_taskset_snprintf(str, 4096, obj2->cpuset);
		printf("CPUSET is %s\n", str);
		hwloc_bitmap_taskset_snprintf(str, 4096, obj2->nodeset);
		printf("NODESET is %s\n", str);
#endif
		if (memory_affinity) {
#if HWLOC_API_VERSION > 0x00020000
			retval =
			    hwloc_set_membind(topology, obj2->nodeset,
					      HWLOC_MEMBIND_THREAD,
					      HWLOC_MEMBIND_MIGRATE |
					      HWLOC_MEMBIND_BYNODESET);
#else
			retval =
			    hwloc_set_membind_nodeset(topology, obj2->nodeset,
						      HWLOC_MEMBIND_THREAD,
						      HWLOC_MEMBIND_MIGRATE);
#endif
			ON_ERR_GOTO(retval, out_destroy_tok,
				    "hwloc_set_membind");
		}
		if (cpu_affinity) {
			retval =
			    hwloc_set_cpubind(topology, obj2->cpuset,
					      HWLOC_CPUBIND_STRICT);
			ON_ERR_GOTO(retval, out_destroy_tok,
				    "hwloc_set_cpubind");
		}
	}
#endif

	if (!use_ase) {
		res = fpgaMapMMIO(afc_h, 0, (uint64_t **) & mmio_ptr);
		ON_ERR_GOTO(res, out_close, "fpgaMapMMIO");
	}
	// reset AFC
	res = fpgaReset(afc_h);
	ON_ERR_GOTO(res, out_unmap, "fpgaReset");

	res = fpgaDmaOpen(afc_h, &dma_h, use_ase);
	ON_ERR_GOTO(res, out_dma_close, "fpgaDmaOpen");
	if (!dma_h) {
		res = FPGA_EXCEPTION;
		ON_ERR_GOTO(res, out_dma_close, "Invaid DMA Handle");
	}

	if (use_ase)
		count = ASE_TEST_BUF_SIZE;
	else
		count = TEST_BUF_SIZE;
	
	//char ch = 'y';
	int option = -1, out, timeoutTime;	// dataCtr=0;
	uint64_t offset=0x0000, value=0x0000, total_fpga_err=0;
	struct timespec tstart={0,0}, tend={0,0};
	bool compare = false, filled = false, lastVal = false;
	uint64_t num=0, tSize=0;
	
	value = RegRead(dma_h, FPGA_VERSION);
	uint64_t vMajor = (value >> 16) & 0xFF;
	uint64_t vMinor = (value >> 0) & 0xFF;
	printf("\n\t\tWelcome to DMA Test Application\n\tApplication using ASE = %d\n\tSW Version = %.02f\n\tFPGA Version = %d.%d", use_ase, SW_VERSION, (int)vMajor, (int)vMinor ); 
						
	do
	{
		printf("\n\n###################################################");
		printf("\n1. Register Read");
		printf("\n2. Register Write");
		printf("\n3. Write Data from Host to FPGA");
		printf("\n4. Read Data from FPGA to Host");		
		printf("\n5. Generate and Read Pattern from FPGA");
		printf("\n6. Generate data at PPE and Receive in HOST");
		printf("\n7. Run Pre-defined Test Cases");
		printf("\n0. Exit");
		
		total_fpga_err = 0;
		//ReadInputFromFile();
		//return 0;
		printf("\n\nEnter choice : ");
		out = scanf ("%d", &option);
		
		switch(option)
		{
			case 1 :	lastVal = debugPrint;
						debugPrint = true;
						printf("\nEnter offset/address (0x00) : ");
						out = scanf("%lx", &offset);
						value = RegRead(dma_h, offset);
						debugPrint = lastVal;
						break;
						
			case 2 :	lastVal = debugPrint;
						debugPrint = true;
						printf("\nEnter offset/address (0x00) : ");
						out = scanf("%lx", &offset);
						printf("\nEnter value (0x00) : ");
						out = scanf("%lx", &value);
						res = RegWrite(dma_h, offset, value);
						debugPrint = lastVal;
						break;
						
			case 3 :	//printf("\nEnter dataSize : ");
						//out = scanf("%" PRIu64, &TestSize);
						//printf("\nEnter DMAChunkSize : ");
						//out = scanf("%" PRIu64, &DMAChunkSize);
						//initializeVals(TestSize, DMAChunkSize, DMAMaxBuff, Poll_Intr, Magic);
						//count = TestSize;
						dma_buf_ptr = (uint64_t*)malloc(count);
						if(!dma_buf_ptr) {
						  res = FPGA_NO_MEMORY;
						  ON_ERR_GOTO(res, out_dma_close, "Error allocating memory");
						}   
						printf("\nTesting for %ld bytes\n", count);
						fill_buffer((char*)dma_buf_ptr, count);
						filled = true;
						if (debugPrint)	printf("\n--- Copy from host to fpga START");
						clock_gettime(CLOCK_MONOTONIC, &tstart);
						res = fpgaDmaTransferSync(dma_h, 0x0, (uint64_t)dma_buf_ptr, count, HOST_TO_FPGA_MM);
						clock_gettime(CLOCK_MONOTONIC, &tend);
						if (debugPrint)	printf("\n--- Ccopy from host to fpga END ---");
						printf("\ncopy from host to fpga took about %.09lf seconds\n",
							   ((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) - ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec));
						ON_ERR_GOTO(res, out_dma_close, "fpgaDmaTransferSync HOST_TO_FPGA_MM");   
						clear_buffer((char*)dma_buf_ptr, count);
						//free(dma_buf_ptr);
						compare = true;
						break;
						
			case 4 :	//printf("\nEnter dataSize : ");
						//out = scanf("%" PRIu64, &TestSize);
						//printf("\nEnter DMAChunkSize : ");
						//out = scanf("%" PRIu64, &DMAChunkSize);
						//initializeVals(TestSize, DMAChunkSize, DMAMaxBuff, Poll_Intr, Magic);
						//count = TestSize;
						dma_buf_ptr = (uint64_t*)malloc(count);
						if(!dma_buf_ptr) {
						  res = FPGA_NO_MEMORY;
						  ON_ERR_GOTO(res, out_dma_close, "Error allocating memory");
						}   
						if ( compare )
							clear_buffer((char*)dma_buf_ptr, count);
						printf("\nTesting for %ld bytes\n", count);
						if (debugPrint)	printf("\n--- Copy from fpga to host START ---");
						clock_gettime(CLOCK_MONOTONIC, &tstart);
						res = fpgaDmaTransferSync(dma_h, (uint64_t)dma_buf_ptr, 0x0, count, FPGA_TO_HOST_MM);
						clock_gettime(CLOCK_MONOTONIC, &tend);
						if (debugPrint)	printf("\n--- Copy from fpga to host END ---");
						printf("\ncopy from fpga to host took about %.09lf seconds\n",
							   ((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) - ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec));
						ON_ERR_GOTO(res, out_dma_close, "fpgaDmaTransferSync FPGA_TO_HOST_MM");
						if ( compare )
							res = verify_buffer((char*)dma_buf_ptr, count);
						clear_buffer((char*)dma_buf_ptr, count);
						//free(dma_buf_ptr);
						compare = false;
						break;
			
			case 5 :	printf("\nEnter dataSize to test (0x00) : ");
						out = scanf("%lx", &TestSize);
						tSize = TestSize;
						printf("\nEnter error frequency (0x00) : ");
						out = scanf("%lx", &ErrFreq);
						value = 1;
						if (ErrFreq > 0)
						{							
							ErrFreq = (((ErrFreq & 0xFFFF) << 39) | (value << 38));
							checkErr = true;
						}
						else if (ErrFreq == 0)
							checkErr = false;
						if ( TestSize == 0 )
							checkErr = true;
						else
							checkErr = false;
						// res = RegWrite(dma_h, 0x20098, ErrFreq);
						
						//if (debugPrint)	
							printf("\nTotal Test Size = %ld bytes\n", tSize);
						if ( tSize > MAX_SIZE )
						{
							num = tSize / MAX_SIZE;
							count = MAX_SIZE;
						}
						else if ( tSize == 0 )
						{
							num = 0;
							tSize = count = MAX_SIZE;
						}
						else
						{
							num = 1;
							count = tSize;
						}
						
						dma_buf_ptr = (uint64_t*)malloc(count);
						if (!dma_buf_ptr) {
							res = FPGA_NO_MEMORY;
							ON_ERR_GOTO(res, out_dma_close, "Error allocating memory");
						}
							
						//if (debugPrint)	printf("\n--- Copy from fpga to host START ---");
						clock_gettime(CLOCK_MONOTONIC, &tstart);
						for ( int itr = 0; itr <= num; itr++ )
						{
							if ( num == 0)
								itr--;
							else{
							if ( itr * MAX_SIZE >= tSize )
							{	
								//if (debugPrint) printf("\nBreaking....Total size tested");
								break;
							}
							if ( itr == num )
								count = tSize - (itr * MAX_SIZE);
							}
							//if (debugPrint) printf("\nTesting for %ld bytes\niteration = %d", count, itr+1);
							res = RegWrite(dma_h, 0x20090, count);
							res = RegWrite(dma_h, 0x20098, (0x100001100000000 | ErrFreq));
							res = RegWrite(dma_h, 0x20098, (0x100000100000000 | ErrFreq));
							
							res = fpgaDmaTransferSync(dma_h, (uint64_t)dma_buf_ptr, FPGA_MEM_ADDR_DATA0, count, FPGA_TO_HOST_MM);
							clock_gettime(CLOCK_MONOTONIC, &tend);
							ON_ERR_GOTO(res, skip, "fpgaDmaTransferSync FPGA_TO_HOST_MM");

							//res = RegWrite(dma_h, 0x20098, 0x0);
							//value = RegRead(dma_h, 0x200A8);
							//printf("\nTotal clock count for write is : %ld\n", value);
							value = RegRead(dma_h, 0x200A0);
							value = ((value>>36)%65536);
							total_fpga_err += value;
							//printf("\nFPGA side error count is : %ld\n", value);
							
							//res = verify_buffer2((char*)dma_buf_ptr, count);
							//ON_ERR_GOTO(res, out_dma_close, "verify_buffer2");
							//printf("\nData Received : ");
							//for(int i=0; i<count; i++)
							//	printf("%lx ", dma_buf_ptr[i]);
							clear_buffer((char*)dma_buf_ptr, count);
							//free(dma_buf_ptr);
							//printf("\n====================\nReceived %ld bytes\n====================", tot_datasize_recv );
							//ch = getchar();
							//if (ch == 's' || ch == 'S')	break;					
						}
					skip:
						//if (debugPrint) printf("\n--- Copy from fpga to host END ---");
						printf("\n========================================\nTotal data received  %ld Bytes = %0.09Lf GBytes\n========================================", tot_datasize_recv, (long double)tot_datasize_recv/(1024*1024*1024) );
						printf("\nTime Taken to transfer fpga to host = %.09lf seconds", ((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) - ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec));
						if ( timedout )
							timeoutTime = FPGA_DMA_TIMEOUT_MSEC;
						else
							timeoutTime = 0;
						printf("\nThroughput = %0.09Lf Bytes per second", (long double)tot_datasize_recv/( ((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) - ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec) - timeoutTime ) );
						res = RegWrite(dma_h, 0x20098, 0x0);
						value = RegRead(dma_h, 0x200A8);
						printf("\nTotal clock count for write is : %ld\n", value*num);
						printf("\nFPGA side error count is       : %ld", total_fpga_err);
						showSwErr();						
						break;
						
			case 6 :	printf("\nEnter dataSize to test (0x00) : ");
						out = scanf("%lx", &TestSize);
						tSize = TestSize;
						printf("\nEnter error frequency (0x00) : ");
						out = scanf("%lx", &ErrFreq);
						value = 1;
						if (ErrFreq > 0)
						{							
							ErrFreq = (((ErrFreq & 0xFFFF) << 39) | (value << 38));
							checkErr = true;
						}
						else if (ErrFreq == 0)
							checkErr = false;
						if ( TestSize == 0 )
							checkErr = true;
						else
							checkErr = false;
						// res = RegWrite(dma_h, 0x20098, ErrFreq);
						
						//if (debugPrint)	
							printf("\nTotal Test Size = %ld bytes\n", tSize);
						if ( tSize > MAX_SIZE )
						{
							num = tSize / MAX_SIZE;
							count = MAX_SIZE;
						}
						else if ( tSize == 0 )
						{
							num = 0;
							tSize = count = MAX_SIZE;
						}
						else
						{
							num = 1;
							count = tSize;
						}
						
						dma_buf_ptr = (uint64_t*)malloc(count);
						if (!dma_buf_ptr) {
							res = FPGA_NO_MEMORY;
							ON_ERR_GOTO(res, out_dma_close, "Error allocating memory");
						}
							
						//if (debugPrint)	printf("\n--- Copy from fpga to host START ---");
						clock_gettime(CLOCK_MONOTONIC, &tstart);
						for ( int itr = 0; itr <= num; itr++ )
						{
							if ( num == 0)
								itr--;
							else{
							if ( itr * MAX_SIZE >= tSize )
							{	
								//if (debugPrint)	printf("\nBreaking....Total size tested");
								break;
							}
							if ( itr == num )
								count = tSize - (itr * MAX_SIZE);
							}
							//if (debugPrint)	printf("\nTesting for %ld bytes\n", count);
							res = RegWrite(dma_h, 0x20090, count);
							res = RegWrite(dma_h, 0x20098, (0x200001100000000 | ErrFreq));
							res = RegWrite(dma_h, 0x20098, (0x200000100000000 | ErrFreq));
							res = RegWrite(dma_h, 0x20100,  0x7000000000040);
							
							res = fpgaDmaTransferSync(dma_h, (uint64_t)dma_buf_ptr, FPGA_MEM_ADDR_DATA0, count, FPGA_TO_HOST_MM);
							clock_gettime(CLOCK_MONOTONIC, &tend);
							ON_ERR_GOTO(res, skip2, "fpgaDmaTransferSync FPGA_TO_HOST_MM");
							
							//res = RegWrite(dma_h, 0x20098, 0x0);
							//value = RegRead(dma_h, 0x200A8);
							//printf("\nTotal clock count for write is : %ld\n", value);
							value = RegRead(dma_h, 0x200A0);
							value = ((value>>36)%65536);
							total_fpga_err += value;
							//printf("\nFPGA side error count is : %ld\n", value);
							
							//res = verify_buffer2((char*)dma_buf_ptr, count);
							//ON_ERR_GOTO(res, out_dma_close, "verify_buffer2");
							//printf("\nData Received : ");
							//for(int i=0; i<count; i++)
							//	printf("%lx ", dma_buf_ptr[i]);
							clear_buffer((char*)dma_buf_ptr, count);
							//free(dma_buf_ptr);
							//printf("\n====================\nReceived %ld bytes\n====================", tot_datasize_recv );
							//ch = getchar();
							//if (ch == 's' || ch == 'S')	break;					
						}
					skip2:
						//if (debugPrint)	printf("\n--- Copy from fpga to host END ---");
						printf("\n========================================\nTotal data received  %ld Bytes = %0.09Lf GBytes\n========================================", tot_datasize_recv, (long double)tot_datasize_recv/(1024*1024*1024) );
						printf("\nTime Taken to transfer fpga to host = %.09lf seconds", ((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) - ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec));
						if ( timedout )
							timeoutTime = FPGA_DMA_TIMEOUT_MSEC;
						else
							timeoutTime = 0;
						printf("\nThroughput = %0.09Lf Bytes per second", (long double)tot_datasize_recv/( ((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) - ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec) - timeoutTime ) );
						res = RegWrite(dma_h, 0x20098, 0x0);
						value = RegRead(dma_h, 0x200A8);
						printf("\nTotal clock count for write is : %ld\n", value*num);
						printf("\nFPGA side error count is       : %ld", total_fpga_err);
						showSwErr();						
						break;
				
			case 7 :	printf("\nTEST CASES :-\ntestSizeArr[22] = {\t0x100, 0x400, 0x800,\n\t\t\t0x1000, 0x4000, 0x8000,\n\t\t\t0x10000, 0x40000, 0x80000,\n\t\t\t0x100000, 0x400000,0x800000,\n\t\t\t0x1000000, 0x4000000, 0x8000000,\n\t\t\t0x10000000, 0x40000000, 0x80000000,\n\t\t\t0x100000000, 0x400000000, 0x800000000,\n\t\t\t0x0	}");
						printf("\nErrFreqArr[22] = {\t0x1, 0x1, 0x1,\n\t\t\t0x1, 0x1, 0x1,\n\t\t\t0x1, 0x1, 0x1,\n\t\t\t0xF, 0xF, 0xF,\n\t\t\t0xFF, 0xFF, 0xFF,\n\t\t\t0xFFFF, 0xFFFF, 0xFFFF,\n\t\t\t0xFFFF, 0xFFFF, 0xFFFF,\n\t\t\t0x0	}\n");
						printf("\n\nEnter number of test to perform (1-22) : ");
						out = scanf("%d", &num_of_test);
						
						if ( num_of_test < 1 || num_of_test > 22 )
						{
							printf("\nWrong choice\t\tEnter again\n\n");
							option = 0;
							break;
						}
						testResult* TR = (testResult*)malloc(sizeof(testResult)*num_of_test); 
						for ( int tc = 0; tc < num_of_test; tc++ )
						{
						total_fpga_err = 0;
						tSize = testSizeArr[tc];
						//printf("\nEnter error frequency (0x00) : ");
						//out = scanf("%lx", &ErrFreq);
						ErrFreq = ErrFreqArr[tc];
						value = 1;
						if (ErrFreq > 0)
						{							
							ErrFreq = (((ErrFreq & 0xFFFF) << 39) | (value << 38));
							checkErr = true;
						}
						else if (ErrFreq == 0)
							checkErr = false;
						if ( TestSize == 0 )
							checkErr = true;
						else
							checkErr = false;
						// res = RegWrite(dma_h, 0x20098, ErrFreq);
						
						printf("\n################################################################################");
						printf("\n\t\tTest Size = %ld bytes = %0.09Lf GBytes", tSize, (long double)tSize/(1024*1024*1024) );
						printf("\n\t\tError Freq (bit-shifted) = %lx", ErrFreq);
						printf("\n################################################################################");
						if ( tSize > MAX_SIZE )
						{
							num = tSize / MAX_SIZE;
							count = MAX_SIZE;
						}
						else if ( tSize == 0 )
						{
							num = 0;
							tSize = count = MAX_SIZE;
						}
						else
						{
							num = 1;
							count = tSize;
						}
						
						dma_buf_ptr = (uint64_t*)malloc(count);
						if (!dma_buf_ptr) {
							res = FPGA_NO_MEMORY;
							ON_ERR_GOTO(res, out_dma_close, "Error allocating memory");
						}
							
						//if (debugPrint)	printf("\n--- Copy from fpga to host START ---");
						clock_gettime(CLOCK_MONOTONIC, &tstart);
						for ( int itr = 0; itr <= num; itr++ )
						{
							if ( num == 0)
								itr--;
							else{
							if ( itr * MAX_SIZE >= tSize )
							{	
								//if (debugPrint)	printf("\nBreaking....Total size tested");
								break;
							}
							if ( itr == num )
								count = tSize - (itr * MAX_SIZE);
							}
							//if (debugPrint)	printf("\nTesting for %ld bytes\niteration = %d", count, itr+1);
							res = RegWrite(dma_h, 0x20090, count);
							res = RegWrite(dma_h, 0x20098, (0x100001100000000 | ErrFreq));
							res = RegWrite(dma_h, 0x20098, (0x100000100000000 | ErrFreq));
							
							res = fpgaDmaTransferSync(dma_h, (uint64_t)dma_buf_ptr, FPGA_MEM_ADDR_DATA0, count, FPGA_TO_HOST_MM);
							clock_gettime(CLOCK_MONOTONIC, &tend);
							ON_ERR_GOTO(res, skip3, "fpgaDmaTransferSync FPGA_TO_HOST_MM");

							//res = RegWrite(dma_h, 0x20098, 0x0);
							//value = RegRead(dma_h, 0x200A8);
							//printf("\nTotal clock count for write is : %ld\n", value);
							value = RegRead(dma_h, 0x200A0);
							value = ((value>>36)%65536);
							total_fpga_err += value;
							//printf("\nFPGA side error count is : %ld\n", value);
							
							//res = verify_buffer2((char*)dma_buf_ptr, count);
							//ON_ERR_GOTO(res, out_dma_close, "verify_buffer2");
							//printf("\nData Received : ");
							//for(int i=0; i<count; i++)
							//	printf("%lx ", dma_buf_ptr[i]);
							clear_buffer((char*)dma_buf_ptr, count);
							//free(dma_buf_ptr);
							//printf("\n====================\nReceived %ld bytes\n====================", tot_datasize_recv );
							//ch = getchar();
							//if (ch == 's' || ch == 'S')	break;					
						}
					skip3:
						//if (debugPrint)	printf("\n--- Copy from fpga to host END ---");
						printf("\n\n===================================================\nTotal data received  %ld Bytes = %0.09Lf GBytes\n===================================================\n", tot_datasize_recv, (long double)tot_datasize_recv/(1024*1024*1024) );
						printf("\nTime Taken to transfer fpga to host = %.09lf seconds", ((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) - ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec));
						if ( timedout )
							timeoutTime = FPGA_DMA_TIMEOUT_MSEC;
						else
							timeoutTime = 0;
						printf("\nThroughput = %0.09Lf Bytes per second", (long double)tot_datasize_recv/( ((double)tend.tv_sec + 1.0e-9*tend.tv_nsec) - ((double)tstart.tv_sec + 1.0e-9*tstart.tv_nsec) - timeoutTime ) );
						res = RegWrite(dma_h, 0x20098, 0x0);
						value = RegRead(dma_h, 0x200A8);
						printf("\nTotal clock count for write is : %ld", value*num);
						printf("\nFPGA side error count is       : %ld", total_fpga_err);
						TR[tc].tSize = tSize;
						TR[tc].tSizeRec = tot_datasize_recv;
						TR[tc].errFreq = ErrFreqArr[tc];
						if ( tot_datasize_recv == tSize && total_fpga_err == tot_sw_err )
						{							
							TR[tc].res = true;
							sprintf( TR[tc].msg, "No issue found" );
						}   
						else
						{   
							TR[tc].res = false;
							if ( timedout )
								sprintf( TR[tc].msg, "Poll Timeout at %ld", tot_datasize_recv );
							else
								sprintf( TR[tc].msg, "Data mis-match around %ld", tot_datasize_recv );
						}
						showSwErr();
						//printf("\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\t<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
						}
						for ( int tc = 0; tc < num_of_test; tc++ )
						{
							printf("\n\n################################################################################\n");
							printf("\n\tTEST CASE %d\n\tTESTED SIZE\t= %ld Bytes = %0.09Lf GBytes\n\tSIZE RECEIVED\t= %ld Bytes = %0.09Lf GBytes\n\tERROR Frequency (0x0000)\t= %lx\n\tINFORMATION\t\t\t= %s\n\tTEST Result (0-FAIL | 1-PASS)\t= %d\n", tc+1, TR[tc].tSize, (long double)TR[tc].tSize/(1024*1024*1024), TR[tc].tSizeRec, (long double)TR[tc].tSizeRec/(1024*1024*1024),TR[tc].errFreq, TR[tc].msg, (int)TR[tc].res );
							printf("\n################################################################################");
						}
						break;
						
			case 0 :	option = 99;
						break;
						
			default :	printf("\nWrong choice\tEnter again\n\n");
						option = 0;
						break;
		}	
						
		if ( option == 0 )
			continue;
		
		//printf("\n\nPress 'y/Y' to continue or 'n/N' to quit the program : ");
		//scanf ("%c", &ch);
	}while ( option != 99 );
	//while ( ch == 'y' || ch == 'Y' );

#if 0
	if (!use_ase) {
		printf("Running DDR sweep test\n");
		res = ddr_sweep(dma_h, 0, 0);
		printf("DDR sweep with unaligned pointer and size\n");
		res |= ddr_sweep(dma_h, 61, 5);
		res |= ddr_sweep(dma_h, 3, 0);
		res |= ddr_sweep(dma_h, 7, 3);
		res |= ddr_sweep(dma_h, 0, 3);
		res |= ddr_sweep(dma_h, 0, 61);
		res |= ddr_sweep(dma_h, 0, 7);
		ON_ERR_GOTO(res, out_dma_close, "ddr_sweep");
	}
#endif
	if ( filled )		free(verify_buf);

 out_dma_close:
	//free_aligned(dma_buf_ptr);
	free(dma_buf_ptr);
	if (dma_h)
		res = fpgaDmaClose(dma_h);
	ON_ERR_GOTO(res, out_unmap, "fpgaDmaClose");

 out_unmap:
	if (!use_ase) {
		res = fpgaUnmapMMIO(afc_h, 0);
		ON_ERR_GOTO(res, out_close, "fpgaUnmapMMIO");
	}
 out_close:
	res = fpgaClose(afc_h);
	ON_ERR_GOTO(res, out_destroy_tok, "fpgaClose");

 out_destroy_tok:
	res = fpgaDestroyToken(&afc_token);
	ON_ERR_GOTO(res, out_destroy_prop, "fpgaDestroyToken");

 out_destroy_prop:
	res = fpgaDestroyProperties(&filter);
	ON_ERR_GOTO(res, out, "fpgaDestroyProperties");

 out:
	return err_cnt;
}