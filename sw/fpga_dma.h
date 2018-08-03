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

/**
 * \fpga_dma.h
 * \brief FPGA DMA BBB API Header
 *
 * Known Limitations
 * - Supports only synchronous (blocking) transfers
 */

#ifndef __FPGA_DMA_H__
#define __FPGA_DMA_H__

#include <opae/fpga.h>

uint64_t tot_sw_err = 0;
uint64_t tot_datasize_recv = 0;
bool checkErr = false;
uint64_t chunkNo = 0;
int debugPrint = 0;
bool timedout = false;

#define FPGA_DMA_TIMEOUT_MSEC (60000)

#ifdef __cplusplus
extern "C" {
#endif

/*
* The DMA driver supports host to FPGA, FPGA to host and FPGA
* to FPGA transfers. The FPGA interface can be streaming
* or memory-mapped. Streaming interfaces are not currently
* supported.
*/
	typedef enum {
		HOST_TO_FPGA_MM = 0,	//Memory mapped FPGA interface
		FPGA_TO_HOST_MM,	//Memory mapped FPGA interface
		FPGA_TO_FPGA_MM,	//Memory mapped FPGA interface
		FPGA_MAX_TRANSFER_TYPE,
	} fpga_dma_transfer_t;

	typedef struct _dma_handle_t *fpga_dma_handle;

// Callback for asynchronous DMA transfers
	typedef void (*fpga_dma_transfer_cb) (void *context);

/**
* fpgaDmaOpen
*
* @brief           Open a handle to DMA BBB.
*                  Scans the device feature chain looking for a DMA BBB.
*
* @param[in]  fpga Handle to the FPGA AFU object obtained via fpgaOpen()
* @param[out] dma  DMA object handle
* @returns         FPGA_OK on success, return code otherwise
*/
	fpga_result fpgaDmaOpen(fpga_handle fpga, fpga_dma_handle * dma, int use_ase);

/**
* fpgaDmaTransferSync
*
* @brief             Perform a blocking copy of 'count' bytes from memory area pointed
*                    by src to memory area pointed by dst where fpga_dma_transfer_t specifies the
*                    type of memory transfer.
* @param[in] dma     Handle to the FPGA DMA object
* @param[in] dst     Address of the destination buffer
* @param[in] src     Address of the source buffer
* @param[in] count   Size in bytes
* @param[in] type    Must be one of the following values:
*                    HOST_TO_FPGA_MM - Copy data from host memory to memory mapped FPGA interface.
*                                      User must specify valid src and dst.
*                    FPGA_TO_HOST_MM - Copy data from memory mapped FPGA interface to host memory
*                                      User must specify valid src and dst.
*                    FPGA_TO_FPGA_MM - Copy data between memory mapped FPGA interfaces
*                                      User must specify valid src and dst.
* @return fpga_result FPGA_OK on success, return code otherwise
*
*/
fpga_result fpgaDmaTransferSync(fpga_dma_handle dma_h, uint64_t dst, uint64_t src, size_t count, fpga_dma_transfer_t type);

/**
* fpgaDmaTransferAsync (Not supported)
*
* @brief             Perform a non-blocking copy of 'count' bytes from memory area pointed
*                    by src to memory area pointed by dst where fpga_dma_transfer_t specifies the
*                    type of memory transfer.
* @param[in] dma     Handle to the FPGA DMA object
* @param[in] dst     Address of the destination buffer
* @param[in] src     Address of the source buffer
* @param[in] count   Size in bytes
* @param[in] type    Must be one of the following values:
*                    HOST_TO_FPGA_MM - Copy data from host memory to memory mapped FPGA interface.
*                                      User must specify valid src and dst.
*                    FPGA_TO_HOST_MM - Copy data from memory mapped FPGA interface to host memory
*                                      User must specify valid src and dst.
*                    FPGA_TO_FPGA_MM - Copy data between memory mapped FPGA interfaces
*                                      User must specify valid src and dst.
* @param[in] cb      Callback to invoke when DMA transfer is complete
* @param[in] context Pointer to define user-defined context
* @return fpga_result FPGA_OK on success, return code otherwise
*
*/
	fpga_result fpgaDmaTransferAsync(fpga_dma_handle dma, uint64_t dst,
					 uint64_t src, size_t count,
					 fpga_dma_transfer_t type,
					 fpga_dma_transfer_cb cb,
					 void *context);

/**
* fpgaDmaClose
*
* @brief           Close the DMA BBB handle.
*
* @param[in] dma   DMA object handle
* @returns         FPGA_OK on success, return code otherwise
*/
	fpga_result fpgaDmaClose(fpga_dma_handle dma);

	
/**
* initializeVals
*
* @brief           
*
* @param[in] dma   
* @returns         
*/
void initializeVals(uint64_t TestSize, uint64_t DMAChunkSize, int DMAMaxBuff, int Poll_Intr, int Magic);

uint64_t RegReadSim(fpga_dma_handle dma_h, uint64_t offset);

fpga_result RegWriteSim(fpga_dma_handle dma_h, uint64_t offset, uint64_t value);

uint64_t RegRead(fpga_dma_handle dma_h, uint64_t offset);

fpga_result RegWrite(fpga_dma_handle dma_h, uint64_t offset, uint64_t value);

void showSwErr();

fpga_result verify_buffer3(char *buf, size_t size, int checkStart);

fpga_result DataStatus(fpga_dma_handle dma_h, bool pause);

void *thread(void *arg);

void checkFPGAStatus(fpga_dma_handle dma_h);

//public:

#ifdef __cplusplus
}
#endif
#endif				// __FPGA_DMA_H__
