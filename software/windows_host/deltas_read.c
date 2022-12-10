// This module is for reading deltas (MFM bit transition delta times) from the 
// PRU and optionally writing them to a file. The deltas are also available 
// through the pointer returned by deltas setup. The deltas are 16 bit unsigned
// values with each word the time difference between the current rising edge and
// the previous rising edge of the MFM data signal in 200 MHz clock
// counts.
//
// The PRU puts the deltas in the shared DDR memory. These
// routines copy them to normal DDR memory. The shared DDR memory is uncached
// so slow to access. Copying is a little faster.
// A thread is used to read the data from the PRU and update the available delta
// count
//
// Call deltas_setup once to setup the process.
// Call deltas_start_thread to start the reader thread and optionally start
//   writing delta data to filedeltas_get_count
// Call deltas_start_read after each read command is sent to the PRU.
// Call deltas_get_count to get the number of deltas available
// Call deltas_wait_read_finished to wait until all deltas are received
// Call deltas_stop_thread when done with the delta thread
//
// 06/27/2015 DJG Made CMD_STATUS_READ_OVERRUN a warning instead of fatal error
// 05/16/2015 DJG Changes for deltas_read_file.c
// 01/04/2015 DJG Changes for start_time_ns
// 11/09/2014 DJG Changes for new note option
//
// Copyright 2014 David Gesswein.
// This file is part of MFM disk utilities.
//
// MFM disk utilities is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// MFM disk utilities is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with MFM disk utilities.  If not, see <http://www.gnu.org/licenses/>.
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <inttypes.h>
//#include <semaphore.h>
//#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "msg.h"
#include "crc_ecc.h"
#include "emu_tran_file.h"
#include "mfm_decoder.h"
#include "cmd.h"
#include "pru_setup.h"
#include "deltas_read.h"
#include "drive.h"
#include "CSerialPort.h"

static void *delta_proc(void *arg);

// Pointer to delta memory
//static uint16_t *deltas;
// The current cylinder and head processing. Used when writing transitions to file
static int deltas_cyl, deltas_head;
// State of thread
static volatile enum {
   THREAD_NOT_CREATED,
   THREAD_SHUTDOWN,
   THREAD_RUNNING
} thread_state = THREAD_NOT_CREATED;

// Number of deltas in buffer
static int num_deltas;
// Deltas are being received from read thread
static int streaming;

// Allocate the memory to hold the deltas and initialize the semaphore.
// Call once before calling other routines.
//
// ddr_mem_size: Size of ddr shared memory in bytes
void *deltas_setup(int ddr_mem_size) {
   //return deltas;
	return NULL;
}

// Start the delta thread.
//
// drive_params: NULL if no transition data should be written
void deltas_start_thread(DRIVE_PARAMS *drive_params)
{
}

// Routine to stop the delta thread
void deltas_stop_thread()
{
}


// Call to start reading the deltas after starting the PRU CMD_READ_TRACK.
// It may also be called again if it is desired to reprocess the same deltas.
//
// cyl, head: Track being read
void deltas_start_read(int cyl, int head, uint16_t deltas[]) {
uint8_t tmp_deltas[131072L];

   printf("deltas_start_read cyl=%u, head=%u\r\n",cyl,head);
   deltas_cyl = cyl;
   deltas_head = head;

	  TXRXBuf[0] = SCMD_REQ; //request
	  TXRXBuf[1] = SCMD_READ_TRACK;
	  TXRXBuf[2] = Crc8_Dallas(TXRXBuf,2);
#ifdef UDBG
	  printf("-->%x %x %x %x\r\n",TXRXBuf[0],TXRXBuf[1],TXRXBuf[2],TXRXBuf[3]);
#endif
	  for (int i = 0; i < 5; i++)
	  	if (SendData(com_port,3,(char*)TXRXBuf) == 3)
			break;
#ifdef UDBG
	  printf("SCMD_READ_TRACK\r\n");
#endif
	  ReciveData(com_port,(char*)TXRXBuf,4);
#ifdef UDBG
	  printf("<--%x %x %x %x\r\n",TXRXBuf[0],TXRXBuf[1],TXRXBuf[2],TXRXBuf[3]);
#endif
	  if ((TXRXBuf[0] == SCMD_ANSW) && (TXRXBuf[1] == SCMD_READ_TRACK) && (TXRXBuf[3] == Crc8_Dallas(TXRXBuf,3)))
		{
	  		//ReciveData(com_port,(char*)deltas,(131072L-4700L)*2);
	  		ReciveData(com_port,(char*)tmp_deltas,131072L);
#ifdef UDBG
			printf("TIMER_ERRORS = %u\r\n",TXRXBuf[2]);
#endif
			for (int i = 0; i < 131072L; i++)
			deltas[i] = tmp_deltas[i];
		}
	  else
 		printf("ERROR CRC!!!!\r\n");
}

// Write deltas to file. Don't call if transitions_file is -1.
//
// deltas: delta data to write
// num_deltas: number of deltas to write in words
static void write_deltas(int fd, uint16_t deltas[], int num_deltas) {

   tran_file_write_track_deltas(fd, deltas, num_deltas, deltas_cyl, deltas_head);
}

// This is the thread for processing deltas. 
// We read a tracks worth of delta from PRU into global deltas then
// wait for semaphore to repeat. If global thread_state is
// THREAD_SHUTDOWN then we exit.
//
// arg: pointer to drive_params if output files should be created or NULL
//    to not create a file
static void *delta_proc(void *arg)
{
   return NULL;
}

// Update our count of deltas. Streaming indicates we are reading data from
// PRU as it comes in. Streaming is set to zero after all data is read from
// the PRU.
//
// num_deltas_in: Total number of deltas read so far
// streaming_in: 1 if data is being read from PRU. 0 when all data read.
void deltas_update_count(int num_deltas_in, int streaming_in)
{
   num_deltas = num_deltas_in;
   streaming = streaming_in;
}

// Get the delta count. While streaming we return the number of deltas. When
// we are done streaming and cur_deltas is >= num_deltas we return -1 to
// indicate to caller it has processed all of the deltas.
//
// cur_delta: Number of deltas processed by caller
// return: Number of deltas read or -1 if no more deltas
int deltas_get_count(int deltas_processed)
{
   return (131072L-4700L);
}

// Wait until all deltas received. This is used when writing the deltas to a
// file but not decoding. The wait ensures the deltas are written before
// staring the next read.
//
// return: number of deltas read
int deltas_wait_read_finished()
{
   printf("deltas_wait_read_finished\r\n");
   return 0;
}
