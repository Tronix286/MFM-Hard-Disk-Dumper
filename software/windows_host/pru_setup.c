// Routines for interfacing with the PRU/PRUSS. Some of the routines are
// generic and others specific to communicating with the code running on
// the PRU. Only PRU 0 is currently used.
//
// Call pru_setup first to open PRU device and map memory.
// Call pru_shutdown when done to close PRU device.
// These routines are specific to the code running on the PRU.
// Call pru_exec_cmd to send a command to PRU and wait for response.
// Call pru_get_cmd_status to get the command completion status.
// Call pru_get_cmd_data to get data value set by a command.
// Call pru_restart to restart PRU from the restart address.
// Call pru_get_pc to get the program counter of the PRU. For debugging
// Call pru_write_mem to write data to memory
// Call pru_read_mem to read data from memory
// Call pru_read_word to read a 32 bit word from memory
// Call pru_write_word to write a 32 bit word to memory
//
// TODO: Use cache control to make memory transfers faster with PRU
//
// 06/19/19 DJG Added support for 8.5 MHz --rate for Xerox Star and
//    fixed typo in comment
// 04/08/19 DJG Added support for 8.6 MHz --rate for WANG SVP
// 03/14/19 DJG Fix comment
// 06/23/18 DJG Add 8.68 MHz data rate support.
// 05/19/17 DJG Add ability to dump PRU shared memory.
// 12/24/15 DJG Comment cleanup
// 11/22/15 DJG Add 15 MHz data rate support.
// 05/17/15 DJG Added routines to allow dumping of state if PRU halts
//   due to error.
// 01/04/15 DJG Added pru_set_clock to set the PRU clock to generate the
//   desired mfm clock and data bit rate.
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
#include <stdlib.h>
#include <inttypes.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
//#include <sys/mman.h>
#include <errno.h>

//#include <prussdrv.h>
//#include <pruss_intc_mapping.h>

#include "msg.h"
#include "cmd.h"

#include "pru_setup.h"
#define ARRAYSIZE(x) (sizeof(x) / sizeof(x[0]))
#include "CSerialPort.h"


// Map DDR shared memory segment into our address space and return addresses
// and size.
//
// ddrmem: Returns pointer to memory (virtual address).
// ddr_phys_addr: Returns physical address of memory.
// return: Size of region in bytes.
static int pru_allocate_ddr_memory(void **ddrmem, uint32_t *ddr_phys_addr)
{
   return (0);
}

// Set up PRU device and map memory
//
// num_pru_in: Number of PRU to setup, 1 or 2
// return: size of DDR memory segment in bytes.
int  pru_setup(int num_pru_in)
{
   return 0;
}

// Wait for completion from PRU and shut down device.
void pru_shutdown() {
}


// This sends a command and data word to the PRU and waits for completion or,
// for reads, that the read has started. To execute a command first write the
// data value if needed then write the command to the command address. The PRU
// will execute the command and update the command address with the command
// status. The read command which we wish to overlap processing data with
// reading returns a command started status and then a command done status.
// See cmd.h for commands.
//
// cmd: Command to execute.
// data: Data value for command.
// return: Zero if no error, command status if error
int pru_exec_cmd(uint32_t cmd, uint32_t data)
{
   return 0;
}

// Return the command status value. See cmd.h for values.
uint32_t pru_get_cmd_status(void) {
   return 0;   
}

// Return the command data value. Not all commands return data.
uint32_t pru_get_cmd_data(void) {
   return 0;
}

// Call this routine to restart PRU at the restart address
// The control register is actually outside PRU data memory but the
// mapped region is large enough to get to it.
//
// pru_num: PRU number, 0 or 1
void pru_restart(int pru_num)
{
}

// Call this routine to get PRU program counter
// The status register is actually outside PRU data memory but the
// mapped region is large enough to get to it.
uint32_t pru_get_pc(int pru_num)
{
   return 0;
}

// Call this routine to get PRU halt state
// The status register is actually outside PRU data memory but the
// mapped region is large enough to get to it.
uint32_t pru_get_halt(int pru_num)
{
   return 0;
}

// Call this routine to print registers
// The status register is actually outside PRU data memory but the
// mapped region is large enough to get to it.
void pru_print_registers(int pru_num)
{
}

// Call this routine to print memory
// The status register is actually outside PRU data memory but the
// mapped region is large enough to get to it.
// type: type of memory to dump
// start: start offset in bytes
// len : length in bytes
void pru_print_memory(MEM_TYPE mem_type, int start, int len)
{
}

// This returns the size and a pointer to the specified memory type
//
// mem_type: Type of memory to read
// mem_ptr: Address of memory
// mem_size: Size of memory
static void get_mem_addr_size(MEM_TYPE mem_type, uint8_t **mem_ptr,
       int *mem_size) {
}

// Write to memory. The DDR memory is uncached so slow.
// TODO: Should use cached memory and use cache flush/invalidate
// to improve performance.
//
// Offset and length are in bytes
// mem_type: Type of memory to read
// data: Pointer to location to write data to
// len: Length in bytes to read
// offset: Offset into memory in bytes to read from
int pru_write_mem(MEM_TYPE mem_type, void *data, int len, int offset)
{
   return 0;
}

// Read from memory
//
// mem_type: Type of memory to read
// data: Pointer to location to write data to
// len: Length in bytes to read
// offset: Offset into memory in bytes to read from
int pru_read_mem(MEM_TYPE mem_type, void *data, int len, int offset)
{
   return 0;
}

// Write a word to memory. Offset is in bytes from start of memory type
// 
// mem_type: Type of memory to read
// offset: Offset into memory in bytes
// return: Memory contents
uint32_t pru_read_word(MEM_TYPE mem_type, int offset) {
   return NULL;
}

// Write a word to memory. 
//
// mem_type: Type of memory to write
// offset: Offset into memory in bytes
// value: Value to write
void pru_write_word(MEM_TYPE mem_type, int offset, uint32_t value) {
   if (offset == PRU0_START_TIME_CLOCKS)
   {
#ifdef UDBG
	  if (value > 255)
		printf("---------------VALUE > 255 = %u\r\n");
#endif

	  TXRXBuf[0] = SCMD_REQ; //request
	  TXRXBuf[1] = SCMD_START_TIME_CLOCKS;
	  TXRXBuf[2] = (uint8_t) value;
	  TXRXBuf[3] = (uint8_t) (value >> 8);
	  TXRXBuf[4] = (uint8_t) (value >> 16);
	  TXRXBuf[5] = (uint8_t) (value >> 24);
	  TXRXBuf[6] = Crc8_Dallas(TXRXBuf,6);
#ifdef UDBG
	  printf("-->%x %x %x %x %x %x %x | VAL=%u\r\n",TXRXBuf[0],TXRXBuf[1],TXRXBuf[2],TXRXBuf[3],TXRXBuf[4],TXRXBuf[5],TXRXBuf[6],value);
#endif
	  for (int i = 0; i < 5; i++)
	  	if (SendData(com_port,7,(char*)TXRXBuf) == 7)
			break;
#ifdef UDBG
	  printf("SCMD_START_TIME_CLOCKS\r\n");
#endif
	  ReciveData(com_port,(char*)TXRXBuf,4);
#ifdef UDBG
	  printf("<--%x %x %x %x\r\n",TXRXBuf[0],TXRXBuf[1],TXRXBuf[2],TXRXBuf[3]);
	  if ((TXRXBuf[0] == SCMD_ANSW) && (TXRXBuf[1] == SCMD_START_TIME_CLOCKS) && (TXRXBuf[3] == Crc8_Dallas(TXRXBuf,3)))
		{
			printf("OK\r\n");
		}
#endif
   }
}

// Wait for bits to be set in a memory location. Timeout after a second
// ptr: Address to check
// mask: Mask for bits to check
// value: Value looking for
// desc: Description for error message on failure
static void wait_bits(uint32_t *ptr, uint32_t mask, uint32_t value, char *desc) {
}
// This routine changes the clock to the PRU if necessary to get the proper
// rate from the PWM. This changes the processor and some of the peripherals
// include ecap. The code only knows how to deal with specific target bit rates
// since calculating proper dividers is non trivial.
// tgt_bitrate_hz: Target bitrate for mfm clock and data in Hertz
// halt: Halt PRU during clock switch. If code is running best to halt.
// return: PRU clock rate in Hertz
uint32_t pru_set_clock(uint32_t tgt_bitrate_hz, int halt) {
  return 0;
}
