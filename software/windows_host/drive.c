// This module is routines for controlling the disk drive.
// drive_select selects the specified drive or none.
// drive_set_head sets the head select lines to select the specified head.
// drive_seek_track0 returns the head to track 0.
// drive_setup sets up the drive for reading.
// drive_read_disk reads the disk drive.
// drive_current_cyl gets the current cylinder the drive heads are on
// drive_rpm get drive rpm
// drive_read_track read a track of deltas from a drive. It steps the
//    head if necessary.
// drive_step steps the head the requested number of cylinders.
// drive_is_file: Indicate if real drive or reading from file
// drive_enable_recovery: Set the recovery line active/inactive
// 
// The drive must be at track 0 on startup or drive_seek_track0 called.
//
// 07/05/2019 DJG Added support for using recovery signal
// 03/22/2019 DJG Added REV C support
// 09/01/2018 DJG Drive 0 is valid for drive_select(), don't drive
//    and select lines
// 08/05/2018 DJG Drive 0 invalid for drive_select()
// 05/06/2018 DJG Adjustement to try to make Syquest disks work better
// 03/09/2018 DJG Make sure correct setup script run so pins are in correct
//    direction.
// 10/02/2016 DJG Rob Jarratt change for DEC RD drives to detect when
//    it recalibrates back to track 0 when stepping past end
// 02/20/2016 DJG Split for drive reading and writing
// 01/06/2016 DJG Detect reversed J4 cable
// 12/24/2015 DJG Fix comment
// 07/30/2015 DJG Added support for revision B board.
// 05/16/2015 DJG Changes for drive_file.c
//
// Copyright 2014-2019 David Gesswein.
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
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include <inttypes.h>
#include <time.h>

//#include <prussdrv.h>
//#include <pruss_intc_mapping.h>

#include "msg.h"
#include "crc_ecc.h"
#include "emu_tran_file.h"
#include "mfm_decoder.h"
#include "cmd.h"
#include "cmd_write.h"
#include "deltas_read.h"
#include "pru_setup.h"
#include "drive.h"
#include "board.h"
#include "CSerialPort.h"



int init_uart(void)
{
   for (int i = 1; i < 100; i++)
   {
       com_port = OpenPort(i);
       if (com_port && (GetPortBoudRate(com_port) > 1200))
	{
	  SetPortBoudRate(com_port,921600);
	  SetPortHardwareFlow(com_port);
	  printf("found com #%u baudrate %u\r\n",i,GetPortBoudRate(com_port));

	  TXRXBuf[0] = SCMD_REQ; //request
	  TXRXBuf[1] = SCMD_GET_VERSION;
	  TXRXBuf[2] = 0x6e;
	  TXRXBuf[3] = 0;
	  if (SendData(com_port,3,(char*)TXRXBuf) != 3)
		printf("error sending\r\n");
	  printf("SCMD_GET_VERSION\r\n");
	  ReciveData(com_port,(char*)TXRXBuf,4);
	  printf("%x %x %x %x\r\n",TXRXBuf[0],TXRXBuf[1],TXRXBuf[2],TXRXBuf[3]);
	  if ((TXRXBuf[0] == SCMD_ANSW) && (TXRXBuf[1] == SCMD_GET_VERSION) && (TXRXBuf[3] == Crc8_Dallas(TXRXBuf,3)))
		{
	  	printf("Board version: %u.%u\r\n", TXRXBuf[2] >> 4,TXRXBuf[2] & 0xF);
		return(TXRXBuf[2]);
		}
	}
       else	
       ClosePort(com_port);
   }
   return 0;
}

// The cylinder the drive heads are at
static int current_cyl;

// Activate drive selects to select specified drive
//
// drive: drive number to select (1-4) or zero to select none.
void drive_select(int drive)
{

   if (drive < 0 || drive > 4) {
      msg(MSG_FATAL, "Invalid drive %d\n", drive);
      exit(1);
   }
}

// Select the disk head. Head 0-15 supported. The MSB of head select
// is reduced write current on older drives but since we aren't writing
// we don't have to worry about it.
//
// head: Head to select 0-15
void drive_set_head(int head)
{
   if (head < 0 || head > 15) {
      msg(MSG_FATAL, "Invalid head %d\n", head);
      exit(1);
   }
	  TXRXBuf[0] = SCMD_REQ; //request
	  TXRXBuf[1] = SCMD_SET_HEAD;
	  TXRXBuf[2] = head;
	  TXRXBuf[3] = Crc8_Dallas(TXRXBuf,3);
	  TXRXBuf[4] = 0;
#ifdef UDBG
	  printf("-->%x %x %x %x\r\n",TXRXBuf[0],TXRXBuf[1],TXRXBuf[2],TXRXBuf[3]);
#endif
	  for (int i = 0; i < 5; i++)
	  	if (SendData(com_port,4,(char*)TXRXBuf) == 4)
			break;
#ifdef UDBG
	  printf("SCMD_SET_HEAD\r\n");
#endif
	  ReciveData(com_port,(char*)TXRXBuf,4);
#ifdef UDBG
	  printf("<--%x %x %x %x\r\n",TXRXBuf[0],TXRXBuf[1],TXRXBuf[2],TXRXBuf[3]);
	  if ((TXRXBuf[0] == SCMD_ANSW) && (TXRXBuf[1] == SCMD_SET_HEAD) && (TXRXBuf[3] == Crc8_Dallas(TXRXBuf,3)))
		{
	  	printf("head_num: %u\r\n", TXRXBuf[2]);
		}
#endif
}

// Returns non zero if track 0 signal is active (zero)
int drive_at_track0(void)
{  
  int rc;
	if (drive_get_drive_status()&1)
		rc = 0;
	else
		rc = 1;
  return rc;
}

// Return the heads to track 0. This is slow so if you know what track you
// are on use a normal seek.
void drive_seek_track0(void)
{
   int rc;
   int16_t max_cyl = (int16_t)MAX_CYL*-1;

#ifdef UDBG
	printf("MAX_CYL = %d %d\r\n",max_cyl,MAX_CYL);
#endif
	  TXRXBuf[0] = SCMD_REQ; //request
	  TXRXBuf[1] = SCMD_SEEK_TRACK0;
	  TXRXBuf[2] = Crc8_Dallas(TXRXBuf,2);
#ifdef UDBG
	  printf("-->%x %x %x\r\n",TXRXBuf[0],TXRXBuf[1],TXRXBuf[2]);
#endif
	  for (int i = 0; i < 5; i++)
	  	if (SendData(com_port,3,(char*)TXRXBuf) == 3)
			break;
#ifdef UDBG
	  printf("SCMD_SEEK to track0\r\n");
#endif
	  ReciveData(com_port,(char*)TXRXBuf,4);
#ifdef UDBG
	  printf("<--%x %x %x %x\r\n",TXRXBuf[0],TXRXBuf[1],TXRXBuf[2],TXRXBuf[3]);
#endif
	  if ((TXRXBuf[0] == SCMD_ANSW) && (TXRXBuf[1] == SCMD_SEEK_TRACK0) && (TXRXBuf[3] == Crc8_Dallas(TXRXBuf,3)))
		{
			rc = TXRXBuf[2];
		}
		else
			rc = -1;

   if (rc != 0) {
      drive_print_drive_status(MSG_FATAL, drive_get_drive_status());
      exit(1);
   };

   // Low indicates we are at track 0
   if (!drive_at_track0()) {
      msg(MSG_FATAL,"Failed to reach track 0\n");
      exit(1);
   }
   current_cyl = 0;
}

// Step requested number of cylinders
// buffered_seek: nonzero if drive supports buffered seeks
// steps: Number of cylinders to step, negative is towards lower cylinder
// update_cyl: Non zero if internal current cylinder should be updated
// err_fatal: If set a timeout error will terminate, otherwise returned
//    as SEEK_TIMEOUT. SEEK_RECAL indicates the drive recalibrated to track 0
int drive_step(int step_speed, int16_t steps, int update_cyl, int err_fatal) {
   int seek_cmd;
   int rc;
   int wait_count = 0, ready = 0;
   int start_cyl = current_cyl;

   if (update_cyl == DRIVE_STEP_UPDATE_CYL) {
      current_cyl += steps;
   }
   if (step_speed == DRIVE_STEP_FAST) {
      seek_cmd = CMD_SEEK_FAST;
   } else {
      seek_cmd = CMD_SEEK_SLOW;
   }

	  TXRXBuf[0] = SCMD_REQ; //request
	  TXRXBuf[1] = SCMD_SEEK;
	  TXRXBuf[2] = seek_cmd;
#ifdef UDBG
	  printf("steps = %d\r\n",steps);
#endif
	  TXRXBuf[3] = (uint8_t) steps;
	  TXRXBuf[4] = (uint8_t) (steps >> 8);
	  TXRXBuf[5] = Crc8_Dallas(TXRXBuf,5);
#ifdef UDBG
	  printf("-->%x %x %x %x %x %x\r\n",TXRXBuf[0],TXRXBuf[1],TXRXBuf[2],TXRXBuf[3],TXRXBuf[4],TXRXBuf[5]);
#endif
	  for (int i = 0; i < 5; i++)
	  	if (SendData(com_port,6,(char*)TXRXBuf) == 6)
			break;
#ifdef UDBG
	  printf("SCMD_SEEK\r\n");
#endif
	  ReciveData(com_port,(char*)TXRXBuf,4);
#ifdef UDBG
	  printf("<--%x %x %x %x\r\n",TXRXBuf[0],TXRXBuf[1],TXRXBuf[2],TXRXBuf[3]);
#endif
	  if ((TXRXBuf[0] == SCMD_ANSW) && (TXRXBuf[1] == SCMD_SEEK) && (TXRXBuf[3] == Crc8_Dallas(TXRXBuf,3)))
		{
			rc = TXRXBuf[2];
		}
		else
			rc = -1;

   if (rc != 0) {
      if (err_fatal) {
         msg(MSG_FATAL,"seek command failed\n");
         drive_print_drive_status(MSG_FATAL, drive_get_drive_status());
         exit(1);
      } else {
         // Wait for seek complete for 10 seconds. This prevents errors if we
         // next read the disk
         while (wait_count++ < 100 && !ready) {
            // Bit is active low
            ready = !(drive_get_drive_status() & BIT_MASK(R31_SEEK_COMPLETE_BIT));
            usleep(100000);
         }
      }
      rc = DRIVE_STEP_TIMEOUT;
   }

   if (drive_at_track0() && start_cyl != 0 && current_cyl != 0) {
      msg(MSG_INFO, "Disk has recalibrated to track 0\n");
      current_cyl = 0;
      rc = DRIVE_STEP_RECAL;
   }

   return rc;
}

// Get current cylinder
int drive_current_cyl() {
   return current_cyl;
}

// Select drive, check if drive is ready and then return to track zero if needed
//
// drive_params: Drive parameters
void drive_setup(DRIVE_PARAMS *drive_params)
{
   // Turn off recovery mode
   drive_enable_recovery(0);

   drive_select(drive_params->drive);


   if (!drive_at_track0()) {
      msg(MSG_INFO, "Returning to track 0\n");
      drive_seek_track0();
   }
}

// Return the drive status value. See print routine for bit definitions.
uint32_t drive_get_drive_status(void)
{
	  TXRXBuf[0] = SCMD_REQ; //request
	  TXRXBuf[1] = SCMD_GET_STATUS;
	  TXRXBuf[2] = Crc8_Dallas(TXRXBuf,2);
#ifdef UDBG
	  printf("-->%x %x %x %x\r\n",TXRXBuf[0],TXRXBuf[1],TXRXBuf[2],TXRXBuf[3]);
#endif
	  for (int i = 0; i < 5; i++)
	  	if (SendData(com_port,3,(char*)TXRXBuf) == 3)
			break;
#ifdef UDBG
	  printf("SCMD_GET_STATUS\r\n");
#endif
	  ReciveData(com_port,(char*)TXRXBuf,4);
#ifdef UDBG
	  printf("<--%x %x %x %x\r\n",TXRXBuf[0],TXRXBuf[1],TXRXBuf[2],TXRXBuf[3]);
#endif
	  if ((TXRXBuf[0] == SCMD_ANSW) && (TXRXBuf[1] == SCMD_GET_STATUS) && (TXRXBuf[3] == Crc8_Dallas(TXRXBuf,3)))
		{
#ifdef UDBG
	  	printf("status: %u\r\n", TXRXBuf[2]);
#endif
		return TXRXBuf[2];
		}
	return 0;
//   return pru_read_word(MEM_PRU0_DATA, PRU0_STATUS);
}

// Decodes and prints the disk status value from the PRU.
//
// status: Disk status value (from pru_get_drive_status)
void drive_print_drive_status(int level, uint32_t status)
{
   struct {
      char *desc;
      int bit;
   } reg_bits[] = {
         {"Write fault", R31_WRITE_FAULT_BIT},
         {"Seek complete", R31_SEEK_COMPLETE_BIT},
         {"Index", R31_INDEX_BIT},
         {"Ready", R31_READY_BIT},
         {"Drive selected", R31_DRIVE_SEL},
 // TODO        {"Track 0", R31_TRACK_0}
   };
   int n;

   for (n = 0; n < ARRAYSIZE(reg_bits); n++) {
      // Control lines are active low
      if (status & (1 << reg_bits[n].bit)) {
         msg(level, "Not %s \n", reg_bits[n].desc);
      } else {
         msg(level, "%s \n", reg_bits[n].desc);
      }
   }
}


// Print the drive Revolutions Per Minute (RPM)
// return drive RPM
double drive_rpm(void) {
uint8_t str[4];
uint32_t pulsewidth,period;

	  TXRXBuf[0] = SCMD_REQ; //request
	  TXRXBuf[1] = SCMD_GET_RPM;
	  TXRXBuf[2] = Crc8_Dallas(TXRXBuf,2);
#ifdef UDBG
	  printf("-->%x %x %x %x\r\n",TXRXBuf[0],TXRXBuf[1],TXRXBuf[2],TXRXBuf[3]);
#endif
	  for (int i = 0; i < 5; i++)
	  	if (SendData(com_port,3,(char*)TXRXBuf) == 3)
			break;
#ifdef UDBG
	  printf("SCMD_GET_RPM\r\n");
#endif
	  ReciveData(com_port,(char*)TXRXBuf,11);
#ifdef UDBG
	  printf("<--%x %x %x %x\r\n",TXRXBuf[0],TXRXBuf[1],TXRXBuf[2],TXRXBuf[3]);
#endif
	  if ((TXRXBuf[0] == SCMD_ANSW) && (TXRXBuf[1] == SCMD_GET_RPM) && (TXRXBuf[10] == Crc8_Dallas(TXRXBuf,10)))
		{
		str[0] = TXRXBuf[2];
		str[1] = TXRXBuf[3];
		str[2] = TXRXBuf[4];
		str[3] = TXRXBuf[5];
		pulsewidth = *((uint32_t*) str);
		str[0] = TXRXBuf[6];
		str[1] = TXRXBuf[7];
		str[2] = TXRXBuf[8];
		str[3] = TXRXBuf[9];
		period = *((uint32_t*) str);
#ifdef UDBG
	  	printf("status: pulsewidth = %u ; period = %u\r\n", pulsewidth,period);
#endif
      		return(400e6 / period * 60);
		}
  return 3600;
}

// Activate recovery mode if enable non zero
//
// enable: Non zero value to enable recovery mode, 0 normal mode
void drive_enable_recovery(int enable)
{
	printf("drive_enable_recovery - NOT SUPPORTED NOW!\r\n");
}
