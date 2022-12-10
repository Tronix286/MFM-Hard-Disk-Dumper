/*
 * drive.h
 *
 *  Created on: Dec 23, 2013
 *      Author: djg
 */

#ifndef DRIVE_H_
#define DRIVE_H_

int init_uart(void);

void drive_select(int drive);
void drive_set_head(int head);
void drive_seek_track0(void);
void drive_setup(DRIVE_PARAMS *drive_params);
void drive_read_disk(DRIVE_PARAMS *drive_params, void *deltas, int max_deltas);
int drive_at_track0(void);
uint32_t drive_get_drive_status(void);
void drive_print_drive_status(int level, uint32_t status);
double drive_rpm(void);
// Step rates for drive_params->step_speed and drive.c drive_step parameter
#define DRIVE_STEP_SLOW 0
#define DRIVE_STEP_FAST 1

#define step_speed_text(x) (x == DRIVE_STEP_SLOW ? "slow ST506" : "fast ST412")

// Parameters to drive.c drive_step
#define DRIVE_STEP_FATAL_ERR 1
#define DRIVE_STEP_RET_ERR 0
#define DRIVE_STEP_NO_UPDATE_CYL 0
#define DRIVE_STEP_UPDATE_CYL 1

// Return values from drive.c drive_step. 0 no error
#define DRIVE_STEP_TIMEOUT 1
#define DRIVE_STEP_RECAL 2

int drive_step(int seek_speed, int16_t steps, int update_cyl, int err_fatal);
int drive_current_cyl(void);
void drive_read_track(DRIVE_PARAMS *drive_params, int cyl, int head,
      void *deltas, int max_deltas);
void drive_initialize(void);
int drive_get_board_revision(void);
void drive_write_disk(DRIVE_PARAMS *drive_params);
void drive_enable_recovery(int enable);


#endif /* DRIVE_H_ */
