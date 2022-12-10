/*
Copyright (c) 2017 Ahmad Bashar Eter

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include "CSerialPort.h"

PORT com_port;
uint8_t TXRXBuf[20];

/*
 * Crc8_Dallas() - Polinom 0x31
 *
 */

//===8<==============Original message text===============
static const uint8_t Crc8Table_Dallas[256] = { 0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
        157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220, 35, 125, 159, 193, 66, 28, 254, 160,
        225, 191, 93, 3, 128, 222, 60, 98, 190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255, 70,
        24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7, 219, 133, 103, 57, 186, 228, 6, 88, 25, 71,
        165, 251, 120, 38, 196, 154, 101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36, 248, 166,
        68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185, 140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242,
        172, 47, 113, 147, 205, 17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80, 175, 241, 19,
        77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238, 50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76,
        18, 145, 207, 45, 115, 202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139, 87, 9, 235, 181,
        54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22, 233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201,
        74, 20, 246, 168, 116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53 };

uint8_t Crc8_Dallas(uint8_t * _pBuff, uint32_t _len)
{
    uint8_t Crc8 = 0;

    for (uint32_t i = 0; i < _len; i++) {
        Crc8 = Crc8Table_Dallas[Crc8 ^ _pBuff[i]];
    }

    return Crc8;
}

PORT OpenPort(int idx)
{
	HANDLE hComm;
	TCHAR comname[100];
	wsprintf(comname, TEXT("\\\\.\\COM%d"), idx);
	hComm = CreateFile(comname,            //port name 
		GENERIC_READ | GENERIC_WRITE, //Read/Write   				 
		0,            // No Sharing                               
		NULL,         // No Security                              
		OPEN_EXISTING,// Open existing port only                     
		0,            // Non Overlapped I/O                           
		NULL);        // Null for Comm Devices
	
	if (hComm == INVALID_HANDLE_VALUE)
		return NULL;
	COMMTIMEOUTS timeouts = { 0 };
	timeouts.ReadIntervalTimeout = 0; //5;
	timeouts.ReadTotalTimeoutConstant = 0; //15;
	timeouts.ReadTotalTimeoutMultiplier = 0; //1;
	timeouts.WriteTotalTimeoutConstant = 100; //5;
	timeouts.WriteTotalTimeoutMultiplier = 0; //0;

	if (SetCommTimeouts(hComm, &timeouts) == FALSE)
		return NULL;

	if (SetCommMask(hComm, EV_RXCHAR) == FALSE)
		return NULL;

	return hComm;
}
void ClosePort(PORT com_port)
{
	CloseHandle(com_port);
}

int SetPortHardwareFlow(PORT com_port)
{
	DCB dcbSerialParams = { 0 };
	BOOL Status;
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	Status = GetCommState(com_port, &dcbSerialParams);
	if (Status == FALSE)
		return FALSE;
	dcbSerialParams.fRtsControl = RTS_CONTROL_ENABLE;
	dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;
	Status = SetCommState(com_port, &dcbSerialParams);
	return Status;
}

int SetPortBoudRate(PORT com_port, int rate)
{
	DCB dcbSerialParams = { 0 };
	BOOL Status;
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	Status = GetCommState(com_port, &dcbSerialParams);
	if (Status == FALSE)
		return FALSE;
	dcbSerialParams.BaudRate = rate;
	Status = SetCommState(com_port, &dcbSerialParams);
	return Status;
}

int SetPortDataBits(PORT com_port, int bits)
{
	DCB dcbSerialParams = { 0 };
	BOOL Status;
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	Status = GetCommState(com_port, &dcbSerialParams);
	if (Status == FALSE)
		return FALSE;
	dcbSerialParams.ByteSize = bits;
	Status = SetCommState(com_port, &dcbSerialParams);
	return Status;
}

int SetPortStopBits(PORT com_port, int bits)
{
	DCB dcbSerialParams = { 0 };
	BOOL Status;
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	Status = GetCommState(com_port, &dcbSerialParams);
	if (Status == FALSE)
		return FALSE;
	dcbSerialParams.StopBits = bits;
	Status = SetCommState(com_port, &dcbSerialParams);
	return Status;
}

int SetPortParity(PORT com_port, int parity)
{
	DCB dcbSerialParams = { 0 };
	BOOL Status;
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	Status = GetCommState(com_port, &dcbSerialParams);
	if (Status == FALSE)
		return FALSE;
	dcbSerialParams.Parity = parity;
	Status = SetCommState(com_port, &dcbSerialParams);
	return Status;
}

int GetPortBoudRate(PORT com_port)
{
	DCB dcbSerialParams = { 0 };
	BOOL Status;
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	Status = GetCommState(com_port, &dcbSerialParams);
	if (Status == FALSE)
		return -1;
	return dcbSerialParams.BaudRate;
}
int GetPortDataBits(PORT com_port) {
	DCB dcbSerialParams = { 0 };
	BOOL Status;
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	Status = GetCommState(com_port, &dcbSerialParams);
	if (Status == FALSE)
		return -1;
	return dcbSerialParams.ByteSize;
}
int GetPortStopBits(PORT com_port) {
	DCB dcbSerialParams = { 0 };
	BOOL Status;
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	Status = GetCommState(com_port, &dcbSerialParams);
	if (Status == FALSE)
		return -1;
	return dcbSerialParams.StopBits;
}
int GetPortParity(PORT com_port) {
	DCB dcbSerialParams = { 0 };
	BOOL Status;
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	Status = GetCommState(com_port, &dcbSerialParams);
	if (Status == FALSE)
		return -1;
	return dcbSerialParams.Parity;
}

int SendData(PORT com_port, size_t size, const char * data)
{
//	DWORD  dNoOFBytestoWrite = strlen(data);
//printf("written = %lu bytes\r\n",dNoOFBytestoWrite);
	DWORD  dNoOfBytesWritten;
	BOOL Status = WriteFile(com_port,
				data,
				size,
				&dNoOfBytesWritten,
				NULL);
	if (Status == FALSE)
		return -1;
	return dNoOfBytesWritten;
}

int ReciveData(PORT com_port, char * data,int len)
{
	DWORD dwEventMask;
	DWORD NoBytesRead;
	BOOL Status;
	Status = WaitCommEvent(com_port, &dwEventMask, NULL);
	if (Status == FALSE) {
		return FALSE;
	}
	Status = ReadFile(com_port, data, len, &NoBytesRead, NULL);
	data[NoBytesRead] = 0;
	if (Status == FALSE) {
		return FALSE;
	}
	return TRUE;
}