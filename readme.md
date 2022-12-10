## Disclaimer
Trying to make a MFM hard disk dumper using a Raspberry Pi Pico board. This is a weekend home project, dont expect too much from it. Here you will find bad code, unstable work, flaws. If you want to repeat this device - know that you do it at your own peril and risk. Most likely, you will have to figure out how the code works, make changes to it, and in the end you still wont get 100% stability. Pi Pico runs at a very high frequency 400MHz (overclocked), which also does not contribute to reliable operation. Therefore, if you are not sure that you are ready to debug and modify this device yourself, it is best to look towards the original project from David Gesswein.
## Description
The project is based on a wonderful development from David Gesswein - [MFM Hard Disk Reader/Emulator](http://https://www.pdp8online.com/mfm/mfm.shtml "MFM Hard Disk Reader/Emulator"). Pi Pico acts as a simple executor of commands that the main computer (host) sends to her through a USB virtual COM port. Pico also sends the read track from the disk to the computer for further decryption by the original software from https://github.com/dgesswein/mfm/tree/master/mfm Of course, changes have been made to the original software that allow it to communicate with pico through a virtual com port using a simple binary protocol.

![](/hardware/photo_2022-12-02_10-48-02.jpg)
## Compiling
I am using MinGW x86 for build mfm_read target. Open MinGW shell, go to /software/windows_host and type `make clean` then `make mfm_read`.
For Pi Pico i am using Visual CODE and cmake.
