/*
 rplidar_off.c

 Copyright (C) 2017 Alessandro Francescon
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 rplidar_off is a util to power off RPLidar.

 Usage:
 - rplidar_off <port>
 */

#include <sys/ioctl.h>
#include <termios.h>
#include <linux/serial.h>
#include <fcntl.h>
#include <stdio.h>

int main(int argc, char* argv[]) {

   if (argc != 2) {
     return -1;
   }

   // Open serial port
   int fd = open(argv[1], O_RDWR | O_NOCTTY );

   // Set RTS pin
   ioctl(fd, TIOCMBIS, TIOCM_RTS);

   getchar();

   // Clear RTS pin
   ioctl(fd, TIOCMBIC, TIOCM_RTS);

   close(fd);

   return 0;

}
