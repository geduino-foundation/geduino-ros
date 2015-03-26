# Motion9.msg
#
# Copyright (C) 2015 Alessandro Francescon
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# The Motion9 message

# The accelerometer full scale range 2g, sensitivyty 16348 LSB / g
uint8 ACCEL_FS_2 = 0

# The accelerometer full scale range 4g, sensitivyty 8192 LSB / g
uint8 ACCEL_FS_4  = 1

# The accelerometer full scale range 8g, sensitivyty 4096 LSB / g
uint8 ACCEL_FS_8 = 2

# The accelerometer full scale range 16g, sensitivyty 2048 LSB / g
uint8 ACCEL_FS_16 = 3

# The gyro full scale range 250 degree/sec, sensitivyty 131 LSB / (degree/sec)
uint8 GYRO_FS_250 = 0

# The gyro full scale range 500 degree/sec, sensitivyty 65.5 LSB / (degree/sec)
uint8 GYRO_FS_500 = 1

# The gyro full scale range 1000 degree/sec, sensitivyty 32.8 LSB / (degree/sec)
uint8 GYRO_FS_1000 = 2

# The gyro full scale range 2000 degree/sec, sensitivyty 16.4 LSB / (degree/sec)
uint8 GYRO_FS_2000 = 3

# The accelerometer full scale range
uint8 full_scale_accel_range

# Accelerometer data
mpu9150_msgs/Int16Vector3 accel

# The gyroscope full scale range
uint8 full_scale_gyro_range

# Gyroscope data
mpu9150_msgs/Int16Vector3 gyro

# Magnetometer data
mpu9150_msgs/Int16Vector3 mag
