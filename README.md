<!--
#
# Licensed to the Apache Software Foundation (ASF) under one
# or more contributor license agreements.  See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership.  The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance
# with the License.  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
#  KIND, either express or implied.  See the License for the
# specific language governing permissions and limitations
# under the License.
#
-->

first attempt to install mynewt on the ID107 HR smartwatch
experimental I2C  KX022 accel sensor driver


my setup is el-cheapo :
no J-link, but st-link
no Segger RTT, but a serial port (console)
(so you have to solder tx, rx of the watch and connect it to serial port)

for info : 
https://mynewt.apache.org


I struggled (and gave up) to install it on raspberry pi
I used : newt_1.4.1-1_amd64.deb



No luck with BLE and bootloader: use the elf.bin binary
The id107 smartwatch has only 256k flash memory

After disactivating the log circular buffer setting in syscfg, bluetooth works!


Current status : working acceleration-sensor (kx022)
working OIC-server, android mynewt app can visualize the data! 

TODO: si114x heart rate sensor
- kx022 -- tap detection, and interrupt generation (detect movement during sleep)
- si1143 disable the sensor (function is already there, but has to be called)



TIP to check config :
newt target config show thingy_my_sensor



In this repo is a working bin : 
id107_kx022_oic_test/app/apps/sensors_test/sensors_test.elf.bin

By flashing it to the ID107HR smart watch, you can get a feel of mynewt

there is an app : Mynewt Sensor Monitor which works with the sensors: 
https://play.google.com/store/apps/details?id=io.runtime.sensoroic&hl=en_US

and uses the OIC protocol to read/and visualize data from the sensors on the watch


