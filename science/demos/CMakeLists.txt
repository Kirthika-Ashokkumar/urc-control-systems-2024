# Copyright 2023 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.20)

project(demos LANGUAGES CXX)

libhal_build_demos(
  DEMOS
  co2_demo
  pressure_sensor_demo
  soil_sensor_demo
  scd40_demo
  color_sensor_demo
  led_strip_demo
  pump_manager_demo
  esp8266_demo

  SOURCES
  ../platform-implementations/scd40.cpp
  ../platform-implementations/pressure_sensor_bme680.cpp
  ../platform-implementations/soil_sensor_sht21.cpp
  ../platform-implementations/color_sensor_opt4048.cpp
  ../platform-implementations/pump_manager.cpp
  ../platform-implementations/esp8266_mission_control.cpp
  

  PACKAGES
  libhal-lpc40
  libhal-util
  libhal-esp8266

  LINK_LIBRARIES
  libhal::lpc40
  libhal::util
  libhal::esp8266
)