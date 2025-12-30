# Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.20)

project(fastdds-examples VERSION "3.4.1.0" LANGUAGES C CXX)

###############################################################################
# Load CMake modules
###############################################################################
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/modules)

find_package(Threads REQUIRED)

# add_subdirectory(benchmark)
# add_subdirectory(configuration)
# add_subdirectory(content_filter)
# add_subdirectory(custom_payload_pool)
# add_subdirectory(delivery_mechanisms)
# add_subdirectory(discovery_server)
# add_subdirectory(flow_control)
add_subdirectory(hello_world)
add_subdirectory(rpc)
add_subdirectory(rtps)
# add_subdirectory(static_edp_discovery)
# add_subdirectory(topic_instances)
# add_subdirectory(xtypes)

# if (SECURITY)
#     add_subdirectory(security)
# endif()
