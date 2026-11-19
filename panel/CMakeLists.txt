cmake_minimum_required(VERSION 3.20)
project(frame_hand_tracking_panel LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

get_filename_component(_fca_default "${CMAKE_CURRENT_SOURCE_DIR}/../../Steam-Frame-Camera-Access" ABSOLUTE)
set(FCA_SOURCE_DIR "${_fca_default}" CACHE PATH "Steam-Frame-Camera-Access source folder")
set(FCA_OPENVR_LIB_DIR "/opt/steamvr/bin/linuxarm64" CACHE PATH "Directory with SteamVR's libopenvr_api.so")
set(FCA_OPENVR_RUNTIME_DIR "/opt/steamvr/bin/linuxarm64" CACHE PATH "libopenvr_api.so location at run time")
set(prompt "${FCA_SOURCE_DIR}/native/prompt")
if(NOT EXISTS "${prompt}/doodle_style.cpp")
  message(FATAL_ERROR "No Steam-Frame-Camera-Access checkout at ${FCA_SOURCE_DIR} (set -DFCA_SOURCE_DIR=...)")
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(CAIRO REQUIRED IMPORTED_TARGET cairo)
pkg_check_modules(FREETYPE REQUIRED IMPORTED_TARGET freetype2)
pkg_check_modules(VULKAN REQUIRED IMPORTED_TARGET vulkan)
if(NOT EXISTS "${FCA_OPENVR_LIB_DIR}/libopenvr_api.so")
  message(FATAL_ERROR "SteamVR's libopenvr_api.so not found in ${FCA_OPENVR_LIB_DIR}")
endif()

add_executable(frame-hand-tracking-panel
  main.cpp
  service.cpp
  hand_ui.cpp
  hand_outline.cpp
  gpu_outline.cpp
  vr_panel.cpp
  "${prompt}/doodle_assets.cpp"
  "${prompt}/doodle_style.cpp"
  "${prompt}/third_party/frameeyeosc/vk_texture.cpp")
target_include_directories(frame-hand-tracking-panel PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/../hand-ray/src" "${prompt}"
  "${prompt}/third_party/frameeyeosc" "${prompt}/third_party/openvr")
target_link_directories(frame-hand-tracking-panel PRIVATE "${FCA_OPENVR_LIB_DIR}")
target_link_libraries(frame-hand-tracking-panel PRIVATE PkgConfig::CAIRO PkgConfig::FREETYPE PkgConfig::VULKAN openvr_api)
set_target_properties(frame-hand-tracking-panel PROPERTIES BUILD_RPATH "${FCA_OPENVR_RUNTIME_DIR}")
target_compile_options(frame-hand-tracking-panel PRIVATE -Wall -Wextra)
file(GLOB _assets CONFIGURE_DEPENDS "${prompt}/assets/*.ttf" "${prompt}/assets/*.png")
set_source_files_properties("${prompt}/doodle_assets.cpp" PROPERTIES
  COMPILE_DEFINITIONS "DOODLE_ASSET_DIR=\"${prompt}/assets\"" OBJECT_DEPENDS "${_assets}")
set(_shaders "${CMAKE_CURRENT_SOURCE_DIR}/shaders")
set_source_files_properties(gpu_outline.cpp PROPERTIES
  COMPILE_DEFINITIONS "GPU_OUTLINE_SHADER_DIR=\"${_shaders}\""
  OBJECT_DEPENDS "${_shaders}/outline.vert.spv;${_shaders}/outline.frag.spv")
