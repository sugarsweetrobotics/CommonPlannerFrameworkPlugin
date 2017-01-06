
# @author Yuki SUGA at SSR

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g")


option(BUILD_COMMONPLANNERFRAMEWORK_PLUGIN "Building CommonPlannerFrameworkPlugin" OFF)

if(NOT BUILD_COMMONPLANNERFRAMEWORK_PLUGIN)
  return()
else()
  if(NOT BUILD_CORBA_PLUGIN)
    message(FATAL_ERROR "CommonPlannerFramework Plugin requires CorbaPlugin.")
  endif()
endif()








set(IDL_INCLUDE_DIRS
  ${OPENRTM_PREFIX}/include/openrtm-1.1/rtm/idl
  ${OPENRTM_PREFIX}/rtm/idl
  idl )
idl_compile_cpp(idl_cpp_files idl_h_files idl TrajectryPlanner)
include_directories(${IDL_INCLUDE_DIRS})

if(0)
set(library CnoidCommonPlannerFramework)

set(library_sources )

add_cnoid_library(${library} STATIC 
  ${library_sources})
#  ${OpenHRP_idl_cpp_files} ${OpenHRP_idl_h_files})

target_link_libraries(${library} CnoidCorba)
apply_common_setting_for_library(${library} "")
endif(0)


# Plugin
set(plugin CnoidCommonPlannerFrameworkPlugin)

set(plugin_sources
  CommonPlannerFrameworkPlugin.cpp
  ${idl_cpp_files})

set(plugin_headers
  ${idl_h_files})

if(OPENRTM_VERSION STREQUAL "1.1.0")
  add_cnoid_plugin(${plugin} SHARED ${plugin_sources} ${plugin_headers} ${mofiles})
else()
  add_cnoid_OpenRTM_plugin(${plugin} SHARED ${plugin_sources} ${plugin_headers} ${mofiles})
  if(ENABLE_GCC_FVISIBILITY_HIDDEN)
    set_source_files_properties(BodyRTCItem.cpp
                                VirtualRobotRTC.cpp
                                VirtualRobotPortHandler.cpp
                                BridgeConf.cpp
                                RTCItem.cpp
                                OpenHRPClockGeneratorItem.cpp
                                ChoreonoidExecutionContext.cpp 
                                PROPERTIES COMPILE_FLAGS "-fvisibility=hidden")
  endif()
endif()

target_link_libraries(${plugin} CnoidBodyPlugin CnoidCorbaPlugin CnoidOpenRTM ${OPENRTM_LIBRARIES})
if(UNIX)
  target_link_libraries(${plugin} ${Boost_REGEX_LIBRARY})
endif()
apply_common_setting_for_plugin(${plugin} "${plugin_headers}")


# if(ENABLE_PYTHON)
#  add_subdirectory(python)
# endif()

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/rtc)
install(DIRECTORY ${PROJECT_BINARY_DIR}/${CNOID_PLUGIN_SUBDIR}/rtc DESTINATION ${CNOID_PLUGIN_SUBDIR}/rtc
  FILES_MATCHING PATTERN "*" EXCLUDE)

