cmake_minimum_required(VERSION 3.20)

set(build_debug false)
set(build_release false)

include(ProcessorCount)
ProcessorCount(num_processors)
if(${num_processors} GREATER 1)
    math(EXPR num_processors "${num_processors}-1")
endif()

if(${CMAKE_ARGV3} STREQUAL "debug")
    set(build_debug true)
elseif(${CMAKE_ARGV3} STREQUAL "release")
    set(build_release true)
endif()

set(build_root_dir ${CMAKE_CURRENT_LIST_DIR}/../builds/msd)
file(REAL_PATH ${build_root_dir} build_root_dir)

set(workspace_root ${CMAKE_CURRENT_LIST_DIR}/../)
file(REAL_PATH workspace_root ${workspace_root})

set(msd_install_root ${workspace_root}/installs/msd)
file(REAL_PATH ${msd_install_root} msd_install_root)

file(MAKE_DIRECTORY ${build_root_dir})
file(MAKE_DIRECTORY ${msd_install_root})

set(generator "Visual Studio 16 2019")

if(${build_debug})
    set(build_type "Debug")
elseif(${build_release})
    set(build_type "Release")
endif()

execute_process(COMMAND ${CMAKE_COMMAND}  -G ${generator} -DCMAKE_INSTALL_PREFIX=${msd_install_root} -S ${workspace_root} -B ${build_root_dir}
                COMMAND_ECHO STDOUT
                WORKING_DIRECTORY ${build_root_dir}
                RESULT_VARIABLE retCode)

execute_process(COMMAND ${CMAKE_COMMAND} --BUILD ${build_root_dir} --target ALL_BUILD --parallel ${num_processors} --config ${build_type} 
                COMMAND_ECHO STDOUT
                WORKING_DIRECTORY ${build_root_dir}
                RESULT_VARIABLE retCode)

execute_process(COMMAND ${CMAKE_COMMAND} --install ${build_root_dir} --config ${build_type}
                COMMAND_ECHO STDOUT
                WORKING_DIRECTORY ${build_root_dir}
                RESULT_VARIABLE retCode)
