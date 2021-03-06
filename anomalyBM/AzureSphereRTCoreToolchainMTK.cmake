set(CMAKE_SYSTEM_NAME Generic)

set(AS_INT_APP_TYPE "RTApp" CACHE INTERNAL "Type of application (\"HLApp\" or \"RTApp\")")
set(AZURE_SPHERE_CMAKE_PATH "$ENV{AzureSphereDefaultSDKDir}CMakeFiles" CACHE INTERNAL "Path to the Azure Sphere SDK CMakeFiles")
set(AZURE_SPHERE_SDK_PATH $ENV{AzureSphereDefaultSDKDir} CACHE INTERNAL "Path to the Azure Sphere SDK")

include("${AZURE_SPHERE_CMAKE_PATH}/AzureSphereToolchainBase.cmake")

if(DEFINED ARM_GNU_PATH)
    string(REPLACE "\\" "/" ARM_GNU_PATH ${ARM_GNU_PATH})
    string(REGEX REPLACE "/$" "" ARM_GNU_PATH ${ARM_GNU_PATH})
    string(REGEX MATCH "bin$" ARM_GNU_PATH_IS_BIN ${ARM_GNU_PATH})
    if("${ARM_GNU_PATH_IS_BIN}" STREQUAL "")
        set(ENV{ArmGnuBasePath} ${ARM_GNU_PATH})
        set(ENV{ArmGnuBinPath} "${ARM_GNU_PATH}/bin")
    else()
        string(FIND ${ARM_GNU_PATH} "/" ARM_GNU_PATH_END REVERSE)
        string(SUBSTRING ${ARM_GNU_PATH} 0 ${ARM_GNU_PATH_END} ARM_GNU_BASE_PATH)
        set(ENV{ArmGnuBasePath} ${ARM_GNU_BASE_PATH})
        set(ENV{ArmGnuBinPath} ${ARM_GNU_PATH})
    endif()
endif()
set(ARM_GNU_BIN_PATH $ENV{ArmGnuBinPath})
set(ARM_GNU_BASE_PATH $ENV{ArmGnuBasePath} CACHE INTERNAL "Path to the ARM embedded toolset")

set(CMAKE_FIND_ROOT_PATH "${ARM_GNU_BASE_PATH}")

# Set up compiler and flags
if(${CMAKE_HOST_WIN32})
    set(CMAKE_C_COMPILER "${ARM_GNU_BIN_PATH}/arm-none-eabi-gcc.exe" CACHE INTERNAL "Path to the C compiler in the ARM embedded toolset targeting Real-Time Core")
    set(CMAKE_CXX_COMPILER "${ARM_GNU_BIN_PATH}/arm-none-eabi-g++.exe" CACHE INTERNAL "Path to the CXX compiler in the ARM embedded toolset targeting Real-Time Core")
    set(CMAKE_AR "${ARM_GNU_BIN_PATH}/arm-none-eabi-ar.exe" CACHE INTERNAL "Path to the AR compiler in the ARM embedded toolset targeting Real-Time Core")

    set(ENV{PATH} "${AZURE_SPHERE_SDK_PATH}/Tools;${ARM_GNU_BIN_PATH};$ENV{PATH}")
else()
    set(CMAKE_C_COMPILER "${ARM_GNU_BIN_PATH}/arm-none-eabi-gcc" CACHE INTERNAL "Path to the C compiler in the ARM embedded toolset targeting Real-Time Core")
    set(CMAKE_CXX_COMPILER "${ARM_GNU_BIN_PATH}/arm-none-eabi-g++" CACHE INTERNAL "Path to the CXX compiler in the ARM embedded toolset targeting Real-Time Core")
    set(CMAKE_AR "${ARM_GNU_BIN_PATH}/arm-none-eabi-ar" CACHE INTERNAL "Path to the AR compiler in the ARM embedded toolset targeting Real-Time Core")
    set(CMAKE_STRIP "${ARM_GNU_BIN_PATH}/arm-none-eabi-strip" CACHE INTERNAL "Path to the strip tool in the ARM embedded toolset targeting Real-Time Core")

    set(ENV{PATH} "${AZURE_SPHERE_SDK_PATH}/Tools:${ARM_GNU_BIN_PATH}:$ENV{PATH}")
endif()

set(CMAKE_C_FLAGS_INIT "-std=gnu11 -mthumb -mcpu=cortex-m4 -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -Wall -Os -g -DNDEBUG -I${ARM_GNU_BASE_PATH}/arm-none-eabi/include/c++/10.3.1/ -I${ARM_GNU_BASE_PATH}/arm-none-eabi/include/")
set(CMAKE_CXX_FLAGS_INIT "-std=gnu++14 -lstdc++ -lsupc++ -fno-common -fno-exceptions -fno-rtti -mthumb -mcpu=cortex-m4 -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -Wall -Os -g -DNDEBUG -I${ARM_GNU_BASE_PATH}/arm-none-eabi/include/c++/10.3.1/ -I${ARM_GNU_BASE_PATH}/arm-none-eabi/include/")
set(CMAKE_EXE_LINKER_FLAGS_INIT "-nostartfiles  -Os -g -DNDEBUG -Wl,--no-undefined -Wl,-n -T \"${CMAKE_SOURCE_DIR}/linker.ld\" -fdata-sections -ffunction-sections -Wl,--gc-sections -Xlinker -Map=${PROJECT_NAME}.map")

#file(GLOB ARM_GNU_INCLUDE_PATH "${ARM_GNU_BASE_PATH}/lib/gcc/arm-none-eabi/*/include")
#set(CMAKE_C_STANDARD_INCLUDE_DIRECTORIES "${ARM_GNU_INCLUDE_PATH}" "${ARM_GNU_BASE_PATH}/arm-none-eabi/include")
#set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES "${ARM_GNU_INCLUDE_PATH}" "${ARM_GNU_BASE_PATH}/arm-none-eabi/include")
#set(COMPILE_DEBUG_FLAGS $<$<CONFIG:Debug>:-g2> $<$<CONFIG:Debug>:-gdwarf-2> $<$<CONFIG:Debug>:-O0>)
#set(COMPILE_RELEASE_FLAGS $<$<CONFIG:Release>:-g1> $<$<CONFIG:Release>:-O3>)
add_compile_options(-Wall ${COMPILE_DEBUG_FLAGS} ${COMPILE_RELEASE_FLAGS})
