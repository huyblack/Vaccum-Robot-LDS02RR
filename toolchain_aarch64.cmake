# --- Target system ---
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# --- Toolchain path ---
set(TOOLCHAIN_ROOT /home/huy/arm-gnu-toolchain-12.2.rel1-x86_64-aarch64-none-linux-gnu)
set(CMAKE_C_COMPILER ${TOOLCHAIN_ROOT}/bin/aarch64-none-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_ROOT}/bin/aarch64-none-linux-gnu-g++)

# --- Sysroot ---
set(CMAKE_SYSROOT /home/huy/mnt/usb_ext4)

# --- Compiler target (optional) ---
set(CMAKE_C_COMPILER_TARGET aarch64-linux-gnu)
set(CMAKE_CXX_COMPILER_TARGET aarch64-linux-gnu)

# --- Linker flags ---
set(CMAKE_EXE_LINKER_FLAGS 
    "--sysroot=${CMAKE_SYSROOT} \
    -B${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu \
    -L${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu \
    -L${CMAKE_SYSROOT}/lib/aarch64-linux-gnu \
    -Wl,-rpath,${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu \
    -Wl,-rpath,${CMAKE_SYSROOT}/lib/aarch64-linux-gnu \
    -Wl,-rpath-link,${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu \
    -Wl,-rpath-link,${CMAKE_SYSROOT}/lib/aarch64-linux-gnu"
    CACHE STRING "" FORCE)

set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}" CACHE STRING "" FORCE)


# --- Standard libraries ---
set(CMAKE_CXX_STANDARD_LIBRARIES "-lstdc++ -lm -lc")

# --- ROS paths ---
set(CMAKE_PREFIX_PATH "${CMAKE_SYSROOT}/opt/ros/humble" CACHE STRING "" FORCE)

# --- Find root path settings ---
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# --- Avoid wrong libstdc++ ---
set(CMAKE_IGNORE_PATH "${TOOLCHAIN_ROOT}/aarch64-none-linux-gnu/lib64")

# --- Help FindThreads ---
set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_HAVE_LIBC_PTHREAD ON)
set(CMAKE_USE_PTHREADS_INIT 1)
set(PTHREAD_INCLUDE_DIR ${CMAKE_SYSROOT}/usr/include)
set(PTHREAD_LIBRARY ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/libpthread.so)


# Add include path for aarch64-linux-gnu headers
set(ADDITIONAL_INCLUDE_DIR "${CMAKE_SYSROOT}/usr/include/aarch64-linux-gnu")

# Add to C and C++ compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -I${ADDITIONAL_INCLUDE_DIR}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I${ADDITIONAL_INCLUDE_DIR}" CACHE STRING "" FORCE)