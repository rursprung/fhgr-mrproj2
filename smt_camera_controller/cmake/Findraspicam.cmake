include(ExternalProject)
ExternalProject_Add(raspicam
  GIT_REPOSITORY    https://github.com/cedricve/raspicam.git
  CMAKE_ARGS        -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
)

ExternalProject_Get_Property(raspicam INSTALL_DIR)

set(raspicam_INCLUDE_DIR ${INSTALL_DIR}/include)
set(raspicam_LIBRARIES ${INSTALL_DIR}/lib/libraspicam.so ${INSTALL_DIR}/lib/libraspicam_cv.so)

mark_as_advanced(raspicam_INCLUDE_DIR pigpio_LIBRARIES)
