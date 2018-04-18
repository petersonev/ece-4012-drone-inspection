## Parrot SDK
set(PARROT_SDK_ROOT ${PROJECT_SOURCE_DIR}/external/parrot_sdk/out/arsdk-native/staging)
link_directories(${PARROT_SDK_ROOT}/usr/lib/)
# manually define the parrot libs, this should later be moved to a FindParrotSDK3.cmake
set(DRONE_INSPECT_LIBS
    arcommands
    arcontroller
    ardatatransfer
    ardiscovery
    armavlink
    armedia
    arnetwork
    arsal
    arstream2
    arstream
    arupdater
    arutils
    ncurses
)
include_directories(${PARROT_SDK_ROOT}/usr/include)
## End Parrot SDK

## SDL2
find_package(SDL2 REQUIRED)


include_directories(${SDL2_INCLUDE_DIR})
set(DRONE_INSPECT_LIBS
    ${DRONE_INSPECT_LIBS}
    ${SDL2_LIBRARY}
)
## End SDL2



# Drone inspection
set(SOURCES FlightControl.cpp)

add_executable(drone_inspection ${SOURCES})

target_link_libraries(drone_inspection ${DRONE_INSPECT_LIBS})
target_link_libraries(drone_inspection path_planner)
