add_definitions(-std=gnu99)

add_library(localizer-renderer SHARED
            renderer_localizer.c)

target_link_libraries(localizer-renderer ${OPENGL_LIBRARIES})

set(REQUIRED_LIBS bot2-vis lcmtypes_localizer geom-utils)

pods_use_pkg_config_packages(localizer-renderer ${REQUIRED_LIBS})

pods_install_headers(renderer_localizer.h DESTINATION
                     localizer_renderer)

pods_install_libraries(localizer-renderer)
pods_install_pkg_config_file(localizer-renderer
    CFLAGS
    LIBS -llocalizer-renderer
    REQUIRES ${REQUIRED_LIBS}
    VERSION 0.0.1)
