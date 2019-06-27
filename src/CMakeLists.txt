add_library(myslam SHARED
        frame.cc
        mappoint.cc
        map.cc
        camera.cc
        config.cc
        feature.cc
        frontend.cc
        backend.cc
        viewer.cc
        visual_odometry.cc
        dataset.cc)

target_link_libraries(myslam
        ${THIRD_PARTY_LIBS})
