# PX4 Gazebo Classic calls QT5_WRAP_CPP for its user camera plugin even when
# the optional GStreamer branch (which normally finds Qt5) is disabled.
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Test)
