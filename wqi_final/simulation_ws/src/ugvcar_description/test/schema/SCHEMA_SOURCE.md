# ROS Package Schema Cache

Unmodified upstream schemas retrieved on 2026-10-15:

- https://github.com/ros-infrastructure/rep/blob/master/xsd/package_format3.xsd
- https://github.com/ros-infrastructure/rep/blob/master/xsd/package_common.xsd

The local XML catalog maps the package manifest's ROS schema URI to these
files. This keeps the original validation enabled without requiring network
access during CTest. These are test resources, not generated build products.
