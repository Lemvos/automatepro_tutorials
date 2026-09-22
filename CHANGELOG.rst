^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for automatepro_tutorials
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2026-09-22)
------------------
* Implement gnss_heading_node: prints the heading clockwise from north with its accuracy, or reports an invalid heading
* Switch the output off when digital_out_node, digital_drive_out_node, or warning_system_out_node stops on Ctrl+C or SIGTERM, and warn when no IO controller subscribes
* Print imu_node and analog_in_node data at most once per second
* Stop every example cleanly on Ctrl+C
* Drop the unused std_srvs, std_msgs, geometry_msgs, and ublox_msgs dependencies
* Run the ament linters, including the copyright check, on all sources

1.0.0 (2025-07-02)
------------------
* Initial release
