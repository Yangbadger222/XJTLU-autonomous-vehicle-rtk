nmea_navsat_driver
===============

XJTLU Repository Note
---------------------

This package is the upstream NMEA GNSS driver used by the XJTLU autonomous vehicle workspace to publish `/fix` from the serial GNSS receiver. Project-specific device parameters are supplied from `src/bringup/config/master_params.yaml`; preserve upstream driver behavior and attribution when editing.

ROS driver to parse NMEA strings and publish standard ROS NavSat message types. Does not require the GPSD daemon to be running.

API
---

This package has no released Code API.

The ROS API documentation and other information can be found at http://ros.org/wiki/nmea_navsat_driver
