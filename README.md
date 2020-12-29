# Thesis: Drones for Intralogistics using IoT
An autonomous inventorying system based on a smart drone.

Made for DJI Tello EDU drone and a USB keyboard emulation RFID Reader.

# Usage:
Execute comp_com.py using Python 3
Execute comp_nav.py using Python 2 after comp_com.py "Launch Drone" option is selected.

# Features:
Autonomous navigation:
  1) Configurable Virtual Coordinates Map based on (x,y) coordinate sets. Given a set of nodes representing the intersections between a series of shelves, a single-branch minimum distance 3D route is plotted to cover the entire warehouse Reserve Area.
  2) Perimeter selection to limit the drone's range through the Virtual Coordinates Map class.
  3) OpenCV drone positioning adjustment through use of Visual Markers on the walls at the edges of each aisle.
  4) RFID confirmation of the drone's physical position.
  5) Calibration options for the OpenCV implementation, drone movement and warehouse dimensions.

Inventorying system:
  1) The inventorying system parses item or item storage unti RFID tag information and updates a locally stored database with their count and flags any misplaced items.
  2) Direct access to the local database through a user interface.
  3) Mock-up FTP implementation for use with a warehouse database network access point.
