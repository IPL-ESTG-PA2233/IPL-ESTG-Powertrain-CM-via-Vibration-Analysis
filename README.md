# IPL-ESTG-Powertrain-CM-via-Vibration-Analysis

Brief Description
-------------------
This is part of the Final Project of the Bachelor's degree in Automotive Engineering, 2021/22 academic year, of the School of Technology and Management of the Polytechnic Institute of Leiria, Portugal.

The project  aims to contribute to the development of a low-cost, versatile, easily attachable/detachable, user-friendly equipment using commercial off-the-shelf components and an open-source software platform for reliable engine vibration data acquisitions.  

The hardware used was a IIM-42352 a 3-axis MEMS acelerometer from TDK InvenSense and a ESP-WROOM-32 Microcontroller

Repository Contents
-------------------
The entire project's code was developed in two distinct programming languages: C++ and Python. The C++ component manages all hardware functionality, while the Python component collects, processes, and displays the vibration data. 

This github has the migrated IIM-42352 library, hence, it is now ESP32-compatible as well as the C++ code used in the project.

The Python scripts are also available:

file_parser.py is responsible of creating the files with the raw data and respective timestamps.

data_process_and_display.py is responsible for processing the raw data and ploting it using FFT and Spectrograms.

Documentation
--------------
IIM-42352 Datasheet: https://invensense.tdk.com/wp-content/uploads/2022/04/DS-000442-IIM-42352-TYP-v1.2.pdf

IIM-42352 Schematic: https://invensense.tdk.com/download-pdf/iim-42652-iim-42352-and-iim-42351-evb-schematic/


Version History
---------------
Current version 1.0.0


Developers
---------------
Luís Barbosa 

Nuno Pereira
