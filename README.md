AQE_Calibrate
=============

A sketch you can use to calibrate your Egg

The sketch implements a console program (@9600 8N1, using Putty or similar) that allows you to change the R0 value for any sensor on the Egg Bus. In addition to this sketch, you'll need:

* The sensor data sheet to map the current ppm to R/R0
* An idea what the true concentration is where you are
* Some patience to make a judgment about sensor stability
