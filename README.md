# A_Hole_Map
Project to detect holes on the street by processing distance and movement sensor data

This project was developped by Pedro Fornari and Lucas Pereira Luiz

We developped a testbanch using systemC, so we can aquire some data from sensors, 
compare with a standard and test innumerous kinds of algorithms before really mount our system on the car again.

The standard was made by using a simple timepiece on the microcontroler and a timepiece on the car,
so every time we went by a hole we checked a point on the timepiece. So the comparision was made by 
checking where the algorithm showed a hole and where it should be, using a margins of 3 seconds of acceptable
error, cause the timepiece marked by hand was not that precisely.

The reader will be able to find more information reading our First Report, which was made testing out 
a matrix of ultrassonic distance sensors, which is only in portuguese by now.
We also developped an Doxyfile for this project, so the reader can have a overview of the code there.
