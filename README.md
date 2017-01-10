This is the simplest possible C++ code for accessing the DGI auxiliary power
interface on the Microchip (formerly Atmel) power debugger.

It simply loads the DLL, initializes the most basic settings, and then fires
off a thread to poll the data buffers, pushing them into some C++ vectors
and writing a file at the end with all the samples.
