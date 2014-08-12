#
# Makefile for Tritech Micron Driver
#
# Author: Eduardo Iscar 
# email: eduardoiscar@gmail.com 
# Date: 11.06.2014
#
# Compiler: G++
# 

build : Serial.o tritech_micron_driver.o main.o
	g++ -o myProgram tritech_micron_driver.o Serial.o main.o -L/usr/local/lib/ -lboost_thread

Serial.o : Serial.cpp
	g++ -c -std=c++0x Serial.cpp 
	
tritech_micron_driver.o : tritech_micron_driver.cpp 
	g++ -c -std=c++0x tritech_micron_driver.cpp 

main.o : main.cpp
	g++ -c -std=c++0x main.cpp	

clean:
	rm -f *.o core

rebuild: clean build
