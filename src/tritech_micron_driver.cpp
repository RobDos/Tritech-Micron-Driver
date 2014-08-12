// ######################################################################
//
//      TritechMicronDriver - A protocol parser for Tritech Micron sonars.
//      Copyright (C) 2011  Randolph Voorhies
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// ######################################################################

#include <micron_driver_ros/tritech_micron_driver.h>
#include <micron_driver_ros/message_types.h>
#include <iostream>
#include <algorithm>
#include <chrono>
#include "stdint.h"
    
using namespace tritech;

// ######################################################################
TritechMicronDriver::TritechMicronDriver(uint16_t _nBins, float _range, float _VOS, uint8_t _angleStepSize, int _leftLimit, int _rightLimit, bool debugMode) :
  hasHeardMtAlive(false),hasHeardMtVersionData(false), hasHeardMtHeadData(false),itsDebugMode(debugMode),stateMachineSemaphore(green), state(waitingforMtAlive_1)
{ 

resetMessage();   
setParameters( _nBins,  _range,  _VOS,  _angleStepSize, _leftLimit, _rightLimit);

}




// ######################################################################
TritechMicronDriver::~TritechMicronDriver()
{ disconnect(); }

// ######################################################################
void TritechMicronDriver::disconnect()
{
  std::cout << "Disconnecting" << std::endl;
  itsRunning = false;
  if(itsSerialThread.joinable())
  {
    std::cout << "Joining..."; 
    try { itsSerialThread.detach(); }
    catch(...) {/*screw you!*/}
    std::cout << "Joined"; 
  }

  std::cout << "Disconnected" << std::endl;
}

// ######################################################################

void TritechMicronDriver::setParameters(uint16_t _nBins, float _range, float _VOS, uint8_t _angleStepSize, int _leftLimit, int _rightLimit) {
   nBins = _nBins;
   range = _range;
   VOS = _VOS;
   angleStepSize = _angleStepSize;
   leftLimit= _leftLimit; 
   rightLimit= _rightLimit;
}

// ######################################################################
bool TritechMicronDriver::connect(std::string const& devName)
{
  if(itsDebugMode) std::cout << "Connecting...";

  itsSerial.flush();

  bool connectionSuccess = itsSerial.connect(devName, 115200); 
  if(!connectionSuccess)
  {
    std::cerr << "Could not connect to serial port!" << std::endl;
    return false;
  }

  if(itsDebugMode) std::cout << "Connected" << std::endl;

  sleep(1);
  itsSerial.writeVector(mtRebootMsg);

  itsRunning = true;
  itsSerialThread = boost::thread(std::bind(&TritechMicronDriver::serialThreadMethod, this));

  itsProcessingThread = boost::thread(std::bind(&TritechMicronDriver::processingThreadMethod, this));

  return true;
}

// ######################################################################
void TritechMicronDriver::configure()
{
  mtHeadCommandMsg headCommandMsg(nBins, range, VOS, angleStepSize, leftLimit, rightLimit);
  itsSerial.writeVector(headCommandMsg.construct());
}

void TritechMicronDriver::reconfigure(uint16_t _nBins, float _range, float _VOS, uint8_t _angleStepSize, int _leftLimit, int _rightLimit) 
{
	//Load the new values
	setParameters( _nBins,  _range,  _VOS,  _angleStepSize, _leftLimit, _rightLimit);
	//Set the semaphore to red in order not to access the state variable at the same time 
	stateMachineSemaphore = red;
	//Set the state variable to configure 
	state = configuring; 

	//Set the semaphore to green to allow the driver to continue
	stateMachineSemaphore = green; 

}

// ######################################################################
void TritechMicronDriver::registerScanLineCallback(std::function<void(float, float, std::vector<uint8_t>)> callback)
{ itsScanLineCallback = callback; }

// ######################################################################
void TritechMicronDriver::serialThreadMethod()
{
  while(itsRunning == true)
  {
    std::vector<uint8_t> bytes = itsSerial.read(2048);
    if(bytes.size() > 0)
    {
      for(size_t i = 0; i < bytes.size(); ++i)
        processByte(bytes[i]); 
	//std::cout << " Finished processing read Vector";
    }
    else { 
		usleep(100000);
	  	//std::cout << std::endl << "No Serial input" << std::endl; 
	 }
  } 
  std::cout << "serialThreadMethod Finished" <<std::endl;
}

// ######################################################################

void TritechMicronDriver::processingThreadMethod()
{
	

  int waitedPeriods = 0; 
  Semaphore firstSemaphore = green; 

  while (itsRunning) {
	if (stateMachineSemaphore == green) {
		//std::cout << "In loop: " <<std::endl; 
		switch(state) {
		case waitingforMtAlive_1: //Waiting for MtAlive
	  		while(!hasHeardMtAlive) {
 			sleep(1);
			std::cout << "Waiting: " <<std::endl; 
			}
			if(itsDebugMode) std::cout << "----------Received mtAlive----Case 1------" <<std::endl;
			state = versionData; 
			break;
		case versionData: //Waiting for MtVersion Data
			while(!hasHeardMtVersionData)
		  	{
				itsSerial.writeVector(mtSendVersionMsg);
				sleep(1);
	  		}
			if(itsDebugMode) std::cout << "----------Received mtVersionData----Case 2------" <<std::endl;
			state = waitingforMtAlive_2;
			break; 
		case  waitingforMtAlive_2: //Waiting for MtAlive 
			hasHeardMtAlive = false; 
		 	while(!hasHeardMtAlive) sleep(1);
			if(itsDebugMode) std::cout << "----------Received mtAlive----Case 3------" <<std::endl;
			if (hasParams) {
				state = scanning;
			}
			else {
				state = configuring;  
			}
			break;
		case configuring: //Configure the Sonar 
			if(itsDebugMode) std::cout << "----------Configuring Sonar----Case 4------" <<std::endl;		
			configure();
			if(itsDebugMode) std::cout << "Changing to State 3" <<std::endl;
			sleep(5);
			state = waitingforMtAlive_2; 
			break;
		case scanning: //Send mtSend Data  
			
			if (firstSemaphore == green) {
				firstSemaphore = red; 
				if(itsDebugMode) std::cout << "----------Scanning: ...   ----Case 5------" <<std::endl;
				sleep(1);
				itsSerial.writeVector(mtSendDataMsg);
			}			
			

			if (waitedPeriods > 15) {
				if(itsDebugMode) std::cout << "----------Scanning: Resending request" <<std::endl;
				itsSerial.writeVector(mtSendDataMsg);
				//usleep(100000); 
				sleep(1);
			}
			else {
				waitedPeriods++; 
				usleep(100000);
			}

			
			if (hasHeardMtHeadData) {
				waitedPeriods = 0; 
				hasHeardMtHeadData = false; 
			}
			break;

			//Transition to configuring in case the sonar reports lacking parameters
			if (!hasParams) {
				state = configuring;
			}
  		}
	}
	else {usleep (10);}
  }
 
  std::cout << "processingThreadMethod Finished" <<std::endl;
}


// ######################################################################
void TritechMicronDriver::resetMessage()
{
  itsMsg = Message();
  itsState = WaitingForAt;
  itsRawMsg.clear();
}

// ######################################################################
void TritechMicronDriver::processByte(uint8_t byte)
{

 // if(itsDebugMode) std::cout << std::endl << "Received Byte: " << std::hex << int(byte) << std::dec ;
  itsRawMsg.push_back(byte);

 if(itsState == WaitingForAt) 
  {
    if(byte == '@')
    {
      itsRawMsg.clear();
      // Tritech's datasheet refers to the first byte as '1' rather than '0', so let's push back a bogus byte here just
      // to make reading the datasheet easier.
      itsRawMsg.push_back(0);
      itsRawMsg.push_back(byte);
      itsState = ReadingHeader; 
      itsMsg = Message();
      return;
    }
    else if(itsDebugMode) std::cout <<" bogus byte: " << std::hex << int(byte) << std::dec;//<< std::endl;
  }
   //std::cout << std::endl;
  

  if(itsState == ReadingHeader)
  {
    // Ignore the 'Hex Length' section
    if(itsRawMsg.size() < 7) return;

    if(itsRawMsg.size() == 7)  { itsMsg.binLength  = uint16_t(byte);      return; }
    if(itsRawMsg.size() == 8)  { 
		itsMsg.binLength |= uint16_t(byte) << 8;
		if (itsMsg.binLength > 1000) {
			if(itsDebugMode) std::cout  <<" Message length too big!" << std::endl; 
			resetMessage(); 
		}
		return; 
		}
    if(itsRawMsg.size() == 9)  { itsMsg.txNode = byte; return; }
    if(itsRawMsg.size() == 10) { itsMsg.rxNode = byte; return; }
    if(itsRawMsg.size() == 11) { itsMsg.count  = byte; return; }
    if(itsRawMsg.size() == 12) 
    {
      itsMsg.type = MessageType(byte);
      itsState = ReadingData;
      return; 
    }

    std::cerr << "Parsing error! " << __FILE__ << ":" << __LINE__ << std::endl;
    resetMessage();
    return;
  }

  if(itsState == ReadingData)
  {
	//if(itsDebugMode) std::cout <<"  Remaining bytes: "<< std::dec  << uint16_t(itsMsg.binLength - (itsRawMsg.size() - 7)) << std::dec; 
    if(uint16_t(itsMsg.binLength - (itsRawMsg.size() - 7)) == 0)
    {
      if(byte == 0x0A)
      {
        itsMsg.data = itsRawMsg;
        processMessage(itsMsg);
        resetMessage();
      }
      else
      {
        if(itsDebugMode) std::cout << " Message finished, but no LF detected!";
        resetMessage();
        return;
      }
    }
  }
}

// ######################################################################
void TritechMicronDriver::processMessage(tritech::Message msg)
{
  if(msg.type == mtVersionData)
  {
    mtVersionDataMsg parsedMsg(msg);
    hasHeardMtVersionData = true;

    if(itsDebugMode) std::cout << std::endl << "Received mtVersionData Message" << std::endl;
    if(itsDebugMode) parsedMsg.print(); 
  }
  else if(msg.type == mtAlive)
  {
    mtAliveMsg parsedMsg(msg);
    hasHeardMtAlive = true;
	hasParams = !parsedMsg.noParams; 

    if(itsDebugMode) std::cout << std::endl << "Received mtAlive Message" << std::endl;
    if(itsDebugMode) parsedMsg.print();
  }
  else if(msg.type == mtHeadData)
  {
    mtHeadDataMsg parsedMsg(msg);
	hasHeardMtHeadData = true; 

	
    if(state==scanning){
		itsSerial.writeVector(mtSendDataMsg);
	}

    float range_meters;
    switch(parsedMsg.rangeUnits)
    {
      case mtHeadDataMsg::meters:  range_meters = parsedMsg.rangeScale;          break;
      case mtHeadDataMsg::feet:    range_meters = parsedMsg.rangeScale * 0.3048; break;
      case mtHeadDataMsg::fathoms: range_meters = parsedMsg.rangeScale * 1.8288; break;
      case mtHeadDataMsg::yards:   range_meters = parsedMsg.rangeScale * 0.9144; break;
    }

    float metersPerBin = range_meters / parsedMsg.scanLine.size();
    if(itsScanLineCallback)
      itsScanLineCallback(parsedMsg.bearing_degrees, metersPerBin, parsedMsg.scanLine);

    if(itsDebugMode) std::cout << std::endl<< "Received mtHeadData Message" << std::endl;
    if(itsDebugMode) parsedMsg.print(); 
  }
  else if(msg.type == mtBBUserData)
  { if(itsDebugMode) std::cout << std::endl<< "Received mtBBUserData Message" << std::endl; }
  else
  { std::cerr << "Unhandled Message Type: " << msg.type << std::endl; }
}

