#include "Comms.h"

Comms::Comms(){
	enumerate_ports();

	serial = NULL;
	bufferIndex = 0;
}

void Comms::begin() {
	if(!maintainConnection()){
		std::cout << "Could not connect to serial device\n";
	}
}

void Comms::end() {
	serial = NULL;
}

bool Comms::isEnded() {
    return serial == NULL;
}

const RobotIn& Comms::getRobotIn() {
    return in;
}

RobotOut& Comms::getRobotOut() {
    return out;
}

void Comms::setRobotOut(const RobotOut &newStruct) {
    out = newStruct;
}

bool Comms::read(){
	if(serial == NULL){
        return false;
	}
    size_t size = serial->available();
	size = size + bufferIndex > BUF_SIZE ? BUF_SIZE - bufferIndex : size;
	//std::cout << "size=" << size << std::endl;

	// if we get less than a full message, write output again
	if(size < sizeof(RobotIn)+2 - bufferIndex){
		setOutBuf();
		serial->write(outBuf, sizeof(outBuf));
		return false;
	}

	// read from serial
	serial->read(readBuf + bufferIndex, size);
	size = size + bufferIndex;
	bufferIndex = 0;

	// if the start byte is not first we have a problem
	while(readBuf[0] != 0xdd || readBuf[sizeof(RobotIn)+1] != crc8.compute(&readBuf[1], sizeof(RobotIn))){
		// attempt to recover
		// find next start byte
		size_t i;
		for(i = 1; i<=size-sizeof(RobotIn)-2; i++)
			if(readBuf[i] == 0xdd)
				break;

		if(i > size - sizeof(RobotIn) - 2){
			// recovery failed
			for(; i < size; i++)
				if(readBuf[i] == 0xdd)
					break;
			// copy new possible start byte to the start of the buffer
			if(i < size){
				for(size_t j = i; j<size; j++){
					readBuf[j - i] = readBuf[j];
				}
				bufferIndex = size - i;
			}

			// write again
			setOutBuf();
			serial->write(outBuf, sizeof(outBuf));
			return false;
		} else{
			// found possible start byte, attempt to read rest of message
			for(size_t j = i; j<size; j++){
				readBuf[j - i] = readBuf[j];
			}
		}
		size -= i;
	}

	// set RobotIn vars
	uint16_t* temp = (uint16_t*)&readBuf[1];
	in.waist = *temp;
	printf("FL = %d\n", in.waist);
	in.shoulder = *(temp + 1);
	in.elbow = *(temp + 2);
	//if (pot < 1024)
	//	in.shoulder = pot;

	// throw away the rest of the serial input
	while(size = serial->available()){
		size = size > BUF_SIZE ? BUF_SIZE : size;
		serial->read(readBuf, size);
	}
	return true;
}

bool Comms::write(){
    if (serial == NULL)
        return false;

	// read and throw away everything from serial
	size_t size;
	uint8_t buffer[BUF_SIZE];
	while(size = serial->available()){
		size = size > BUF_SIZE ? BUF_SIZE : size;
		serial->read(buffer, size);
	}

	// write to serial
	setOutBuf();
	size_t bytesWritten = serial->write(outBuf, sizeof(outBuf));

	// check if write succeeded
	if(bytesWritten != sizeof(outBuf)){
		serial->close();
		serial = NULL;
		std::cout << "Connection lost during write\n";
        return false;
	}

	return true;
}

void Comms::setOutBuf(){
	outBuf[0] = 0xdd;
	outBuf[1] = out.driveFL;
	outBuf[2] = out.driveBL;
	outBuf[3] = out.driveFR;
	outBuf[4] = out.driveBR;
	outBuf[5] = out.waist;
	outBuf[6] = out.shoulder;
	outBuf[7] = out.elbow;
	outBuf[8] = out.wrist;
	outBuf[9] = out.vacuum;
	outBuf[10] = crc8.compute(&outBuf[1], 9);
}

bool Comms::maintainConnection(){
	if(serial == NULL){
		std::vector<PortInfo> devices_found = list_ports();
		for(std::vector<PortInfo>::iterator it = devices_found.begin(); it != devices_found.end(); ++it){
			if (it->hardware_id.find("PID_6015") != std::string::npos){ // look for product ID
				std::cout << "Trying to connect to port " << it->port << ": " << it->description << std::endl;
				serial = new Serial(it->port, BAUD_RATE, serial::Timeout::simpleTimeout(TIMEOUT));

				if(serial->isOpen()){
					std::cout << "Connection successful\n";
					return true;
				} else{
					std::cout << "Connection unsuccessful\n";
					serial = NULL;
				}
			}
		}
		return false;
	}
	if(serial->isOpen()){
		return true;
	}else{
		serial = NULL;
		std::cout << "Connection lost\n";
		return false;
	}
}

void Comms::enumerate_ports(){
	std::vector<PortInfo> devices_found = list_ports();
	for(std::vector<PortInfo>::iterator it = devices_found.begin(); it != devices_found.end(); ++it){
		std::cout << it->port << ", " << it->description << ", " << it->hardware_id << std::endl;
	}
}