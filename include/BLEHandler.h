#ifndef BLEHANDLER_H
#define BLEHANDLER_H

#include <string>
#include <BLEUtils.h>

namespace BLEHandler
{
	void initBLE(const std::string &deviceName, const std::string &serviceUUID, const std::string &charUUID);
	void sendData(const std::string &data);
}

#endif // BLEHANDLER_H
