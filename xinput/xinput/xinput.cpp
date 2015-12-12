// xinput.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include "xinput.h"
#include <string>
#include "SerialClass.h"


int _tmain(int argc, _TCHAR* argv[])
{
	CXBOXController* Player1;
	Serial* SP = new Serial("\\\\.\\COM3");
	Player1 = new CXBOXController(1);

	std::cout << "Instructions:\n";
	std::cout << "[A] Vibrate Left Only\n";
	std::cout << "[B] Vibrate Right Only\n";
	std::cout << "[X] Vibrate Both\n";
	std::cout << "[Y] Vibrate Neither\n";
	std::cout << "[BACK] Exit\n";
	if (SP->IsArduinoConnected())
		std::cout << "Arduino Connected" << std::endl;

	struct package{
		short LX;
		short LY;
		short RX;
		short RY;
		short leftTrigger;
		short rightTrigger;
		int p;
		int i;
		short d;
		short crc;
	};
	struct package pack;
	pack.p = 0;
	pack.i = 0;
	pack.d = 0;
	while (true)
	{
			if (Player1->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_A)
			{
				//Player1->Vibrate(65535, 0);
				if (pack.p < 254)pack.p += 1;
			}

			if (Player1->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_B)
			{
				//Player1->Vibrate(0, 65535);
				if(pack.p >= 1) pack.p -= 1;
			}

			if (Player1->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_X)
			{
				//Player1->Vibrate(65535, 65535);
				if (pack.i < 254) pack.i += 1;
			}

			if (Player1->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_Y)
			{
				//Player1->Vibrate();
				if (pack.i > 0) pack.i -= 1;
			}
			if (Player1->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_UP)
			{
				//Player1->Vibrate(65535, 65535);
				if (pack.d < 254) pack.d++;
			}

			if (Player1->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_DOWN)
			{
				//Player1->Vibrate();
				if (pack.d > 0) pack.d--;
			}

			if (Player1->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_BACK)
			{
				while (true) {
					if (Player1->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_START) break;
				}
			}
			
			
			// przypisanie i remapping
			pack.LX = (Player1->GetState().Gamepad.sThumbLX / 1092) + 30;
			if (pack.LX > 27 && pack.LX < 33) pack.LX = 30;
			else if (pack.LX < 30) pack.LX += 2;
			else pack.LX -= 2;
			pack.LY = (Player1->GetState().Gamepad.sThumbLY / 1092) + 30;
			if (pack.LY > 27 && pack.LY < 33) pack.LY = 30;
			else if (pack.LY < 30) pack.LY += 2;
			else pack.LY -= 2;
			pack.RX = (Player1->GetState().Gamepad.sThumbRX / 1092) + 30;
			if (pack.RX > 27 && pack.RX < 33) pack.RX = 30;
			else if (pack.RX < 30) pack.RX += 2;
			else pack.RX -= 2;
			pack.RY = (Player1->GetState().Gamepad.sThumbRY / 1092) + 30;
			if (pack.RY > 27 && pack.RY < 33) pack.RY = 30;
			else if (pack.RY < 30) pack.RY += 2;
			else pack.RY -= 2;
			pack.leftTrigger = Player1->GetState().Gamepad.bLeftTrigger;
			pack.rightTrigger = Player1->GetState().Gamepad.bRightTrigger;
			pack.crc = pack.LX + pack.LY + pack.RX + pack.RY + pack.leftTrigger;
			// preambu³a pakietu
			char data = 0xAB;
			// wysy³anie danych
			SP->WriteData(&data, 1);
			SP->WriteData(&pack.LX, sizeof(pack.LX));
			SP->WriteData(&pack.LY, sizeof(pack.LY));
			SP->WriteData(&pack.RX, sizeof(pack.RX));
			SP->WriteData(&pack.RY, sizeof(pack.RY));
			SP->WriteData(&pack.leftTrigger, sizeof(pack.leftTrigger));
			SP->WriteData(&pack.rightTrigger, sizeof(pack.rightTrigger));
			SP->WriteData(&pack.p, sizeof(pack.p));
			SP->WriteData(&pack.i, sizeof(pack.i));
			SP->WriteData(&pack.d, sizeof(pack.d));
			SP->WriteData(&pack.crc, sizeof(pack.crc));
			// wyœwietlenie danych z kontrolera
			std::system("cls");
			std::cout << "QUADCOPTER REMOTE v0.3\n";
			std::cout << "Pitch: " << pack.LY << "\n";
			std::cout << "Roll: " << pack.LX << "\n";
			std::cout << "Yaw: " << pack.RX << "\n";
			std::cout << "Throttle: " << pack.leftTrigger << "\n";
			std::cout << "P: " << pack.p/100.0 << "\nI: " << pack.i/100.0 << "\nD: " << pack.d/100.0 << std::endl;
	}

	delete(Player1);


	return 0;
}



CXBOXController::CXBOXController(int playerNumber)
{
	// Set the Controller Number
	_controllerNum = playerNumber - 1;
}

XINPUT_STATE CXBOXController::GetState()
{
	// Zeroise the state
	ZeroMemory(&_controllerState, sizeof(XINPUT_STATE));

	// Get the state
	XInputGetState(_controllerNum, &_controllerState);

	return _controllerState;
}

bool CXBOXController::IsConnected()
{
	// Zeroise the state
	ZeroMemory(&_controllerState, sizeof(XINPUT_STATE));

	// Get the state
	DWORD Result = XInputGetState(_controllerNum, &_controllerState);

	if (Result == ERROR_SUCCESS)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void CXBOXController::Vibrate(int leftVal, int rightVal)
{
	// Create a Vibraton State
	XINPUT_VIBRATION Vibration;

	// Zeroise the Vibration
	ZeroMemory(&Vibration, sizeof(XINPUT_VIBRATION));

	// Set the Vibration Values
	Vibration.wLeftMotorSpeed = leftVal;
	Vibration.wRightMotorSpeed = rightVal;

	// Vibrate the controller
	XInputSetState(_controllerNum, &Vibration);
}


Serial::Serial(char *portName)
{
	//We're not yet connected
	this->connected = false;

	//Try to connect to the given port throuh CreateFile
	this->hSerial = CreateFile(portName,
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		NULL);

	//Check if the connection was successfull
	if (this->hSerial == INVALID_HANDLE_VALUE)
	{
		//If not success full display an Error
		if (GetLastError() == ERROR_FILE_NOT_FOUND){

			//Print Error if neccessary
			printf("ERROR: Handle was not attached. Reason: %s not available.\n", portName);

		}
		else
		{
			printf("ERROR!!!");
		}
	}
	else
	{
		//If connected we try to set the comm parameters
		DCB dcbSerialParams = { 0 };

		//Try to get the current
		if (!GetCommState(this->hSerial, &dcbSerialParams))
		{
			//If impossible, show an error
			printf("failed to get current serial parameters!");
		}
		else
		{
			//Define serial connection parameters for the arduino board
			dcbSerialParams.BaudRate = CBR_115200;
			dcbSerialParams.ByteSize = 8;
			dcbSerialParams.StopBits = ONESTOPBIT;
			dcbSerialParams.Parity = NOPARITY;

			//Set the parameters and check for their proper application
			if (!SetCommState(hSerial, &dcbSerialParams))
			{
				printf("ALERT: Could not set Serial Port parameters");
			}
			else
			{
				//If everything went fine we're connected
				this->connected = true;
				//We wait 2s as the arduino board will be reseting
				Sleep(ARDUINO_WAIT_TIME);
			}
		}
	}

}

Serial::~Serial()
{
	//Check if we are connected before trying to disconnect
	if (this->connected)
	{
		//We're no longer connected
		this->connected = false;
		//Close the serial handler
		CloseHandle(this->hSerial);
	}
}

int Serial::ReadData(char *buffer, unsigned int nbChar)
{
	//Number of bytes we'll have read
	DWORD bytesRead;
	//Number of bytes we'll really ask to read
	unsigned int toRead;

	//Use the ClearCommError function to get status info on the Serial port
	ClearCommError(this->hSerial, &this->errors, &this->status);

	//Check if there is something to read
	if (this->status.cbInQue>0)
	{
		//If there is we check if there is enough data to read the required number
		//of characters, if not we'll read only the available characters to prevent
		//locking of the application.
		if (this->status.cbInQue>nbChar)
		{
			toRead = nbChar;
		}
		else
		{
			toRead = this->status.cbInQue;
		}

		//Try to read the require number of chars, and return the number of read bytes on success
		if (ReadFile(this->hSerial, buffer, toRead, &bytesRead, NULL) && bytesRead != 0)
		{
			return bytesRead;
		}

	}

	//If nothing has been read, or that an error was detected return -1
	return -1;

}


bool Serial::WriteData(short *buffer, unsigned int nbChar)
{
	DWORD bytesSend;

	//Try to write the buffer on the Serial port
	if (!WriteFile(this->hSerial, (void *)buffer, nbChar, &bytesSend, 0))
	{
		//In case it don't work get comm error and return false
		ClearCommError(this->hSerial, &this->errors, &this->status);

		return false;
	}
	else
		return true;
}

bool Serial::WriteData(char *buffer, unsigned int nbChar)
{
	DWORD bytesSend;

	//Try to write the buffer on the Serial port
	if (!WriteFile(this->hSerial, (void *)buffer, nbChar, &bytesSend, 0))
	{
		//In case it don't work get comm error and return false
		ClearCommError(this->hSerial, &this->errors, &this->status);

		return false;
	}
	else
		return true;
}

bool Serial::WriteData(int *buffer, unsigned int nbChar)
{
	DWORD bytesSend;

	//Try to write the buffer on the Serial port
	if (!WriteFile(this->hSerial, (void *)buffer, nbChar, &bytesSend, 0))
	{
		//In case it don't work get comm error and return false
		ClearCommError(this->hSerial, &this->errors, &this->status);

		return false;
	}
	else
		return true;
}

bool Serial::IsArduinoConnected()
{
	//Simply return the connection status
	return this->connected;
}