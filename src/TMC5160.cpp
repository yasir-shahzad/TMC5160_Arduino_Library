

#include "TMC5160.h"

TMC5160::TMC5160(uint32_t fclk) : _fclk(fclk)
{
}

TMC5160::~TMC5160()
{
    ;
}

bool TMC5160::begin()
{
    bool retVal = false;

    /* Clear the reset and charge pump undervoltage flags */
    globalStatus.reset = true;
    globalStatus.uv_cp = true;
    writeRegister(ADDRESS_GSTAT, globalStatus.bytes);

    drvconf.drvstrength = 2;
    drvconf.bbmtime = 0;
    drvconf.bbmclks = 4;
    writeRegister(ADDRESS_DRV_CONF, drvconf.bytes);

    setCurrentMilliamps(1600);
    setModeChangeSpeeds(170, 0, 0);

    // Set initial PWM values
    pwmconf.bytes = 0xC40C001E;   // Reset default
    pwmconf.pwm_autoscale = true; // Temp to set OFS and GRAD initial values
    if (_fclk > DEFAULT_F_CLK)
        pwmconf.pwm_freq = 0;
    else
        pwmconf.pwm_freq = 0b01; // recommended : 35kHz with internal typ. 12MHZ clock. 0b01 => 2/683 * f_clk
    pwmconf.pwm_grad = 0;
    pwmconf.pwm_ofs =30;
    pwmconf.freewheel = FREEWHEEL_NORMAL;

    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_autograd = true;
    writeRegister(ADDRESS_PWMCONF, pwmconf.bytes);

    // Recommended settings in quick config guide
    chopConf.toff = 2;
    chopConf.tbl = 0;
    chopConf.hstrt_tfd = 7;
    chopConf.hend_offset = 7;
    chopConf.mres = 0;
    chopConf.chm = 0;
    chopConf.tpfd = 0;
    writeRegister(ADDRESS_CHOPCONF, chopConf.bytes);
    setRampMode(VELOCITY_MODE);

    globalConfig.en_pwm_mode = true; // Enable stealthChop PWM mode
    globalConfig.multistep_filt = true;
    //  globalConfig.shaft = 0;  // 1 to invert the motor direction
    writeRegister(ADDRESS_GCONF, globalConfig.bytes);

    if (globalConfig.bytes == readRegister(ADDRESS_GCONF))
    {
        retVal = true;
    }

    // Set default start, stop, threshold speeds.
    setRampSpeeds(50, 0, 0); // Start, stop, threshold speeds */

    return (retVal);
}


void TMC5160::setRampMode(RampMode mode)
{
    switch (mode)
    {
    case POSITIONING_MODE:
        writeRegister(ADDRESS_RAMPMODE, POSITIONING_MODE);
        break;

    case VELOCITY_MODE:
        setMaxSpeed(
            0); // There is no way to know if we should move in the positive or negative direction => set speed to 0.
        writeRegister(ADDRESS_RAMPMODE, VELOCITY_MODE_POS);
        break;

    case HOLD_MODE:
        writeRegister(ADDRESS_RAMPMODE, HOLD_MODE);
        break;
    }

    _currentRampMode = mode;
}

float TMC5160::getCurrentPosition()
{
    int32_t uStepPos = readRegister(ADDRESS_XACTUAL);

    if ((uint32_t)(uStepPos) == 0xFFFFFFFF)
        return NAN;
    else
        return (float)uStepPos / (float)_uStepCount;
}

float TMC5160::getEncoderPosition()
{
    int32_t uStepPos = readRegister(ADDRESS_X_ENC);

    if ((uint32_t)(uStepPos) == 0xFFFFFFFF)
        return NAN;
    else
        return (float)uStepPos / (float)_uStepCount;
}

void TMC5160::invertDriver(bool invert)
{
    globalConfig.bytes = readRegister(ADDRESS_GCONF);

    if (invert)
    {
        globalConfig.bytes |= (1 << 4);
    }
    else
    {
        globalConfig.bytes &= ~(1 << 4);
    }

    writeRegister(ADDRESS_GCONF, globalConfig.bytes);
}

float TMC5160::getLatchedPosition()
{
    int32_t uStepPos = readRegister(ADDRESS_XLATCH);

    if ((uint32_t)(uStepPos) == 0xFFFFFFFF)
        return NAN;
    else
        return (float)uStepPos / (float)_uStepCount;
}

float TMC5160::getLatchedEncoderPosition()
{
    int32_t uStepPos = readRegister(ADDRESS_ENC_LATCH);

    if ((uint32_t)(uStepPos) == 0xFFFFFFFF)
        return NAN;
    else
        return (float)uStepPos / (float)_uStepCount;
}

float TMC5160::getTargetPosition()
{
    int32_t uStepPos = readRegister(ADDRESS_XTARGET);

    if ((uint32_t)(uStepPos) == 0xFFFFFFFF)
        return NAN;
    else
        return (float)uStepPos / (float)_uStepCount;
}

float TMC5160::getCurrentSpeed()
{
    uint32_t data = readRegister(ADDRESS_VACTUAL);

    if (data == 0xFFFFFFFF)
        return NAN;

    // Convert 24-bit signed integer to 32-bit signed integer
    if (bitRead(data, 23))  // if negative, sign-extend
        data |= 0xFF000000;

    return speedToHz(data);
}


void TMC5160::setCurrentPosition(float position, bool updateEncoderPos)
{
    writeRegister(ADDRESS_XACTUAL, (int)(position * (float)_uStepCount));

    if (updateEncoderPos)
    {
        writeRegister(ADDRESS_X_ENC, (int)(position * (float)_uStepCount));
        clearEncoderDeviationFlag();
    }
}

void TMC5160::setTargetPosition(float position)
{
    writeRegister(ADDRESS_XTARGET, (int32_t)(position * (float)_uStepCount));
}
unsigned long Starttime = 0;
int count = 0;
void TMC5160::setMaxSpeed(float speed)
{
    if (speed != 0)
    {
        count++;
        if (millis() - Starttime > 3000)
        {
            writeRegister(ADDRESS_GLOBAL_SCALER, constrain(136, 32, 256));
        }
    }
    if (speed == 0)
    {
        writeRegister(ADDRESS_GLOBAL_SCALER, constrain(174, 32, 256));
        count = 0;
    }
    if (count == 0)
    {
        Starttime = millis();
    }
    speed = 1.666666667 * speed;
    writeRegister(ADDRESS_VMAX, speedFromHz(fabs(speed)));

    if (_currentRampMode == VELOCITY_MODE)
    {
        writeRegister(ADDRESS_RAMPMODE,
                      speed < 0.0f ? VELOCITY_MODE_NEG : VELOCITY_MODE_POS);
    }
}

void TMC5160::setRampSpeeds(float startSpeed, float stopSpeed, float transitionSpeed)
{
    writeRegister(ADDRESS_VSTART, speedFromHz(fabs(startSpeed)));
    writeRegister(ADDRESS_VSTOP, speedFromHz(fabs(stopSpeed)));
    writeRegister(ADDRESS_V_1, speedFromHz(fabs(transitionSpeed)));
}

void TMC5160::setAcceleration(float maxAccel)
{
    writeRegister(ADDRESS_AMAX, accelFromHz(fabs(maxAccel)));
}

void TMC5160::setAccelerations(float maxAccel, float startAccel, float maxDecel, float finalDecel)
{
    writeRegister(ADDRESS_DMAX, accelFromHz(fabs(maxDecel)));
    writeRegister(ADDRESS_AMAX, accelFromHz(fabs(maxAccel)));
    writeRegister(ADDRESS_A_1, accelFromHz(fabs(startAccel)));
    writeRegister(ADDRESS_D_1, accelFromHz(fabs(finalDecel)));
}

/**
 *
 * @see Datasheet rev 1.15, section 6.3.2.2 "ADDRESS_RAMP_STAT - Ramp & Reference Switch Status Register".
 * @return true if the target position has been reached, false otherwise.
 */
bool TMC5160::isTargetPositionReached(void)
{
    rampStatus.bytes = readRegister(ADDRESS_RAMP_STAT);
    return rampStatus.position_reached ? true : false;
}

/**
 *
 * @see Datasheet rev 1.15, section 6.3.2.2 "ADDRESS_RAMP_STAT - Ramp & Reference Switch Status Register".
 * @return true if the target velocity has been reached, false otherwise.
 */
bool TMC5160::isTargetVelocityReached(void)
{
    rampStatus.bytes = readRegister(ADDRESS_RAMP_STAT);
    return rampStatus.velocity_reached ? true : false;
}

void TMC5160::terminateRampEarly()
{
    // §14.2.4 Early Ramp Termination option b)
    writeRegister(ADDRESS_VSTART, 0);
    writeRegister(ADDRESS_VMAX, 0);
}

void TMC5160::disable()
{
    chopConf.toff = 0;
    writeRegister(ADDRESS_CHOPCONF, chopConf.bytes);
}

void TMC5160::enable()
{
    writeRegister(ADDRESS_CHOPCONF, chopConf.bytes);
}


bool TMC5160::isIcRest()
{
    globalStatus.bytes = readRegister(ADDRESS_GSTAT);
    return globalStatus.reset? true : false;
}


DriverStatus TMC5160::getDriverStatus()
{
    globalStatus.bytes = readRegister(ADDRESS_GSTAT);
    drvStatus.bytes = readRegister(ADDRESS_DRV_STATUS);

    // Serial.print(drvStatus.bytes, BIN);

    // if (drvStatus.stealth)
    // {
    //     Serial.println("stealthchop");
    // }
    // else
    // {
    //     Serial.println("speread cycle");
    // }
    // if (drvStatus.stst)
    // {
    //     Serial.println("standstill");
    // }
    // Serial.println("Sg_result:" + String(drvStatus.sg_result));

    if (globalStatus.uv_cp)
        return CP_UV;
    if (drvStatus.s2vsa)
        return S2VSA;
    if (drvStatus.s2vsb)
        return S2VSB;
    if (drvStatus.s2ga)
        return S2GA;
    if (drvStatus.s2gb)
        return S2GB;
    if (drvStatus.ot)
        return OT;
    if (globalStatus.drv_err)
        return OTHER_ERR;
    if (drvStatus.otpw)
        return OTPW;

    return OK;
}

//     Serial.println(F("Single device commands available: (line end with \\n)"));
//     Serial.println(F("h       -- Show this help"));
//     Serial.println(F("d       -- Disconnect / back to device list"));
//     Serial.println(F("s       -- Display current device settings"));
//     Serial.println(F("reg (-i)-- Display current raw register value (with comparison to default)"));
//     Serial.println(F("def     -- Apply the default settings"));
//     Serial.println(F("a <val> -- Change the Address"));
//     Serial.println(F("z <val> -- Set a zero-value"));
//     Serial.println(F("r       -- Flip the Rotation sign"));
//     Serial.println(F("t       -- read the current state for testing"));
//     Serial.println(F("(or blank)"));

const char *TMC5160::getDriverStatusDescription(DriverStatus st)
{
    switch (st)
    {
    case OK:
        return "OK";
    case CP_UV:
        return "Charge pump undervoltage";
    case S2VSA:
        return "Short to supply phase A";
    case S2VSB:
        return "Short to supply phase B";
    case S2GA:
        return "Short to ground phase A";
    case S2GB:
        return "Short to ground phase B";
    case OT:
        return "Overtemperature";
    case OTHER_ERR:
        return "Other driver error";
    case OTPW:
        return "Overtemperature warning";
    default:
        break;
    }

    return "Unknown";
}

void TMC5160::setModeChangeSpeeds(float pwmThrs, float coolThrs, float highThrs)
{
    writeRegister(ADDRESS_TPWMTHRS, thrsSpeedToTstep(pwmThrs));
    writeRegister(ADDRESS_TCOOLTHRS, thrsSpeedToTstep(coolThrs));
    writeRegister(ADDRESS_THIGH, thrsSpeedToTstep(highThrs));
}


bool TMC5160::setEncoderResolution(int motorSteps, int encResolution, bool inverted)
{
    // See §22.2
    float factor = (float)motorSteps * (float)_uStepCount / (float)encResolution;

    // Check if the binary prescaler gives an exact match
    if ((int)(factor * 65536.0f) * encResolution == motorSteps * _uStepCount * 65536)
    {
        encmode.bytes = readRegister(ADDRESS_ENCMODE);
        encmode.enc_sel_decimal = false;
        writeRegister(ADDRESS_ENCMODE, encmode.bytes);

        int32_t encConst = (int)(factor * 65536.0f);
        if (inverted)
            encConst = -encConst;
        writeRegister(ADDRESS_ENC_CONST, encConst);

#if 0
		Serial.println("Using binary mode");
		Serial.print("Factor : 0x");
		Serial.print(encConst, HEX);
		Serial.print(" <=> ");
		Serial.println((float)(encConst) / 65536.0f);
#endif

        return true;
    }
    else
    {
        encmode.bytes = readRegister(ADDRESS_ENCMODE);
        encmode.enc_sel_decimal = true;
        writeRegister(ADDRESS_ENCMODE, encmode.bytes);

        int integerPart = floor(factor);
        int decimalPart = (int)((factor - (float)integerPart) * 10000.0f);
        if (inverted)
        {
            integerPart = 65535 - integerPart;
            decimalPart = 10000 - decimalPart;
        }
        int32_t encConst = integerPart * 65536 + decimalPart;
        writeRegister(ADDRESS_ENC_CONST, encConst);

#if 0
		Serial.println("Using decimal mode");
		Serial.print("Factor : 0x");
		Serial.print(encConst, HEX);
		Serial.print(" <=> ");
		Serial.print(integerPart);
		Serial.print(".");
		Serial.println(decimalPart);
#endif

        // Check if the decimal prescaler gives an exact match. Floats have about 7 digits of precision so no worries
        // here.
        return ((int)(factor * 10000.0f) * encResolution == motorSteps * (int)_uStepCount * 10000);
    }
}

void TMC5160::setEncoderIndexConfiguration(ENCMODE_sensitivity_Values sensitivity, bool nActiveHigh,
                                           bool ignorePol, bool aActiveHigh, bool bActiveHigh)
{
    encmode.bytes = readRegister(ADDRESS_ENCMODE);

    encmode.sensitivity = sensitivity;
    encmode.pol_N = nActiveHigh;
    encmode.ignore_AB = ignorePol;
    encmode.pol_A = aActiveHigh;
    encmode.pol_B = bActiveHigh;

    writeRegister(ADDRESS_ENCMODE, encmode.bytes);
}

void TMC5160::setEncoderLatching(bool enabled)
{
    encmode.bytes = readRegister(ADDRESS_ENCMODE);

    encmode.latch_x_act = true;
    encmode.clr_cont = enabled;

    writeRegister(ADDRESS_ENCMODE, encmode.bytes);
}

void TMC5160::setCurrentMilliamps(uint16_t Irms) {
    const int32_t const_val = 11585;
    const int32_t Vfs = 325;
    const float Rsense = 0.075f;
    int32_t cs = 31;  // Initial CS value

    // Calculate GlobalScaler and CS
    uint32_t globalScaler = 0;
    bool found = false;

    for (; cs >= 0; cs--) {
        globalScaler = ((Irms * const_val * Rsense) / ((cs + 1) * Vfs))-1;
        if (globalScaler == 0 || (globalScaler >= 128 && globalScaler <= 255)) {
            found = true;
            break;
        }
    }

    if (found) {
        iholdrun.irun = cs;
        iholdrun.ihold = 16;
        iholdrun.iholddelay = 10;
        // Print the results
        Serial.println("GlobalScaler: " + String(globalScaler));
        Serial.println("cs: " + String(cs));
        writeRegister(ADDRESS_GLOBAL_SCALER, constrain(globalScaler, 32, 256));
        writeRegister(ADDRESS_IHOLD_IRUN, iholdrun.bytes);
    } else {
        // TODO(yasir): just for testing have to improve it with return values or some error handling
        Serial.println("invalid current parameters: "+String(Irms));
    }
}

void TMC5160::setEncoderAllowedDeviation(int steps)
{
    writeRegister(ADDRESS_ENC_DEVIATION, steps * _uStepCount);
}

bool TMC5160::isEncoderDeviationDetected()
{

    encstatus.bytes = readRegister(ADDRESS_ENC_STATUS);
    return encstatus.deviation_warn;
}

void TMC5160::clearEncoderDeviationFlag()
{

    encstatus.deviation_warn = true;
    writeRegister(ADDRESS_ENC_STATUS, encstatus.bytes);
}

void TMC5160::setShortProtectionLevels(int s2vsLevel, int s2gLevel, int shortFilter, int shortDelay)
{
    shortConf.s2vs_level = constrain(s2vsLevel, 4, 15);
    shortConf.s2g_level = constrain(s2gLevel, 2, 15);
    shortConf.shortfilter = constrain(shortFilter, 0, 3);
    shortConf.shortdelay = constrain(shortDelay, 0, 1);

    writeRegister(ADDRESS_SHORT_CONF, shortConf.bytes);
}




TMC5160_SPI::TMC5160_SPI( uint8_t chipSelectPin, uint32_t fclk, const SPISettings &spiSettings, SPIClass &spi )
: TMC5160(fclk), _CS(chipSelectPin), _spiSettings(spiSettings), _spi(&spi)
{
	pinMode(chipSelectPin, OUTPUT);
}


// calls to read/write registers must be bracketed by the begin/endTransaction calls

void _chipSelect( uint8_t pin, bool select )
{
	digitalWrite(pin, select?LOW:HIGH);
}

void TMC5160_SPI::_beginTransaction()
{
	_spi->beginTransaction(_spiSettings);
	_chipSelect(_CS, true);
}

void TMC5160_SPI::_endTransaction()
{
	_chipSelect(_CS, false);
	_spi->endTransaction();
}

uint32_t TMC5160_SPI::readRegister(uint8_t address)
{
    _beginTransaction();
    _spi->transfer(address);

    uint32_t value = 0;
    for (int8_t shift = 24; shift >= 0; shift -= 8) {
        value |= (uint32_t)_spi->transfer(0x00) << shift;
    }
    _endTransaction();

    return value;
}


uint8_t TMC5160_SPI::writeRegister(uint8_t address, uint32_t data)
{
    // address register
    _beginTransaction();
    uint8_t status = _spi->transfer(address | WRITE_ACCESS);

    // send new register value
    for (int8_t shift = 24; shift >= 0; shift -= 8) {
        _spi->transfer((data >> shift) & 0xFF);
    }
    _endTransaction();

    return status;
}



TMC5160_UART_Generic::TMC5160_UART_Generic(uint8_t slaveAddress, uint32_t fclk)
: TMC5160(fclk), _slaveAddress(slaveAddress), _currentMode(STREAMING_MODE)
{

}

bool TMC5160_UART_Generic::begin()
{
	CommunicationMode oldMode = _currentMode;
	setCommunicationMode(RELIABLE_MODE);

	// SLAVECONF_Register slaveConf = { 0 };
	// slaveConf.senddelay = 2; // minimum if more than one slave is present.
	// writeRegister(ADDRESS_SLAVECONF, slaveConf.bytes);

	bool result = TMC5160::begin();
	setCommunicationMode(oldMode);
	return result;
}

uint32_t TMC5160_UART_Generic::readRegister(uint8_t address, ReadStatus *status)
{
	uint32_t data = 0xFFFFFFFF;
	ReadStatus readStatus;

	switch (_currentMode)
	{
		case STREAMING_MODE:
			data = _readReg(address, &readStatus);
			break;

		case RELIABLE_MODE:
		{
			int retries = NB_RETRIES_READ;
			readStatus = NO_REPLY; //Worst case. If there is no reply for all retries this should be notified to the user.
			do {
				ReadStatus trialStatus;
				data = _readReg(address, &trialStatus);

				if (trialStatus == SUCCESS || (readStatus == NO_REPLY && trialStatus != NO_REPLY))
					readStatus = trialStatus;

				if (trialStatus == NO_REPLY)
					resetCommunication();

				retries--;
			} while (readStatus != SUCCESS && retries > 0);
			break;
		}
	}

	if (status != nullptr)
		*status = readStatus;

	return data;
}

uint8_t TMC5160_UART_Generic::writeRegister(uint8_t address, uint32_t data, ReadStatus *status)
{
	switch (_currentMode)
	{
		case STREAMING_MODE:
			_writeReg(address, data);

			if (status != nullptr)
				*status = SUCCESS;
			break;

		case RELIABLE_MODE:
		{
			int retries = NB_RETRIES_WRITE;
			ReadStatus writeStatus = NO_REPLY;
			do {
				_writeReg(address, data);
				_writeAttemptsCounter++;

				ReadStatus readStatus;
				uint8_t counter = readRegister(ADDRESS_IFCNT, &readStatus) & 0xFF;

				if (readStatus != NO_REPLY)
					writeStatus = readStatus;

				if (readStatus == SUCCESS)
				{
					if (counter != _transmissionCounter + 1)
						writeStatus = BAD_CRC;

					_transmissionCounter = counter;
				}

			retries--;
			} while (writeStatus != SUCCESS && retries > 0);

			if (status != nullptr)
				*status = writeStatus;

			if (writeStatus == SUCCESS)
				_writeSuccessfulCounter++;

			break;
		}
	}

	return 0;
}

void TMC5160_UART_Generic::resetCommunication()
{
	//FIXME should take into account the previous baud rate !
	// For now let's wait 1ms. The spec asks for ~75 bit times so this should be OK for baud rates > 75kbps
	// as of now (09/2018) delay() is broken for small durations on ESP32. Use delayMicroseconds instead
	delayMicroseconds(1000);

#ifdef SERIAL_DEBUG
	Serial.println("Resetting communication.");
#endif

	uartFlushInput();
}

void TMC5160_UART_Generic::setSlaveAddress(uint8_t slaveAddress, bool NAI)
{
	SLAVECONF_Register slaveConf = { 0 };
	slaveConf.senddelay = 2; // minimum if more than one slave is present.
	slaveConf.slaveaddr = constrain(NAI ? slaveAddress-1 : slaveAddress, 0, 253); //NB : if NAI is high SLAVE_ADDR is incremented.

	writeRegister(ADDRESS_SLAVECONF, slaveConf.bytes);

	_slaveAddress = NAI ? slaveConf.slaveaddr+1 : slaveConf.slaveaddr;
}

void TMC5160_UART_Generic::setCommunicationMode(TMC5160_UART_Generic::CommunicationMode mode)
{
	if (_currentMode == mode)
		return;

	_currentMode = mode;

	if (mode == RELIABLE_MODE)
	{
		//Initialize the 8-bit transmission counter.
		_transmissionCounter = readRegister(ADDRESS_IFCNT) & 0xFF;
	}
}

void TMC5160_UART_Generic::resetCommunicationSuccessRate()
{
	_readAttemptsCounter = _readSuccessfulCounter = _writeAttemptsCounter = _writeSuccessfulCounter = 0;
}

float TMC5160_UART_Generic::getReadSuccessRate()
{
	if (_readAttemptsCounter == 0)
		return 0;

	return (float)_readSuccessfulCounter / (float)_readAttemptsCounter;
}

float TMC5160_UART_Generic::getWriteSuccessRate()
{
	if (_writeAttemptsCounter == 0)
		return 0;

	return (float)_writeSuccessfulCounter / (float)_writeAttemptsCounter;
}

uint32_t TMC5160_UART_Generic::_readReg(uint8_t address, ReadStatus *status)
{
	uint8_t outBuffer[4], inBuffer[8];

	outBuffer[0] = SYNC_BYTE;
	outBuffer[1] = _slaveAddress;
	outBuffer[2] = address;

	computeCrc(outBuffer, 4);

	uartFlushInput();

	beginTransmission();
	uartWriteBytes(outBuffer, 4);
	endTransmission();

	_readAttemptsCounter++;

	unsigned long startTime = micros();
	uint8_t rxLen = 0;
	while (micros() - startTime < 1000 && rxLen < 8) //Timeout : 1ms TODO depends on the baudrate and the SENDDELAY parameter
	{
		if (uartBytesAvailable() > 0)
		{
			inBuffer[rxLen++] = uartReadByte();
			//Discard the first bytes if necessary
			if (rxLen == 1 && inBuffer[0] != SYNC_BYTE)
				rxLen = 0;
		}
	}

	if (rxLen < 8)
	{
		if (status != nullptr)
			*status = NO_REPLY;

	#ifdef SERIAL_PRINT_ERRORS
		Serial.print("Read 0x");
		Serial.print(address, HEX);
		Serial.print(": No reply (");
		Serial.print(rxLen);
		Serial.println(" bytes read)");
		Serial.print("{");
		for (int i = 0; i < 8; i++)
		{
			Serial.print("0x");
			Serial.print(inBuffer[i], HEX);
			Serial.print(" ");
		}
		Serial.println("}");
	#endif

		return 0xFFFFFFFF;
	}

	if (inBuffer[0] != SYNC_BYTE || inBuffer[1] != MASTER_ADDRESS || inBuffer[2] != address)
	{
		if (status != nullptr)
			*status = INVALID_FORMAT;

	#ifdef SERIAL_PRINT_ERRORS
		Serial.print("Read 0x");
		Serial.print(address, HEX);
		Serial.println(": Invalid answer format.");
		Serial.print("{");
		for (int i = 0; i < 8; i++)
		{
			Serial.print("0x");
			Serial.print(inBuffer[i], HEX);
			Serial.print(" ");
		}
		Serial.println("}");
	#endif

		uartFlushInput();

		return 0xFFFFFFFF;
	}

	uint8_t receivedCrc = inBuffer[7];
	computeCrc(inBuffer, 8);

	if (receivedCrc != inBuffer[7])
	{
		if (status != nullptr)
			*status = BAD_CRC;

	#ifdef SERIAL_PRINT_ERRORS
		Serial.print("Read 0x");
		Serial.print(address, HEX);
		Serial.println(": Bad CRC.");
	#endif

		return 0xFFFFFFFF;
	}

	uint32_t data = 0;
	for (int i = 0; i < 4; i++)
		data += ((uint32_t)inBuffer[3+i] << ((3-i)*8));

	#ifdef SERIAL_DEBUG
	Serial.print("Read 0x");
	Serial.print(address, HEX);
	Serial.print(": 0x");
	Serial.println(data, HEX);
	#endif

	_readSuccessfulCounter++;

	if (status != nullptr)
		*status = SUCCESS;

	return data;
}

void TMC5160_UART_Generic::_writeReg(uint8_t address, uint32_t data)
{
#ifdef SERIAL_DEBUG
	Serial.print("Writing 0x");
	Serial.print(address, HEX);
	Serial.print(": 0x");
	Serial.println(data, HEX);
#endif

	uint8_t buffer[8];
	buffer[0] = SYNC_BYTE;
	buffer[1] = _slaveAddress;
	buffer[2] = address | WRITE_ACCESS;
	for (int i = 0; i < 4; i++)
		buffer[3+i] = (data & (0xFFul << ((3-i)*8))) >> ((3-i)*8);

	computeCrc(buffer, 8);

#if 0
	//Intentional interference to test the reliable mode : change the CRC
	if (random(256) < 64)
		buffer[7]++;
#endif

	beginTransmission();
	uartWriteBytes(buffer, 8);
	endTransmission();
}

/* From Trinamic TMC5130A datasheet Rev. 1.14 / 2017-MAY-15 §5.2 */
void TMC5160_UART_Generic::computeCrc(uint8_t *datagram, uint8_t datagramLength)
{
	int i,j;
	uint8_t* crc = datagram + (datagramLength-1); // CRC located in last byte of message
	uint8_t currentByte;

	*crc = 0;
	for (i = 0; i < (datagramLength-1); i++)
	{
		currentByte = datagram[i];
		for (j = 0; j < 8; j++)
		{
			if ((*crc >> 7) ^ (currentByte & 0x01))
				*crc = (*crc << 1) ^ 0x07;
			else
				*crc = (*crc << 1);

			currentByte = currentByte >> 1;
		} // for CRC bit
	} // for message byte
}
