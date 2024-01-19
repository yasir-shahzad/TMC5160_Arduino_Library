

#include "TMC5160.h"

TMC5160::TMC5160(uint32_t fclk) : _fclk(fclk)
{
}

TMC5160::~TMC5160()
{
    ;
}

bool TMC5160::begin(const PowerStageParameters &powerParams, const MotorParameters &motorParams,
                    MotorDirection stepperDirection)
{
    bool retVal = false;

    /* Clear the reset and charge pump undervoltage flags */
    TMC5160_Reg::GSTAT_Register gstat = {0};
    gstat.reset = true;
    gstat.uv_cp = true;
    writeRegister(TMC5160_Reg::GSTAT, gstat.bytes);

    TMC5160_Reg::DRV_CONF_Register drvConf = {0};
    drvConf.drvstrength = constrain(powerParams.drvStrength, 0, 3);
    drvConf.bbmtime = constrain(powerParams.bbmTime, 0, 24);
    drvConf.bbmclks = constrain(powerParams.bbmClks, 0, 15);
    writeRegister(TMC5160_Reg::DRV_CONF, drvConf.bytes);

    writeRegister(TMC5160_Reg::GLOBAL_SCALER, constrain(motorParams.globalScaler, 32, 256));

    // set initial currents and delay
    TMC5160_Reg::IHOLD_IRUN_Register iholdrun = {0};
    iholdrun.ihold = constrain(motorParams.ihold, 0, 31);
    iholdrun.irun = constrain(motorParams.irun, 0, 31);
    iholdrun.iholddelay = 10;
    writeRegister(TMC5160_Reg::IHOLD_IRUN, iholdrun.bytes);

    // TODO(yasir): set short detection / overcurrent protection levels
    setModeChangeSpeeds(170, 0, 0);

    // Set initial PWM values
    TMC5160_Reg::PWMCONF_Register pwmconf = {0};
    pwmconf.bytes = 0xC40C001E;   // Reset default
    pwmconf.pwm_autoscale = true; // Temp to set OFS and GRAD initial values
    if (_fclk > DEFAULT_F_CLK)
        pwmconf.pwm_freq = 0;
    else
        pwmconf.pwm_freq = 0b01; // recommended : 35kHz with internal typ. 12MHZ clock. 0b01 => 2/683 * f_clk
    pwmconf.pwm_grad = motorParams.pwmGradInitial;
    pwmconf.pwm_ofs = motorParams.pwmOfsInitial;
    pwmconf.freewheel = motorParams.freewheeling;

    pwmconf.pwm_autoscale = true;
    pwmconf.pwm_autograd = true;
    writeRegister(TMC5160_Reg::PWMCONF, pwmconf.bytes);

    // Recommended settings in quick config guide
    _chopConf.toff = 2;
    _chopConf.tbl = 0;
    _chopConf.hstrt_tfd = 7;
    _chopConf.hend_offset = 7;
    _chopConf.mres = 0;
    _chopConf.chm = 0;
    _chopConf.tpfd = 0;
    writeRegister(TMC5160_Reg::CHOPCONF, _chopConf.bytes);
    setRampMode(VELOCITY_MODE);

    TMC5160_Reg::GCONF_Register gconf = {0};
    gconf.en_pwm_mode = true; // Enable stealthChop PWM mode
    gconf.multistep_filt = true;
    //  gconf.shaft = 0;  // 1 to invert the motor direction
    writeRegister(TMC5160_Reg::GCONF, gconf.bytes);

    if (gconf.bytes == readRegister(TMC5160_Reg::GCONF))
    {
        retVal = true;
    }

    // Set default start, stop, threshold speeds.
    setRampSpeeds(50, 0, 0); // Start, stop, threshold speeds */

    return (retVal);
}


void TMC5160::setRampMode(TMC5160::RampMode mode)
{
    switch (mode)
    {
    case POSITIONING_MODE:
        writeRegister(TMC5160_Reg::RAMPMODE, TMC5160_Reg::POSITIONING_MODE);
        break;

    case VELOCITY_MODE:
        setMaxSpeed(
            0); // There is no way to know if we should move in the positive or negative direction => set speed to 0.
        writeRegister(TMC5160_Reg::RAMPMODE, TMC5160_Reg::VELOCITY_MODE_POS);
        break;

    case HOLD_MODE:
        writeRegister(TMC5160_Reg::RAMPMODE, TMC5160_Reg::HOLD_MODE);
        break;
    }

    _currentRampMode = mode;
}

float TMC5160::getCurrentPosition()
{
    int32_t uStepPos = readRegister(TMC5160_Reg::XACTUAL);

    if ((uint32_t)(uStepPos) == 0xFFFFFFFF)
        return NAN;
    else
        return (float)uStepPos / (float)_uStepCount;
}

float TMC5160::getEncoderPosition()
{
    int32_t uStepPos = readRegister(TMC5160_Reg::X_ENC);

    if ((uint32_t)(uStepPos) == 0xFFFFFFFF)
        return NAN;
    else
        return (float)uStepPos / (float)_uStepCount;
}

void TMC5160::invertDriver(bool invert)
{
    invertMotor = invert;

    uint32_t _GCONF = tmc5160_readInt(TMC5160_GCONF); // dummy or old data
    _GCONF = tmc5160_readInt(TMC5160_GCONF);

    if (invert)
    {
        _GCONF |= (1 << 4);
    }
    else
    {
        _GCONF &= ~(1 << 4);
    }

    tmc5160_writeInt(TMC5160_GCONF, _GCONF);
}

float TMC5160::getLatchedPosition()
{
    int32_t uStepPos = readRegister(TMC5160_Reg::XLATCH);

    if ((uint32_t)(uStepPos) == 0xFFFFFFFF)
        return NAN;
    else
        return (float)uStepPos / (float)_uStepCount;
}

float TMC5160::getLatchedEncoderPosition()
{
    int32_t uStepPos = readRegister(TMC5160_Reg::ENC_LATCH);

    if ((uint32_t)(uStepPos) == 0xFFFFFFFF)
        return NAN;
    else
        return (float)uStepPos / (float)_uStepCount;
}

float TMC5160::getTargetPosition()
{
    int32_t uStepPos = readRegister(TMC5160_Reg::XTARGET);

    if ((uint32_t)(uStepPos) == 0xFFFFFFFF)
        return NAN;
    else
        return (float)uStepPos / (float)_uStepCount;
}

float TMC5160::getCurrentSpeed()
{
    uint32_t data = readRegister(TMC5160_Reg::VACTUAL);

    if (data == 0xFFFFFFFF)
        return NAN;

    // Returned data is 24-bits, signed => convert to 32-bits, signed.
    if (bitRead(data, 23)) // highest bit set => negative value
        data |= 0xFF000000;

    return speedToHz(data);
}

void TMC5160::setCurrentPosition(float position, bool updateEncoderPos)
{
    writeRegister(TMC5160_Reg::XACTUAL, (int)(position * (float)_uStepCount));

    if (updateEncoderPos)
    {
        writeRegister(TMC5160_Reg::X_ENC, (int)(position * (float)_uStepCount));
        clearEncoderDeviationFlag();
    }
}

void TMC5160::setTargetPosition(float position)
{
    writeRegister(TMC5160_Reg::XTARGET, (int32_t)(position * (float)_uStepCount));
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
            writeRegister(TMC5160_Reg::GLOBAL_SCALER, constrain(136, 32, 256));
        }
    }
    if (speed == 0)
    {
        writeRegister(TMC5160_Reg::GLOBAL_SCALER, constrain(174, 32, 256));
        count = 0;
    }
    if (count == 0)
    {
        Starttime = millis();
    }
    speed = 1.666666667 * speed;
    writeRegister(TMC5160_Reg::VMAX, speedFromHz(fabs(speed)));

    if (_currentRampMode == VELOCITY_MODE)
    {
        writeRegister(TMC5160_Reg::RAMPMODE,
                      speed < 0.0f ? TMC5160_Reg::VELOCITY_MODE_NEG : TMC5160_Reg::VELOCITY_MODE_POS);
    }
}

void TMC5160::setRampSpeeds(float startSpeed, float stopSpeed, float transitionSpeed)
{
    writeRegister(TMC5160_Reg::VSTART, speedFromHz(fabs(startSpeed)));
    writeRegister(TMC5160_Reg::VSTOP, speedFromHz(fabs(stopSpeed)));
    writeRegister(TMC5160_Reg::V_1, speedFromHz(fabs(transitionSpeed)));
}

void TMC5160::setAcceleration(float maxAccel)
{
    writeRegister(TMC5160_Reg::AMAX, accelFromHz(fabs(maxAccel)));
}

void TMC5160::setAccelerations(float maxAccel, float startAccel, float maxDecel, float finalDecel)
{
    writeRegister(TMC5160_Reg::DMAX, accelFromHz(fabs(maxDecel)));
    writeRegister(TMC5160_Reg::AMAX, accelFromHz(fabs(maxAccel)));
    writeRegister(TMC5160_Reg::A_1, accelFromHz(fabs(startAccel)));
    writeRegister(TMC5160_Reg::D_1, accelFromHz(fabs(finalDecel)));
}

/**
 *
 * @see Datasheet rev 1.15, section 6.3.2.2 "RAMP_STAT - Ramp & Reference Switch Status Register".
 * @return true if the target position has been reached, false otherwise.
 */
bool TMC5160::isTargetPositionReached(void)
{
    TMC5160_Reg::RAMP_STAT_Register rampStatus = {0};
    rampStatus.bytes = readRegister(TMC5160_Reg::RAMP_STAT);
    return rampStatus.position_reached ? true : false;
}

/**
 *
 * @see Datasheet rev 1.15, section 6.3.2.2 "RAMP_STAT - Ramp & Reference Switch Status Register".
 * @return true if the target velocity has been reached, false otherwise.
 */
bool TMC5160::isTargetVelocityReached(void)
{
    TMC5160_Reg::RAMP_STAT_Register rampStatus = {0};
    rampStatus.bytes = readRegister(TMC5160_Reg::RAMP_STAT);
    return rampStatus.velocity_reached ? true : false;
}

void TMC5160::terminateRampEarly()
{
    // §14.2.4 Early Ramp Termination option b)
    writeRegister(TMC5160_Reg::VSTART, 0);
    writeRegister(TMC5160_Reg::VMAX, 0);
}

void TMC5160::disable()
{
    TMC5160_Reg::CHOPCONF_Register chopconf = {0};
    chopconf.bytes = _chopConf.bytes;
    chopconf.toff = 0;
    writeRegister(TMC5160_Reg::CHOPCONF, chopconf.bytes);
}

void TMC5160::enable()
{
    writeRegister(TMC5160_Reg::CHOPCONF, _chopConf.bytes);
}


bool TMC5160::isIcRest()
{
    TMC5160_Reg::GSTAT_Register gstat = {0};
    gstat.bytes = readRegister(TMC5160_Reg::GSTAT);
    if (gstat.reset)
    {
        return true;
    }
    else
    {
        return false;
    }
}
TMC5160::DriverStatus TMC5160::getDriverStatus()
{
    TMC5160_Reg::GSTAT_Register gstat = {0};
    gstat.bytes = readRegister(TMC5160_Reg::GSTAT);
    TMC5160_Reg::DRV_STATUS_Register drvStatus = {0};
    drvStatus.bytes = readRegister(TMC5160_Reg::DRV_STATUS);

    Serial.print(drvStatus.bytes, BIN);

    if (drvStatus.stealth)
    {
        Serial.println("stealthchop");
    }
    else
    {
        Serial.println("speread cycle");
    }
    if (drvStatus.stst)
    {
        Serial.println("standstill");
    }
    Serial.println("Sg_result:" + String(drvStatus.sg_result));

    if (gstat.uv_cp)
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
    if (gstat.drv_err)
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
    writeRegister(TMC5160_Reg::TPWMTHRS, thrsSpeedToTstep(pwmThrs));
    writeRegister(TMC5160_Reg::TCOOLTHRS, thrsSpeedToTstep(coolThrs));
    writeRegister(TMC5160_Reg::THIGH, thrsSpeedToTstep(highThrs));
}

bool TMC5160::setEncoderResolution(int motorSteps, int encResolution, bool inverted)
{
    // See §22.2
    float factor = (float)motorSteps * (float)_uStepCount / (float)encResolution;

    // Check if the binary prescaler gives an exact match
    if ((int)(factor * 65536.0f) * encResolution == motorSteps * _uStepCount * 65536)
    {
        TMC5160_Reg::ENCMODE_Register encmode = {0};
        encmode.bytes = readRegister(TMC5160_Reg::ENCMODE);
        encmode.enc_sel_decimal = false;
        writeRegister(TMC5160_Reg::ENCMODE, encmode.bytes);

        int32_t encConst = (int)(factor * 65536.0f);
        if (inverted)
            encConst = -encConst;
        writeRegister(TMC5160_Reg::ENC_CONST, encConst);

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
        TMC5160_Reg::ENCMODE_Register encmode = {0};
        encmode.bytes = readRegister(TMC5160_Reg::ENCMODE);
        encmode.enc_sel_decimal = true;
        writeRegister(TMC5160_Reg::ENCMODE, encmode.bytes);

        int integerPart = floor(factor);
        int decimalPart = (int)((factor - (float)integerPart) * 10000.0f);
        if (inverted)
        {
            integerPart = 65535 - integerPart;
            decimalPart = 10000 - decimalPart;
        }
        int32_t encConst = integerPart * 65536 + decimalPart;
        writeRegister(TMC5160_Reg::ENC_CONST, encConst);

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

void TMC5160::setEncoderIndexConfiguration(TMC5160_Reg::ENCMODE_sensitivity_Values sensitivity, bool nActiveHigh,
                                           bool ignorePol, bool aActiveHigh, bool bActiveHigh)
{
    TMC5160_Reg::ENCMODE_Register encmode = {0};
    encmode.bytes = readRegister(TMC5160_Reg::ENCMODE);

    encmode.sensitivity = sensitivity;
    encmode.pol_N = nActiveHigh;
    encmode.ignore_AB = ignorePol;
    encmode.pol_A = aActiveHigh;
    encmode.pol_B = bActiveHigh;

    writeRegister(TMC5160_Reg::ENCMODE, encmode.bytes);
}

void TMC5160::setEncoderLatching(bool enabled)
{
    TMC5160_Reg::ENCMODE_Register encmode = {0};
    encmode.bytes = readRegister(TMC5160_Reg::ENCMODE);

    encmode.latch_x_act = true;
    encmode.clr_cont = enabled;

    writeRegister(TMC5160_Reg::ENCMODE, encmode.bytes);
}

void TMC5160::setCurrentMilliamps(uint16_t Irms) {
	const int32_t const_val = 11585;
    const int32_t Vfs = 325;
    const float Rsense = 0.075f;
    int32_t cs = 31; // Initial CS value

    TMC5160_Reg::IHOLD_IRUN_Register iholdrun_ = {0};

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
        iholdrun_.irun = cs;
        iholdrun_.ihold = 16;
        iholdrun_.iholddelay = 10;
        // Print the results
        Serial.println("GlobalScaler: " + String(globalScaler));
        Serial.println("cs: " + String(cs));
    } else {
        // TODO(yasir): just for testing have to improve it with return values or some error handling
        Serial.println("invalid current parameters: "+String(Irms));
    }
}

void TMC5160::setEncoderAllowedDeviation(int steps)
{
    writeRegister(TMC5160_Reg::ENC_DEVIATION, steps * _uStepCount);
}

bool TMC5160::isEncoderDeviationDetected()
{
    TMC5160_Reg::ENC_STATUS_Register encStatus = {0};
    encStatus.bytes = readRegister(TMC5160_Reg::ENC_STATUS);
    return encStatus.deviation_warn;
}

void TMC5160::clearEncoderDeviationFlag()
{
    TMC5160_Reg::ENC_STATUS_Register encStatus = {0};
    encStatus.deviation_warn = true;
    writeRegister(TMC5160_Reg::ENC_STATUS, encStatus.bytes);
}

void TMC5160::setShortProtectionLevels(int s2vsLevel, int s2gLevel, int shortFilter, int shortDelay)
{
    TMC5160_Reg::SHORT_CONF_Register shortConf = {0};
    shortConf.s2vs_level = constrain(s2vsLevel, 4, 15);
    shortConf.s2g_level = constrain(s2gLevel, 2, 15);
    shortConf.shortfilter = constrain(shortFilter, 0, 3);
    shortConf.shortdelay = constrain(shortDelay, 0, 1);

    writeRegister(TMC5160_Reg::SHORT_CONF, shortConf.bytes);
}
