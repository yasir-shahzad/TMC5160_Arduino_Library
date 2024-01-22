
#ifndef TMC5160_H
#define TMC5160_H

#include "TMC5160_Register.h"
#include <Arduino.h>
#include <SPI.h>


enum RampMode
{
    POSITIONING_MODE,
    VELOCITY_MODE,
    HOLD_MODE
};

enum DriverStatus
{
    OK,        // No error condition
    CP_UV,     // Charge pump undervoltage
    S2VSA,     // Short to supply phase A
    S2VSB,     // Short to supply phase B
    S2GA,      // Short to ground phase A
    S2GB,      // Short to ground phase B
    OT,        // Overtemperature (error)
    OTHER_ERR, // ADDRESS_GSTAT drv_err is set but none of the above conditions is found.
    OTPW       // Overtemperature pre warning
};

class TMC5160
{
  public:
    TMC5160(uint32_t fclk = DEFAULT_F_CLK);
    ~TMC5160();
    static constexpr uint8_t IC_VERSION = 0x30;
    static constexpr uint32_t DEFAULT_F_CLK = 12000000; // Typical internal clock frequency in Hz.


    // struct PowerStageParameters
    // {
    //     uint8_t drvStrength = 2; // MOSFET gate driver current (0 to 3)
    //     uint8_t bbmTime = 0;     // "Break Before Make" duration specified in ns (0 to 24)
    //     uint8_t bbmClks = 4;     // "Break Before Make" duration specified in clock cycles (0 to 15).
    // };

    // struct MotorParameters
    // {
        // PWMCONF_freewheel_Values freewheeling = FREEWHEEL_NORMAL;
    //     uint8_t pwmOfsInitial = 30;        // initial stealthChop PWM amplitude offset (0-255)
    //     uint8_t pwmGradInitial = 0;        // initial stealthChop velocity dependent gradient for PWM amplitude
    // };



    virtual bool begin();

    virtual uint32_t readRegister(uint8_t address) = 0;  // addresses are from TMC5160.h
    virtual uint8_t writeRegister(uint8_t address, uint32_t data) = 0;

    void setRampMode(RampMode mode);  //Doxygen
    float getCurrentPosition();  // Return the current internal position (steps)
    float getEncoderPosition();  // Return the current position according to the encoder counter (steps)
    float getLatchedPosition();  // Return the position that was latched on the last ref switch / encoder event (steps)
    float getLatchedEncoderPosition();  // Return the encoder position that was latched on the last encoder event (steps)
    float getTargetPosition();         // Get the target position (steps)
    float getCurrentSpeed();           // Return the current speed (steps / second)

    void setPowerStageParameters();

    void setCurrentPosition(float position, bool updateEncoderPos = false);
    // update the encoder counter as well to keep them in sync.
    void setTargetPosition(float position);
    void setMaxSpeed(float speed);  // Set the max speed ADDRESS_VMAX (steps/second)
    // Set the ramp start speed ADDRESS_VSTART, ramp stop speed ADDRESS_VSTOP, acceleration transition speed
    void setRampSpeeds(float startSpeed, float stopSpeed, float transitionSpeed);
    void setAcceleration(float maxAccel);  // Set the ramp acceleration / deceleration (steps / second^2)
    void setAccelerations(float maxAccel, float startAccel, float maxDecel, float finalDecel);

    bool isTargetPositionReached(void);  // Return true if the target position has been reached
    bool isTargetVelocityReached(void);  // Return true if the target velocity has been reached

    void terminateRampEarly();  // Stop the current motion according to the set ramp mode and motion parameters. The max

    void setHardwareEnablePin(uint8_t hardware_enable_pin);
    void enable();
    void disable();

    bool isIcRest();
    DriverStatus getDriverStatus();  // Get the current driver status (OK / error conditions)
    static const char *getDriverStatusDescription(DriverStatus st);  // Get a human readable description

    /* Set the speeds (in steps/second) at which the internal functions and modes will be turned on or off.
     * Below pwmThrs, "stealthChop" PWM mode is used.
     * Between pwmThrs and highThrs, "spreadCycle" classic mode is used.
     * Between coolThrs and highThrs, "spreadCycle" is used ; "coolStep" current reduction and "stallGuard" load
     * measurement can be enabled. Above highThrs, "constant Toff" mode and fullstep mode can be enabled. See the TMC
     * 5160 datasheet for details and optimization. Setting a speed to 0 will disable this threshold.
     */
    void setModeChangeSpeeds(float pwmThrs, float coolThrs, float highThrs);

    /* Set the encoder constant to match the motor and encoder resolutions.
     * This function will determine if the binary or decimal mode should be used
     * and return false if no exact match could be found (for example for an encoder
     * with a resolution of 360 and a motor with 200 steps per turn). In this case
     * the best approximation in decimal mode will be used.
     *
     * Params :
     * 		motorSteps : the number of steps per turn for the motor
     * 		encResolution : the actual encoder resolution (pulses per turn)
     * 		inverted : whether the encoder and motor rotations are inverted
     *
     * Return :
     * 		true if an exact match was found, false otherwise
     */
    bool setEncoderResolution(int motorSteps, int encResolution, bool inverted = false);

    /* Configure the encoder N event context.
     * Params :
     * 		sensitivity : set to one of ENCODER_N_NO_EDGE, ENCODER_N_RISING_EDGE, ENCODER_N_FALLING_EDGE,
     * ENCODER_N_BOTH_EDGES nActiveHigh : choose N signal polarity (true for active high) ignorePol : if true, ignore A
     * and B polarities to validate a N event aActiveHigh : choose A signal polarity (true for active high) to validate
     * a N event bActiveHigh : choose B signal polarity (true for active high) to validate a N event
     */
    void setEncoderIndexConfiguration(ENCMODE_sensitivity_Values sensitivity, bool nActiveHigh = true,
                                      bool ignorePol = true, bool aActiveHigh = false, bool bActiveHigh = false);

    /* Enable/disable encoder and position latching on each encoder N event (on each revolution)
     * The difference between the 2 positions can then be compared regularly to check
     * for an external step loss.
     */
    void setEncoderLatching(bool enabled);

    /* Set maximum number of steps between internal position and encoder position
     * before triggering the deviation flag.
     * Set to 0 to disable. */
    void setEncoderAllowedDeviation(int steps);

    /* Check if a deviation between internal pos and encoder has been detected */
    bool isEncoderDeviationDetected();

    /* Clear encoder deviation flag (deviation condition must be handled before) */
    void clearEncoderDeviationFlag();

    // TODO end stops and stallguard config functions ?

    /* Configure the integrated short protection. Check datasheet for details.
     * - s2vsLevel : 4 (highest sensitivity) to 15 ; 6 to 8 recommended ; reset default 6
     * - s2gLevel : 2 (highest sensitivity) to 15 ; 6 to 14 recommended ; reset default 6 ; increase at higher voltage
     * - shortFilter : 0 to 3 ; reset default 1 ; increase in case of erroneous detection
     * - shortDelay : 0 to 1 ; reset default 0
     */
    void setShortProtectionLevels(int s2vsLevel, int s2gLevel, int shortFilter, int shortDelay = 0);

    void setStealthChop();
    void setVelocityMode();
    void setDCStep();
    void setSpreadCycle();
    void setEncoder(int counts);
    void invertDriver(bool invert);
    void setCurrentMilliamps(uint16_t Irms);
    void setMicrosteps(uint8_t microsteps);

  protected:
    static constexpr uint8_t WRITE_ACCESS = 0x80; // Register write access for spi / uart communication

  private:
    uint32_t _fclk;
    RampMode _currentRampMode;
    static constexpr uint16_t _uStepCount = 256; // Number of microsteps per step

    GCONF_Register globalConfig;
    RAMP_STAT_Register rampStatus;
    CHOPCONF_Register chopConf;
    GSTAT_Register globalStatus;
    DRV_STATUS_Register drvStatus;
    ENCMODE_Register encmode;
    IHOLD_IRUN_Register iholdrun; // Create a variable of the IHOLD_IRUN_Register type
    DRV_CONF_Register drvconf;
    PWMCONF_Register pwmconf;
    ENC_STATUS_Register encstatus;
    SHORT_CONF_Register shortConf;
    COOLCONF_Register coolConf;
    SW_MODE_Register switchMode;

    // Referring to Topic 12.1 on Page 81 of Datasheet Version 1.17 for Real-world Unit Conversions
    //  v[Hz] = v[5160A] * ( f CLK [Hz]/2 / 2^23 )
    float speedToHz(long speedInternal)
    { // fclk = 12MHz, _uStepCount = 256,
        return ((float)speedInternal * (float)_fclk / (float)(1ul << 24) / (float)_uStepCount);
    }
    long speedFromHz(float speedHz)
    {
        return (long)(speedHz / ((float)_fclk / (float)(1ul << 24)) * (float)_uStepCount);
    }

    // Following §14.1 Real world unit conversions
    // a[Hz/s] = a[5160A] * f CLK [Hz]^2 / (512*256) / 2^24
    long accelFromHz(float accelHz)
    {
        return (long)(accelHz / ((float)_fclk * (float)_fclk / (512.0 * 256.0) / (float)(1ul << 24)) *
                      (float)_uStepCount);
    }

    // See §12 Velocity based mode control
    long thrsSpeedToTstep(float thrsSpeed)
    {
        return thrsSpeed != 0.0 ? (long)constrain((float)_fclk / (thrsSpeed * 256.0), 0, 1048575) : 0;
    }
};




/* SPI interface : 
 * the TMC5160 SWSEL input has to be low (default state).
 */
class TMC5160_SPI : public TMC5160
{
  public:
    TMC5160_SPI(uint8_t chipSelectPin, uint32_t fclk = DEFAULT_F_CLK,
                const SPISettings &spiSettings = SPISettings(1000000, MSBFIRST, SPI_MODE3), SPIClass &spi = SPI);

    uint32_t readRegister(uint8_t address);
    uint8_t writeRegister(uint8_t address, uint32_t data);

  private:
    uint8_t _CS;
    SPISettings _spiSettings;
    SPIClass *_spi;

    void _beginTransaction();
    void _endTransaction();
};








/* Generic UART interface */
class TMC5160_UART_Generic : public TMC5160
{
  public:
    /* Read/write register return codes */
    enum ReadStatus
    {
        SUCCESS,
        NO_REPLY,
        INVALID_FORMAT,
        BAD_CRC
    };

    /* Serial communication modes. In reliable mode, register writes are checked and
     * retried if necessary, and register reads are retried multiple times in case
     * of failure. In streaming mode, none of these checks are performed and register
     * read / writes are tried only once. Default is Streaming mode. */
    enum CommunicationMode
    {
        RELIABLE_MODE,
        STREAMING_MODE
    };

    TMC5160_UART_Generic(uint8_t slaveAddress = 0, // TMC5160 slave address (default 0 if NAI is low, 1 if NAI is high)
                         uint32_t fclk = DEFAULT_F_CLK);

    virtual bool begin();

    uint32_t readRegister(uint8_t address, ReadStatus *status);
    uint32_t readRegister(uint8_t address)
    {
        return readRegister(address, nullptr);
    }
    uint8_t writeRegister(uint8_t address, uint32_t data,
                          ReadStatus *status); // Pass an optional status pointer to detect failures.
    uint8_t writeRegister(uint8_t address, uint32_t data)
    {
        return writeRegister(address, data, nullptr);
    }

    void resetCommunication(); // Reset communication with TMC5160 : pause activity on the serial bus.

    void setSlaveAddress(uint8_t slaveAddress,
                         bool NAI = true); // Set the slave address register. Take into account the TMC5160 NAI input
                                           // (default to high). Range : 0 - 253 if NAI is low, 1 - 254 if NAI is high.
    void setInternalSlaveAddress(uint8_t slaveAddress)
    {
        _slaveAddress = slaveAddress;
    }
    uint8_t getSlaveAddress()
    {
        return _slaveAddress;
    }

    void setCommunicationMode(CommunicationMode mode);

    /* Register read / write statistics */
    void resetCommunicationSuccessRate();
    float getReadSuccessRate();
    float getWriteSuccessRate();

  protected:
    static constexpr uint8_t NB_RETRIES_READ = 3;
    static constexpr uint8_t NB_RETRIES_WRITE = 3;

    uint8_t _slaveAddress;
    CommunicationMode _currentMode;
    uint8_t _transmissionCounter;

    uint32_t _readAttemptsCounter;
    uint32_t _readSuccessfulCounter;
    uint32_t _writeAttemptsCounter;
    uint32_t _writeSuccessfulCounter;

    virtual void beginTransmission()
    {
        delayMicroseconds(180); // FIXME a communication reset time is necessary between 2 read/write accesses. Depends
                                // on the baudrate !
    }

    virtual void endTransmission()
    {
    }

    virtual void uartFlushInput() = 0;
    virtual void uartWriteBytes(const uint8_t *buf, uint8_t len) = 0;
    virtual int uartReadBytes(uint8_t *buf, uint8_t len) = 0;
    virtual uint8_t uartReadByte() = 0;
    virtual int uartBytesAvailable() = 0;

    uint32_t _readReg(uint8_t address, ReadStatus *status);
    void _writeReg(uint8_t address, uint32_t data);

  private:
    static constexpr uint8_t SYNC_BYTE = 0x05;
    static constexpr uint8_t MASTER_ADDRESS = 0xFF;

    void computeCrc(uint8_t *datagram, uint8_t datagramLength);
};

/* Arduino UART interface :
 * the TMC5160 SWSEL input must be tied high.
 *
 * This class does not handle TX/RX switch on the half-duplex bus.
 * It should be used only if there is another mechanism to switch between
 * transmission and reception (e.g. on Teensy the Serial class can be configured
 * to control an external transceiver).
 *
 * Serial must be initialized externally. Serial.setTimeout() must be set to a
 * decent value to avoid blocking for too long if there is a RX error.
 */
class TMC5160_UART : public TMC5160_UART_Generic
{
  public:
    TMC5160_UART(Stream &serial = Serial,  // Serial port to use
                 uint8_t slaveAddress = 0, // TMC5160 slave address (default 0 if NAI is low, 1 if NAI is high)
                 uint32_t fclk = DEFAULT_F_CLK)
        : TMC5160_UART_Generic(slaveAddress, fclk), _serial(&serial)
    {
    }

  protected:
    Stream *_serial;

    virtual void uartFlushInput()
    {
        while (_serial->available())
            _serial->read();
    }

    virtual void uartWriteBytes(const uint8_t *buf, uint8_t len)
    {
        _serial->write(buf, len);
    }

    virtual int uartReadBytes(uint8_t *buf, uint8_t len)
    {
        return _serial->readBytes(buf, len);
    }

    virtual uint8_t uartReadByte()
    {
        return (uint8_t)(_serial->read());
    }

    virtual int uartBytesAvailable()
    {
        return _serial->available();
    }
};

/* Arduino UART interface with external transceiver support :
 * the TMC5160 SWSEL input must be tied high.
 * See TMC5160 datasheet §5.4 figure 5.2 for wiring details
 *
 * This interface switches a digital pin to control an external transceiver to
 * free the bus when not transmitting.
 *
 * This is not optimized : the interface has to wait for the end of the
 * transmission.
 *
 * Serial must be initialized externally. Serial.setTimeout() must be set to a
 * decent value to avoid blocking for too long if there is a RX error.
 */
class TMC5160_UART_Transceiver : public TMC5160_UART
{
  public:
    TMC5160_UART_Transceiver(
        uint8_t txEnablePin = -1, // pin to enable transmission on the external transceiver
        Stream &serial = Serial,  // Serial port to use
        uint8_t slaveAddress = 0, // TMC5160 slave address (default 0 if NAI is low, 1 if NAI is high)
        uint32_t fclk = DEFAULT_F_CLK)
        : TMC5160_UART(serial, slaveAddress, fclk), _txEn(txEnablePin)
    {
        pinMode(_txEn, OUTPUT);
    }

  protected:
    void beginTransmission()
    {
        // TMC5160_UART::beginTransmission();
        digitalWrite(_txEn, HIGH);
    }

    void endTransmission()
    {
        _serial->flush();
        digitalWrite(_txEn, LOW);
    }

  private:
    uint8_t _txEn;
};

#endif // TMC5160_H
