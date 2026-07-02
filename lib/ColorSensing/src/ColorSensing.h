#pragma once
/**
 * @author Vincent Rohkamm
 * @date: 18.11.2025
 * @description: User Interface for Robot
 */


#ifdef _MSC_VER
    #pragma region Includes //-----------------------------------------------------------------------
#endif

#include <Arduino.h>
#include <CustomDatatypes.h>
#include <Adafruit_AS7341.h>
#include <Adafruit_EEPROM_I2C.h>
#include <Adafruit_FRAM_I2C.h>

class UserInterface;

#ifdef _MSC_VER
    #pragma endregion Includes //-----------------------------------------------------------------------
    #pragma region eeprom
#endif

class EEPROM {
    private:

    #define EEPROM_ADDR 0x50  

    #define EEPROM_PACKAGE_SIZE 2
    #define EEPROM_PACKAGE_OVERHEAD 1
    #define EEPROM_PACKAGE_NUM 10

    #define EEPROM_START_ADD_FRONT_WHITE 0x000
    #define EEPROM_START_ADD_MIDDLE_WHITE 0x01E

    #define EEPROM_START_ADD_FRONT_BLUE 0x03C
    #define EEPROM_START_ADD_MIDDLE_BLUE 0x05A

    #define EEPROM_START_ADD_FRONT_CP 0x078
    #define EEPROM_START_ADD_MIDDLE_CP 0x096

    #define EEPROM_START_ADD_FRONT_BLACK 0x0B4
    #define EEPROM_START_ADD_MIDDLE_BlACK 0x0D2

    #define EEPROM_START_ADD_FRONT_DZ 0x0F0
    #define EEPROM_START_ADD_MIDDLE_DZ 0x10E

    #define EEPROM_START_ADD_REFLECTIVE 0x12C

    #define EEPROM_MAIN_SPEED 0x14A

    #define EEPROM_NEXT_AVALIBLE_ADDR 0x14C

    Adafruit_EEPROM_I2C i2ceeprom;

    int16_t GetStartAddr(PoI_Type type, char sensor);


    

    public:
        ErrorCodes Init();
        ErrorCodes WriteToEEPROM(PoI_Type type, char sensor, uint16_t* buffer);
        ErrorCodes ReadFromEEPROM(PoI_Type type, char sensor, uint16_t* buffer);
};

#ifdef _MSC_VER
    #pragma endregion  eeprom //-----------------------------------------------------------------------
    #pragma region Color Sensing
#endif


class ColorSensing{
    private:
    // --- Adressen & Hardware-Settings ---
        static constexpr uint8_t MULTIPLEX_ADRESS = 0x70;
        static constexpr uint8_t LED_CURRENT      = 10;

        static constexpr uint8_t REFL_CTRL = D6;
        static constexpr uint8_t REFL_READ = A1;

    // --- Sensor Timings ---
    //  Total integration time will be (ATIME + 1) * (ASTEP + 1) * 2.78µS
        static constexpr uint16_t ATIME_Front     = 100;
        static constexpr uint16_t ASTEP_Front     = 150;
        static constexpr uint16_t ATIME_Middle    = 150;
        static constexpr uint16_t ASTEP_Middle    = 700;
 
    // --- Multiplikatoren ---
        static constexpr uint8_t MULT_up          = 100;
        static constexpr uint8_t MULT_down        = 100;
 
    // --- Schwellenwerte Middle Sensor ---
        static constexpr uint16_t MIDDLE_BLACK_RANGE_UP   = 4000;
        static constexpr uint16_t MIDDLE_WHITE_RANGE_DOWN = 20000;
        static constexpr uint16_t MIDDLE_WHITE_RANGE_UP   = 3000;
        static constexpr uint16_t MIDDLE_SILVER_RANGE_F5  = 10000;
        static constexpr uint16_t MIDDLE_BLUE_RANGE_DOWN  = 2000;
        static constexpr uint16_t MIDDLE_RED_RANGE_DOWN   = 10000;
 
    // --- Schwellenwerte Front Sensor ---
        static constexpr uint16_t FRONT_BLACK_RANGE_UP    = 1500;
        static constexpr uint16_t FRONT_WHITE_RANGE_DOWN  = 1900;
        static constexpr uint16_t FRONT_WHITE_RANGE_UP    = 200;
 
    // --- Konfiguration ---
        static constexpr uint8_t RUNS_calibration = 7;
        static constexpr uint32_t COLOR_TIMEOUT   = 1000;
    
    // --- LEDs ---
        static constexpr bool _ENABLE_LED_FRONT  = true;
        static constexpr bool _ENABLE_LED_MIDDLE = true;

    // --- Float-Modifikatoren ---
    // WICHTIG: Das 'f' am Ende macht daraus einen 32-bit float statt 64-bit double!
    // Das spart auf einem STM32 massiv Rechenzeit und Flash-Speicher.
        static constexpr float MOD_FrontRead       = 1.0f;
        static constexpr float MOD_MiddleRead      = 2.5f;
        static constexpr float MOD_HIGH_WAVELENGTH = 2.5f;
        static constexpr float MOD_WHITE           = 1.5f;

    // --- History ---
        #define WINDOW_SIZE 7
        #define REFL_WINDOWS_SIZE 20
        #define NOISE_THRESHOLD 600
        #define FLICKER_MIN_COUNT 4

        uint16_t f5History[WINDOW_SIZE];
        uint16_t f6History[WINDOW_SIZE];
        uint16_t f7History[WINDOW_SIZE];
        uint8_t historyIndex = 0;
        bool _checkpoint = false;
        void UpdateHistory(uint16_t f5, uint16_t f6, uint16_t f7);

        uint16_t reflHistory[REFL_WINDOWS_SIZE];
        uint8_t refl_index = 0;

    // --- Objects for CS ---
        EEPROM* _eeprom;
        UserInterface* _ui;
        Adafruit_AS7341 front;
	    Adafruit_AS7341 middle;

    // private Fields for storing current state of the sensors
        PoI_Type colorFront;
        TileType colorMiddle;

        bool _ALERT;
        bool _FREEZE_SENSOR;
        bool _READING;

    // saved calibration Values
        static constexpr uint8_t WHITE  = 0;
        static constexpr uint8_t BLACK  = 1;
        static constexpr uint8_t BLUE   = 2;
        static constexpr uint8_t RED    = 3;
        static constexpr uint8_t SILVER = 4;

        rawColor frontColorsCalibrated[5];
        rawColor middleColorsCalibrated[5]; 

    // Diagnostic functionality
        Stream* _debugPort;
        void printDebugData(uint16_t* rawColor, char sensor);

    // --- Multiplexer ---
        #define MULTIPLEX_ADRESS 0x70
        /**
         * @brief toggles between multiplexer channels; enables use of multiple CS
         * @param bus: 0...front; 1...middle
         */
        void TCA9548A(uint8_t bus){
            Wire.beginTransmission(MULTIPLEX_ADRESS);    // TCA9548A address
            Wire.write(1 << bus);                   // send byte to select bus
            Wire.endTransmission();
        }

    // analyze Sensor Data
        // @todo: check function
        PoI_Type checkFront();
        TileType checkMiddle();

    public:

        /**
         * @brief Constructor for ColorSensing class
         * @param debugPort: optional Stream for debug output (e.g. Serial), if left empty, no debug output will be produced
         */
        ColorSensing(Stream* debugPort = nullptr) : _debugPort(debugPort) {}

        /**
         * @brief inits ColorSensing for Floor
         * @param wire: Wire channel
         * @param ui: UserInterface instance
         * @param eeprom: EEPROM instance
         *
         * @return	b1...Middle not found
		 * @return	b2...LED Error front
		 * @return	b3...LED Error middle
		 * @return	b4...Front GAIN Error
		 * @return	b5...Middle GAIN Error
		 * @return	b6...Middle ATIME / ASTEP Error
		 * @return	b7...Front ATIME / ASTEP Error
         */
        uint8_t Init(TwoWire* wire, UserInterface* ui, EEPROM* eeprom);

        /**
         * @brief returns current floor type
         * @return current floor type as PoI_Type
         */
        TileType GetFloor();

        /**
         * @brief returns current floor type using middle Sensor BLOCKING OPERATION!
         * @return current floor type as PoI_Type
         */
        TileType GetFloorBlocking();

        /**
         * @brief returns if the front sensor registers not White
         * @note returns false while frozen (ramp/turn) regardless of last reading
         * @return _ALERT: true...not white; false...white
         */
        bool GetAlert();

        /**
         * @brief freezes the cs output to white, use on ramp, etc
         * @param enable: true...freeze; false...normal operation
         * @return true...frozen, false...normal operation
         */
        bool Freeze(bool enable){ _checkpoint = false; return (_FREEZE_SENSOR = enable); }

        /**
         * @brief Gets Freeze status
         * @return true...frozen, false...normal operation
         */
        bool Freeze(){ return _FREEZE_SENSOR; }
        

        /**
         * @brief calibrates all color sensors to selected color
         * @param color: calibrated floor color
         * @return returns if successful
         */
        ErrorCodes Calibrate(PoI_Type color);

        /**
         * @brief starts or stops reading of the sensors
         * @param enable: true...start reading; false...stop reading
         */
        ErrorCodes EnableRead(bool enable);

        /**
         * @brief Checks if the sensors finished measurement and updates floor type
         */
        ErrorCodes Update();

        void resetCheckpoint() { _checkpoint = false; }
};