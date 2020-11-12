// This is all the parameters and variables for the sensors - column 6 is a dummy for a boolean input (Engine Coolant Level)
//                                    Turbo       Tbox temp     EGT       Oil Press   Oil Temp  Cool Lev  Roll
//const char *senseName[] =             {"BST",     "BOX",        "EGT",    "O_P",      "O_T",    "LVL",    "ANG"   };
//const bool senseactive[] =            { true,     true,         true,     true,       true,     true,     true    }; // sensor is enabled or not
//bool warnhistatus[] =                 { false,    false,        false,    false,      false,    false,    false   }; // flag for alternating warning animations
//uint8_t sensefault[] =                { 0,        0,            0,        0,          0,        0,        0       }; // flag set when sensor in error state.
////const unsigned char* senseglyphs[] =  { trbBMP,   tboxBMP,      egtBMP,   eopBMP,     eotBMP,   coollev,   D2BMP  }; // pointers to our bitmaps
//int sensevals[] =                     { 0,        0,            -20,      50,         101,      0,        0       }; // bogus start values for debug
//const uint8_t senseunits[] =          { 1,        0,            0,        1,          0,        2,        3       }; // 0 for C 1 for psi 2 for "OK" 3 for "deg"
//const int sensemaxvals[] =            { 32,       150,          900,      72,         150,      1,        45      }; // the maximum scale to use in our graph for each sensor
//const int8_t senseminvals[] =         { 0,        -40,          -40,      0,          -40,      0,        -45     }; // the minimum scale to use in our graph for each sensor
//int sensepeakvals[] =                 { 0,        -40,          -40,      0,          -40,      1,        0       }; // somewhere for us to store peak values
//const int sensewarnhivals[] =         { 29,       140,          750,      60,         100,      999,      30      }; // High value that we want to be warned if exceeded
//const int sensewarnlowvals[] =        { -999,     -999,         -999,     20,         -999,     -999,     -30     }; // Low value that we want to be warned if exceeded (-999 means we don't care)


// Greg's modified list for RGV
//                              Sensor#  0           1            2          3          4        5         6
//                                    TC/i2c?     TC/i2c?        i2c        i2c      TC/i2c?   Dig:Hallx2  i2c
//                                     CHT-1       CHT-2        Tire-F    Tire-R     Ambient   Gear?      Lean (calc'd from rpm and speed)
const char *sensorName[] =            {"CHT #1", "CHT #2", "Tire Temp - Front", "Tire Temp - Rear", "Ambient Temp", "Gear Position", "Lean Angle"}; // Names for future use
const char *sensorLabel[] =           {"CH1",     "CH2",        "T-F",    "T-R",      "AMB",    "GPI",    "ANG"     }; // This 3 letter abbreviation is the label on the UI
const bool senseactive[] =            { true,     true,         true,     true,       true,     true,     true      }; // sensor is enabled or not
bool warnhistatus[] =                 { false,    false,        false,    false,      false,    false,    false     }; // flag for alternating warning animations
uint8_t sensefault[] =                { 0,        0,            0,        0,            0,        0,        0       }; // flag set when sensor in error state.
int sensevals[] =                     { 25,       25,           25,       25,           40,       1,        0       }; // bogus start values for debug
const uint8_t senseunits[] =          { 0,        0,            0,        0,            0,        4,        3       }; // 0 for C 1 for psi 2 for "OK" 3 for "deg", 4 for "count"
const int sensemaxvals[] =            { 300,      300,          150,      150,          120,      6,        60      }; // the maximum scale to use in our graph for each sensor
const int8_t senseminvals[] =         { -10,      -10,          -10,      -10,          -10,      0,        -60     }; // the minimum scale to use in our graph for each sensor
int sensepeakvals[] =                 { 0,        0,             0,       0,            0,        0,        0       }; // somewhere for us to store peak values
const int sensewarnhivals[] =         { 80,       80,           100,      100,          100,      10,       55      }; // High value that we want to be warned if exceeded
const int sensewarnlowvals[] =        { -999,    -999,          -999,     -999,         45,       -999,     -55     }; // Low value that we want to be warned if exceeded (-999 means we don't care)
