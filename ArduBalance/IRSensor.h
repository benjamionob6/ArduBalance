/*

*/
#include <Arduino.h> //For analogRead and the like

//generated by matlab script
const short linearizemm[] = {
  800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,
  800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,
  800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,
  800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,800,789,778,
  767,756,744,733,722,711,700,700,689,678,667,656,644,633,622,611,600,600,595,589,584,
  579,574,568,563,558,553,547,542,537,532,526,521,516,511,505,500,500,497,493,490,486,
  483,479,476,472,469,466,462,459,455,452,448,445,441,438,434,431,428,424,421,417,414,
  410,407,403,400,400,397,393,390,386,383,379,376,372,369,366,362,359,355,352,348,345,
  341,338,334,331,328,324,321,317,314,310,307,303,300,300,299,297,296,295,294,292,291,
  290,288,287,286,285,283,282,281,279,278,277,276,274,273,272,271,269,268,267,265,264,
  263,262,260,259,258,256,255,254,253,251,250,250,249,247,246,245,244,242,241,240,238,
  237,236,235,233,232,231,229,228,227,226,224,223,222,221,219,218,217,215,214,213,212,
  210,209,208,206,205,204,203,201,200,200,199,199,198,197,196,196,195,194,194,193,192,
  191,191,190,189,189,188,187,186,186,185,184,184,183,182,181,181,180,179,179,178,177,
  176,176,175,174,174,173,172,171,171,170,169,169,168,167,166,166,165,164,164,163,162,
  161,161,160,159,159,158,157,156,156,155,154,154,153,152,151,151,150,150,150,149,149,
  148,148,147,147,147,146,146,145,145,145,144,144,143,143,142,142,142,141,141,140,140,
  139,139,139,138,138,137,137,136,136,136,135,135,134,134,134,133,133,132,132,131,131,
  131,130,130,129,129,128,128,128,127,127,126,126,125,125,125,124,124,123,123,123,122,
  122,121,121,120,120,120,119,119,118,118,117,117,117,116,116,115,115,115,114,114,113,
  113,112,112,112,111,111,110,110,109,109,109,108,108,107,107,106,106,106,105,105,104,
  104,104,103,103,102,102,101,101,101,100,100,99,99,98,98,98,97,97,96,96,95,95,95,94,94,
  93,93,93,92,92,91,91,90,90,90,90,90,89,89,89,89,89,89,88,88,88,88,88,88,87,87,87,87,87,
  87,86,86,86,86,86,85,85,85,85,85,85,84,84,84,84,84,84,83,83,83,83,83,83,82,82,82,82,82,
  82,81,81,81,81,81,80,80,80,80,80,80,79,79,79,79,79,79,78,78,78,78,78,78,77,77,77,77,77,
  76,76,76,76,76,76,75,75,75,75,75,75,74,74,74,74,74,74,73,73,73,73,73,72,72,72,72,72,72,
  71,71,71,71,71,71,70,70,70,70,70,70,69,69,69,69,69,68,68,68,68,68,68,67,67,67,67,67,67,
  66,66,66,66,66,66,65,65,65,65,65,65,64,64,64,64,64,63,63,63,63,63,63,62,62,62,62,62,62,
  61,61,61,61,61,61,60,60,60
};

class IRSensor{
    int port;
    int num_readings;
    short readings[8];
    int buffstart, buffstop;
  public:
    void takeReading();
    short getLastReading();
    IRSensor(int);
    ~IRSensor();
};