#pragma once

#include <Global.h>

class Cyclone2 {
protected:
    static void setupClock();
    static void setupSMC();
    static void setupEBI();
public:
    static bool Init();
};