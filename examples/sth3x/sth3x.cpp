#include "config/stm32plus.h"

using namespace stm32plus;


class STH3xTest
{
protected:
    typedef STH3x<I2C1_Default<I2CTwoByteMasterPollingFeature>> MySTH3x;

    MySTH3x * sth3x;

    void run()
    {
        MySTH3x::Parameters params;

        sth30 = new MySTH3x(params);
        sth30->softReset();
        if (!sth30->initialise())
            return;
        sth30->updateData();
        float temp = sth30->getTemperature(Mysth30::Cel);
        float hum;
        hum = sth30->getRelHumidity();
    }
}