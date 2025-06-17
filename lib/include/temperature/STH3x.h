#pragma once

namespace stm32plus
{

template <class TI2CPeripheral>
class STH3x : public TI2CPeripheral
{
public:
    enum
    {
        I2C_BUS_ADDRESS = 0x44 << 1
    };

    enum SHT3xSensor
    {
        SHT30,
        SHT31,
        SHT35
    };

    enum Command
    {
        /* 软件复位命令 */

        SOFT_RESET_CMD = 0x30A2,
        /*
        单次测量模式
        命名格式：Repeatability_CS_CMD
        CS：Clock stretching
        */
        HIGH_ENABLED_CMD = 0x2C06,
        MEDIUM_ENABLED_CMD = 0x2C0D,
        LOW_ENABLED_CMD = 0x2C10,
        HIGH_DISABLED_CMD = 0x2400,
        MEDIUM_DISABLED_CMD = 0x240B,
        LOW_DISABLED_CMD = 0x2416,

        /*
        周期测量模式
        命名格式：Repeatability_MPS_CMD
        MPS：measurement per second
        */
        HIGH_0_5_CMD = 0x2032,
        MEDIUM_0_5_CMD = 0x2024,
        LOW_0_5_CMD = 0x202F,
        HIGH_1_CMD = 0x2130,
        MEDIUM_1_CMD = 0x2126,
        LOW_1_CMD = 0x212D,
        HIGH_2_CMD = 0x2236,
        MEDIUM_2_CMD = 0x2220,
        LOW_2_CMD = 0x222B,
        HIGH_4_CMD = 0x2334,
        MEDIUM_4_CMD = 0x2322,
        LOW_4_CMD = 0x2329,
        HIGH_10_CMD = 0x2737,
        MEDIUM_10_CMD = 0x2721,
        LOW_10_CMD = 0x272A,
        /* 周期测量模式读取数据命令 */
        READOUT_FOR_PERIODIC_MODE = 0xE000,

    };

    struct CalibrationFactors
    {
        CalibrationFactors() : Factor(1.), Shift(0.) { }
        float Factor;
        float Shift;
    };

    enum TemperatureScale
    {
        Cel,
        Far,
        Kel
    };

    enum AbsHumidityScale
    {
        mmHg,
        Torr, //same as mm Hg
        Pa,
        Bar,
        At, //Techical atmosphere
        Atm, //Standart atmosphere
        mH2O,
        psi,
    };

    struct Parameters : TI2CPeripheral::Parameters
    {
        bool ADDR;
    };

protected:
    uint8_t buffer[6];
    float _TemperatureCeil;
    float _RelHumidity;
    CalibrationFactors _TemperatureCalibration;
    CalibrationFactors _RelHumidityCalibration;

    /*
        * 	Factors for poly for calculating absolute humidity (in Torr):
        *	P = (RelativeHumidity /100%) * sum(_AbsHumPoly[i]*T^i)
        *	where P is absolute humidity (Torr/mm Hg),
        *	T is Temperature(Kelvins degree) / 1000,
        * 	^ means power.
        *	For more data, check the NIST chemistry webbook:
        *	http://webbook.nist.gov/cgi/cbook.cgi?ID=C7732185&Units=SI&Mask=4&Type=ANTOINE&Plot=on#ANTOINE
    */
    float _AbsHumPoly[6] = {-157.004, 3158.0474, -25482.532, 103180.197, -209805.497, 171539.883};

    bool crc8(uint8_t MSB, uint8_t LSB, uint8_t crc);

public:
    STH3x(Parameters & params);

    bool initialise() const;
    void softReset() const;

    void updateData();

    float getTemperature(TemperatureScale degree);
    float getRelHumidity();
    void SetTemperatureCalibrationFactors(CalibrationFactors TemperatureCalibration);
    void SetRelHumidityCalibrationFactors(CalibrationFactors RelHumidityCalibration);
    float GetAbsHumidity(AbsHumidityScale scale);
    float GetTempTolerance(TemperatureScale Degree, SHT3xSensor SensorType);
    float GetRelHumTolerance(SHT3xSensor SensorType);

    // float GetAbsHumTolerance(AbsHumidityScale Scale, SHT3xSensor SensorType);


private:
};

template <class TI2CPeripheral>
bool STH3x<TI2CPeripheral>::crc8(uint8_t MSB, uint8_t LSB, uint8_t crc_)
{
    /*
    *	Name  : CRC-8
    *	Poly  : 0x31	x^8 + x^5 + x^4 + 1
    *	Init  : 0xFF
    *	Revert: false
    *	XorOut: 0x00
    *	Check : for 0xBE,0xEF CRC is 0x92
    */
    uint8_t crc = 0xFF;
    uint8_t i;
    crc ^= MSB;

    for (i = 0; i < 8; i++)
        crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;

    crc ^= LSB;
    for (i = 0; i < 8; i++)
        crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;

    if (crc == crc_)
    {
        return true;
    }
    else
    {
        return false;
    }
}

template <class TI2CPeripheral>
STH3x<TI2CPeripheral>::STH3x(Parameters & params) : TI2CPeripheral(params), buffer{0}
{
    this->setSlaveAddress(I2C_BUS_ADDRESS);
}

template <class TI2CPeripheral>
bool STH3x<TI2CPeripheral>::initialise() const
{
    // this->writeByte(MEDIUM_2_CMD >> 8, MEDIUM_2_CMD & 0xFF);
    return this->writeBytes(MEDIUM_1_CMD, nullptr, 0);
}

template <class TI2CPeripheral>
void STH3x<TI2CPeripheral>::softReset() const
{
    // this->prepareWrite(SOFT_RESET_CMD);
    this->writeBytes(SOFT_RESET_CMD, nullptr, 0);
    MillisecondTimer::delay(2);
}

template <class TI2CPeripheral>
void STH3x<TI2CPeripheral>::updateData()
{
    this->readBytes(READOUT_FOR_PERIODIC_MODE, buffer, 6);
    if (crc8(buffer[0], buffer[1], buffer[2]) && crc8(buffer[3], buffer[4], buffer[5]))
    {
        uint16_t temperatureRaw = buffer[0] << 8 | buffer[1];
        uint16_t RelHumidityRaw = buffer[3] << 8 | buffer[4];
        _TemperatureCeil = ((float)temperatureRaw) * 0.00267033 - 45.;
        _TemperatureCeil = _TemperatureCeil * _TemperatureCalibration.Factor + _TemperatureCalibration.Shift;

        _RelHumidity = ((float)RelHumidityRaw) * 0.0015259;
        _RelHumidity = _RelHumidity * _RelHumidityCalibration.Factor + _RelHumidityCalibration.Shift;
    }
    else
    {
    }
}

template <class TI2CPeripheral>
float STH3x<TI2CPeripheral>::getTemperature(TemperatureScale degree)
{
    float temperature = _TemperatureCeil;
    if (degree == TemperatureScale::Kel)
    {
        temperature += 273.15;
    }
    else if (degree == TemperatureScale::Far)
    {
        temperature = temperature * 1.8 + 32;
    }
    return temperature;
}

template <class TI2CPeripheral>
float STH3x<TI2CPeripheral>::getRelHumidity()
{
    return _RelHumidity;
}

template <class TI2CPeripheral>
void STH3x<TI2CPeripheral>::SetTemperatureCalibrationFactors(CalibrationFactors TemperatureCalibration)
{
    _TemperatureCalibration = TemperatureCalibration;
}

template <class TI2CPeripheral>
void STH3x<TI2CPeripheral>::SetRelHumidityCalibrationFactors(CalibrationFactors RelHumidityCalibration)
{
    _RelHumidityCalibration = RelHumidityCalibration;
}

template <class TI2CPeripheral>
float STH3x<TI2CPeripheral>::GetAbsHumidity(AbsHumidityScale scale)
{
    float millikelvins = GetTemperature(Kel) / 1000.;
    float Pressure = 0.;
    for (uint8_t i = 0; i < 6; i++)
    {
        float term = 1.;
        for (uint8_t j = 0; j < i; j++)
        {
            term *= millikelvins;
        }
        Pressure += term * _AbsHumPoly[i];
    }
    Pressure *= getRelHumidity();
    switch (scale)
    {
        case mmHg: {
            //Already in mm Hg
            break;
        }
        case Torr: {
            //Already in Torr
            break;
        }
        case Pa: {
            Pressure *= 133.322;
            break;
        }
        case Bar: {
            Pressure *= 0.0013332;
            break;
        }
        case At: {
            Pressure *= 0.0013595;
            break;
        }
        case Atm: {
            Pressure *= 0.0013158;
            break;
        }
        case mH2O: {
            Pressure *= 0.013595;
            break;
        }
        case psi: {
            Pressure *= 0.019337;
            break;
        }
        default: {
            break;
        }
    }
    return Pressure;
}

template <class TI2CPeripheral>
float STH3x<TI2CPeripheral>::GetTempTolerance(TemperatureScale Degree, SHT3xSensor SensorType)
{
    //Temperature tolerance is similar for both SHT30 and SHT31
    //At first, calculate at Celsius (similar to Kelvins), than, if need, recalculate to Farenheit

    float Temperature = getTemperature();
    float Tolerance;
    switch (SensorType)
    {
        case SHT30: {
            if ((0. <= Temperature) && (Temperature <= 65.))
            {
                Tolerance = 0.2;
            }
            else if (Temperature > 65.)
            {
                //Linear from 0.2 at 65 C to 0.6 at 125 C.
                Tolerance = 0.0067 * Temperature - 0.2333;
            }
            else //if (Temperature < 0.)
            {
                //Linear from 0.6 at -40 C to 0.2 at 0 C.
                Tolerance = -0.01 * Temperature + 0.2;
            }
            break;
        }
        case SHT31: {
            if ((0. <= Temperature) && (Temperature <= 90.))
            {
                Tolerance = 0.2;
            }
            else if (Temperature > 65.)
            {
                //Linear from 0.2 at 90 C to 0.5 at 125 C.
                Tolerance = 0.0086 * Temperature - 0.5714;
            }
            else //if (Temperature < 0.)
            {
                //Linear from 0.3 at -40 C to 0.2 at 0 C.
                Tolerance = -0.0025 * Temperature + 0.2;
            }
            break;
        }
        case SHT35: {
            if (Temperature <= 0.)
            {
                Tolerance = 0.2;
            }
            else if ((0. < Temperature) && (Temperature <= 20.))
            {
                //Linear from 0.2 at 0 C to 0.1 at 20 C.
                Tolerance = -0.005 * Temperature + 0.2;
            }
            else if ((20. < Temperature) && (Temperature <= 60.))
            {
                Tolerance = 0.1;
            }
            else if ((60. < Temperature) && (Temperature <= 90.))
            {
                //Linear from 0.1 at 60 C to 0.2 at 90 C.
                Tolerance = -0.0033 * Temperature - 0.1;
            }
            else //if (90. < Temperature)
            {
                //Linear from 0.2 at 90 C to 0.4 at 125 C.
                Tolerance = 0.0057 * Temperature - 0.3143;
            }
            break;
        }
    }
    if (Degree == Far)
    {
        Tolerance *= 1.8;
    }


    return Tolerance;
    // return ReturnValueIfError(Tolerance);
}

template <class TI2CPeripheral>
float STH3x<TI2CPeripheral>::GetRelHumTolerance(SHT3xSensor SensorType)
{
    float RelHumidity = getRelHumidity();
    float Tolerance;
    switch (SensorType)
    {
        case SHT30: {
            if ((10. <= RelHumidity) && (RelHumidity <= 90.))
            {
                Tolerance = 2.;
            }
            else if (RelHumidity < 10.)
            {
                //Linear from 4 at 0% to 2 at 10%
                Tolerance = -0.2 * RelHumidity + 4.;
            }
            else
            {
                //Linear from 2 at 90% to 4 at 100%
                Tolerance = 0.2 * RelHumidity - 16.;
            }
            break;
        }
        case SHT31: {
            Tolerance = 2.;
            break;
        }
        case SHT35: {
            if (RelHumidity <= 80.)
            {
                Tolerance = 1.5;
            }
            else //if (80 < RelHumidity)
            {
                //Linear from 0.5 at 80% to 2 at 100%
                Tolerance = 0.025 * RelHumidity - 0.5;
            }
            break;
        }
    }
    // return ReturnValueIfError(Tolerance);
    return Tolerance;
}
// template <class TI2CPeripheral>
// float STH30ControlI2C<TI2CPeripheral>::GetAbsHumTolerance(AbsHumidityScale Scale, SHT3xSensor SensorType)
// {
//     /*	Dependence of absolute humidity is similar (from 0 to 80C) to P = H*a*exp(b*T),
// *	where P is absolute humidity, H is relative humidity, T is temperature (Celsius),
// *	a ~= 0.0396, b~=0.0575.
// *	So its relative tolerance dP/P =  square root  [ (dH/H)^2 + (b*dT)^2 ].
// */
//
//     float RelHumidityRelTolerance = GetRelHumTolerance(SensorType)/getRelHumidity();
//     float TemperatureRelTolerance = 0.0575*GetTempTolerance(Cel, SensorType);
//     RelHumidityRelTolerance*=RelHumidityRelTolerance;
//     TemperatureRelTolerance*=TemperatureRelTolerance;
//     return	ReturnValueIfError(
//             GetAbsHumidity(Scale) * sqrt(RelHumidityRelTolerance+TemperatureRelTolerance)
//             );
// }


}