#ifndef storage
#define storage
#include <Maths.h>
#include <pid.h>

#define STORAGE_VERSION 31415 // 3.14159265359

#define STORAGE_SIZE_BOOL       (sizeof(bool) * 1)
#define STORAGE_SIZE_FLOAT      (sizeof(float) * 1)
#define STORAGE_SIZE_VEC3       (sizeof(double) * 3)
#define STORAGE_SIZE_MATRIX3    (sizeof(double) * 9)
#define STORAGE_SIZE_PID        (sizeof(float) * 6)

enum BoolValues {
    propsIn,
    useLeds,

    useAntiGravity,

    BoolValuesCount
};

enum FloatValues {
    accLPF,
    gyroLPF,

    accInsInf,
    magInsInf,

	loopFreqRate,
	loopFreqLevel,

	iRelaxMinRate,

	antiGravityMul,
    boostLpf,
    boostSpeed,

    insAccMaxG,

    FloatValuesCount
};

enum Vec3Values {
    accOffset,
    gyroOffset,
    magHardIron,

    accMul,
    gyroMul,
    magMul,

    Vec3ValuesCount
};

enum Matrix3Values {
    magSoftIron,

    Matrix3ValuesCount
};

enum PidValues {
    ratePidR,
    ratePidP,
    ratePidY,
    levelPidR,
    levelPidP,
    levelPidY,

    PidValuesCount
};

class Storage {
public:

    static void begin();

    static bool read(BoolValues floatVal);
    static float read(FloatValues floatVal);
    static Vec3 read(Vec3Values floatVal);
    static Matrix3 read(Matrix3Values matVal);
    static PID read(PidValues pidVal);

    static void write(BoolValues floatVal, bool flag);
    static void write(FloatValues floatVal, float val);
    static void write(Vec3Values floatVal, Vec3 vec);
    static void write(Matrix3Values matVal, Matrix3 mat);
    static void write(PidValues matVal, PID mat);

    static void writeDefaults();

private:
    static const int eepromOffset = 0;
    static const int eepromSize = 1080;

    static void erase();

    static int boolStart();
    static int floatStart();
    static int vec3Start();
    static int matrix3Start();
    static int pidStart();
    static int size();
    static const int offset = 0;
};

#endif