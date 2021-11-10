#ifndef storage
#define storage
#include <Maths.h>
#include <pid.h>

#define STORAGE_VERSION 3141 // 3.14159265359

#define STORAGE_SIZE_BOOL       (sizeof(bool)   * 1)
#define STORAGE_SIZE_FLOAT      (sizeof(float)  * 1)
#define STORAGE_SIZE_VEC3       (sizeof(double) * 3)
#define STORAGE_SIZE_QUATERNION (sizeof(double) * 4)
#define STORAGE_SIZE_MATRIX3    (sizeof(double) * 9)
#define STORAGE_SIZE_PID        (sizeof(float)  * 6)

enum BoolValues {
    propsIn,
    useLeds,

    useAntiGravity,

    useVCell,

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

    batLpf,
    batMul,

    insSensorFusion,

    FloatValuesCount
};

enum Vec3Values {
    accOffset,
    accScale,
    gyroOffset,
    magOffset,
    magScale,

    Vec3ValuesCount
};

enum QuaternionValues {
    accAngleOffset,

    QuaternionValuesCount
};

enum Matrix3Values {

    Matrix3ValuesCount
};

enum PidValues {
    ratePidR,
    ratePidP,
    ratePidY,
    levelPidR,
    levelPidP,
    levelPidY,

    altitudePid,
    velPid,

    PidValuesCount
};

class Storage {
public:

    static void begin();

    static bool read(BoolValues floatVal);
    static float read(FloatValues floatVal);
    static Vec3 read(Vec3Values floatVal);
    static Quaternion read(QuaternionValues quatVal);
    static Matrix3 read(Matrix3Values matVal);
    static PID read(PidValues pidVal);

    static void write(BoolValues floatVal, bool flag);
    static void write(FloatValues floatVal, float val);
    static void write(Vec3Values floatVal, Vec3 vec);
    static void write(QuaternionValues quatVal, Quaternion quat);
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
    static int quaternionStart();
    static int matrix3Start();
    static int pidStart();
    static int size();
    static const int offset = 0;
};

#endif