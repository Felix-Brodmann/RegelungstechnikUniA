#include "../include/mpu6050.hpp"
#include "../include/complementaryFilter.hpp"
#include <iostream>
#include <vector>
#include <iomanip>
#include <fstream>
#include <thread>
#include <chrono>

int main()
{
    RegelungstechnikUniA::MPU6050 mpu;
    try
    {
        std::cout << "Initializing MPU6050" << std::endl;
        mpu.init("/dev/i2c-1");
        std::cout << "Powering up MPU6050" << std::endl;
        mpu.powerUp(false);
        std::cout << "Setting sample frequency and range" << std::endl;
        mpu.setSampleFrequency(1000);
        mpu.setGyroRange(FS_SEL_500);
        mpu.setAccelRange(AFS_SEL_2G);
        std::cout << "Initializing MPU6050 complete" << std::endl << std::endl;
        std::cout << std::fixed << std::setprecision(3);
        std::ofstream file("data.csv");
        file << "Zeit [ms],Winkelgeschwindigkeit Z-Achse[°/s],Beschleunigung X-Achse[g],Beschleunigung Y-Achse[g], Winkel[°]" << std::endl;
        std::cout << std::endl << "Kalibriere Gyroskop. Dies dauert 20 Sekunden" << std::endl;
        mpu.calibrateGyro(20);
        std::cout << "Kalibrierung abgeschlossen" << std::endl << std::endl;
        std::vector<double> gyro;
        std::vector<double> accel;
        RegelungstechnikUniA::ComplementaryFilter filter(0.98);
        usleep(10000);
        int modulo = 0;
        while (true)
        {
            gyro = mpu.readGyro(IN_DEG_PER_S);
            accel = mpu.readAccel(IN_G);;
            filter.update(gyro[Z_AXIS], accel[X_AXIS], accel[Y_AXIS]);
            if (modulo % 10 == 0)
            {
                std::cout << "Winkel: " << filter.getAngle() << std::endl;
            }
            modulo++;
            usleep(10000);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
    return 0;
}