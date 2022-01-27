#include <iostream>

#include "ICM20649.hpp"
#include "utils.hpp"

int
main()
{
  ICM20649::Config config;
  ICM20649::Device device(config);

  float data[3];

  while (ICM20649::get_key() != 27)
  {
    std::cout << "\nGYROSCOPE [deg/s]\n";
    std::cout << "resolution: " << device.gyroscope_resolution() << '\n';
    device.gyroscope_d(data[0], data[1], data[2]);
    std::cout << data[0] << ' ' << data[1] << ' ' << data[2] << '\n';

    std::cout << "ACCELEROMETER [m/s2]\n";
    std::cout << "resolution: " << device.accelerometer_resolution() << '\n';
    device.accelerometer(data[0], data[1], data[2]);
    std::cout << data[0] << ' ' << data[1] << ' ' << data[2] << '\n';

    std::cout << "TEMPERATURE [C]\n";
    std::cout << device.temperature() << '\n';

    std::cout << "\nPress 'ESC' to exit\n";

#if defined(__unix__)
    std::cout << "\033[F\033[F\033[F\033[F\033[F\033[F\033[F\033[F\033[F\033["
                 "F\033[F\r";
#endif

    usleep(33000);
  }

  return 0;
}
