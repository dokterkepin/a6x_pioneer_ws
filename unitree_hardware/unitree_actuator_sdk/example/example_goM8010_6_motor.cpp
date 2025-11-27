#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"


int main() {

  SerialPort  serial("/dev/ttyUSB0");
  MotorCmd    cmd;
  MotorData   data;

  while(true) 
  {
    cmd.motorType = MotorType::GO_M8010_6;
    data.motorType = MotorType::GO_M8010_6;
    cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::CALIBRATE);

    cmd.mode = queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
    cmd.id   = 0;
    cmd.kp   = 0.0 / (queryGearRatio(MotorType::GO_M8010_6) * queryGearRatio(MotorType::GO_M8010_6));
    cmd.kd   = 0.0 / (queryGearRatio(MotorType::GO_M8010_6) * queryGearRatio(MotorType::GO_M8010_6));
    cmd.q   = 0.0 * queryGearRatio(MotorType::GO_M8010_6);
    cmd.dq   = 0.0;
    cmd.tau  = 0.0;
    serial.sendRecv(&cmd, &data);

    std::cout <<  std::endl;
    std::cout <<  "motor.q: "    << data.q    <<  std::endl;
    std::cout <<  "motor.temp: "   << data.temp   <<  std::endl;
    std::cout <<  "motor.W: "      << data.dq      <<  std::endl;
    std::cout <<  "motor.merror: " << data.merror <<  std::endl;
    std::cout <<  std::endl;

    usleep(200);
  }

}