#include "thermometer.h"
#include <stdlib.h>
#include <iostream>

Thermometer::Thermometer() {
  outFilename = "/tmp/ros2_ws/data/thermometer_output.txt";
  outFile.open(outFilename, std::ios_base::app);
  if (!outFile.is_open()) {
    std::cerr << "Unable to open " << outFilename << " for writing." << std::endl;
  }
}

Thermometer::~Thermometer() {
 if (outFile.is_open()) {
    outFile.close();
  }
}

void Thermometer::step(const radl_in_t * in, const radl_in_flags_t* inflags,
                       radl_out_t * out, radl_out_flags_t* outflags){
  float noise = (float)(rand()%20000 - 10000)/10000;
  out->thermometer_temp->temp = in->house_temp->temp + noise;
  //std::cout << "Temperature : " << out->thermometer_temp->temp << " Noise : " << noise << std::endl;
  outFile << "Temperature : " << out->thermometer_temp->temp << " Noise : " << noise << std::endl;
}


