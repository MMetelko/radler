#include "thermometer.h"

#include <stdlib.h>
#include <iostream>

void Thermometer::Thermometer() {
  outfile.open("$ROS2_WS/install/data/thermometer_output.log", std::ios_base::app);
}

Thermometer::~Thermometer() {
  if (outfile.is_open()) {
    outfile.close();
  }
}

void Thermometer::step(const radl_in_t * in, const radl_in_flags_t* inflags,
                       radl_out_t * out, radl_out_flags_t* outflags){
  float noise = (float)(rand()%20000 - 10000)/10000;
  out->thermometer_temp->temp = in->house_temp->temp + noise;
  outfile << "Temperature : " << out->thermometer_temp->temp << " Noise : " << noise << std::endl;
}


