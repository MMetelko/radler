#include "thermostat.h"
#include <iostream>

Thermostat::Thermostat() {
  this->set_temp = 75.0;
  this->status = false;
  this->tol = 2.0;

  outFilename = "/tmp/ros2_ws/data/thermostat_output.txt";
  outFile.open(outFilename, std::ios_base::app);
  if (!outFile.is_open()) {
    std::cerr << "Unable to open " << outFilename << " for writing." << std::endl;
  }
}

Thermostat::~Thermostat() {
  if (outFile.is_open()) {
    outFile.close();
  }
}

void Thermostat::step(const radl_in_t * in, const radl_in_flags_t* inflags,
                      radl_out_t * out, radl_out_flags_t* outflags){
  // change the set temperature 
  this->set_temp = in->thermostat_set_temp->temp;

  // set the status
  this->status = in->thermostat_switch->status;

  // decide whether to switch on the heater
  if (in->thermometer_temp->temp > (this->set_temp + this->tol)) {
    out->heater_switch->switch_on = false;
  } else if (this->status && (in->thermometer_temp->temp < this->set_temp)) {
    out->heater_switch->switch_on = true;
  } else {
    out->heater_switch->switch_on = this->status;
  } 
  //std::cout << "Switch : " << out->heater_switch->switch_on << std::endl;
  outFile << "Switch : " << out->heater_switch->switch_on << std::endl;
}

