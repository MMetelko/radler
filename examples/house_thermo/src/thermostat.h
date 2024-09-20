#include "radl__thermostat.h"
#include <fstream>

class Thermostat {
 private:
  float set_temp;
  bool status;
  float tol;
  std::ofstream outfile;
 public:
  Thermostat();
  ~Thermostat();
  void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);
};
