#include "radl__thermometer.h"
#include <fstream>

class Thermometer {
 private:
  std::ofstream outfile;

 public: 
  Thermometer()
  ~Thermometer()
  void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);
};
