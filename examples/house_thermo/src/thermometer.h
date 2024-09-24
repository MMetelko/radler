#include "radl__thermometer.h"
#include <fstream>
#include <string>

class Thermometer {
 private:
  std::string outFilename;
  std::ofstream outFile;

 public: 
  Thermometer();
  ~Thermometer();
  void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);
};
