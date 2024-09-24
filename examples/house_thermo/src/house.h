#include "radl__house.h"
#include <fstream>
#include <string>

class House {
 private:
  float temp;
  float leak_rate;
  float interval;
  std::string outFilename;
  std::ofstream outFile;
 public:
  House();
  ~House();
  void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);
};



