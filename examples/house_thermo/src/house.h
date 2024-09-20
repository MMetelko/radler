#include "radl__house.h"
#include <fstream>

class House {
 private:
  float temp;
  float leak_rate;
  float interval;
  std::ofstream outfile;
 public:
  House();
  ~House();
  void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);
};



