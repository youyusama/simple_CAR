#ifndef BTOR2_FRONTEND_H
#define BTOR2_FRONTEND_H

#include "model/Btor2IR.h"

#include <string>

namespace car {

// Parse BTOR2 into owned IR and validate the safety-property interface.
class Btor2Frontend {
  public:
    static Btor2IR LoadIR(const std::string &path);

  private:
    static Btor2IR Parse(const std::string &path);
    static void Validate(const Btor2IR &ir);
};

} // namespace car

#endif
