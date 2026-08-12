#ifndef WL_PACKAGE_RESIZE_H
#define WL_PACKAGE_RESIZE_H

#include "Btor2Frontend.h"
#include "WLTypes.h"

namespace car {

// Complete segment-level selective bitblasting and finite-domain resizing pass.
class WLPackageResize {
  public:
    static void Run(Btor2IR &ir, WLIRTraceMap &traceSources);
};

} // namespace car

#endif
