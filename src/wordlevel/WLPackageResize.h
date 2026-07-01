#ifndef WL_PACKAGE_RESIZE_H
#define WL_PACKAGE_RESIZE_H

#include "Btor2Frontend.h"

namespace car {

// IR-to-IR finite-domain package resizing pass.
class WLPackageResize {
  public:
    static void Run(Btor2IR &ir);
};

} // namespace car

#endif
