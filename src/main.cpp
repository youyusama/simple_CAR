#include "Settings.h"
#include "SimpleCAR.h"
#include <cstdlib>

int main(int argc, char **argv) {
    car::Settings settings;
    if (!car::ParseSettings(argc, argv, settings)) return EXIT_FAILURE;

    car::SimpleCAR app(settings);
    if (!app.LoadModel()) return EXIT_FAILURE;
    if (!settings.wlBitblastOutputPath.empty()) {
        return EXIT_SUCCESS;
    }
    app.Prove();
    return 0;
}
