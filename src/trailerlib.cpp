#include <iostream>
#include "trailerlib.h"

int main(int argc, char* argv) {
    TruckTrailer trailer(3.7, 8.0, 2.6, 4.5, 1.0, 1.0, 9.0, 0.6, 0.5, 1.0);
    trailer.plot_trailer(0, 0, 0.1745, -0.1745, 0.0);

    return 0;
}