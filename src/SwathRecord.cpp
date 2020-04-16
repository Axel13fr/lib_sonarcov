#include "lib_sonarcov/SwathRecord.h"
#include "lib_sonarcov/AngleUtils.h"
#include <cmath>

using namespace scov;
using namespace ang;

EPoint SwathRecord::outerPoint(BoatSide side) const
{
    double swath_width = 0;
    double rotate_degs = 0;
    if (side == BoatSide::Stbd) {
        swath_width = this->swath_stbd;
        rotate_degs = 90;
    } else if (side == BoatSide::Port) {
        swath_width = this->swath_port;
        rotate_degs = -90;
    }

    auto orient_rad = headingToRadians(angle360(this->heading + rotate_degs));
    auto x_dot = cos(orient_rad) * swath_width;
    auto y_dot = sin(orient_rad) * swath_width;

    return EPoint(this->loc_x + x_dot, this->loc_y + y_dot);
}
