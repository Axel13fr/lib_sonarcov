#include "lib_sonarcov/SwathRecord.h"
#include <cmath>

using namespace scov;

//---------------------------------------------------------------
// Procedure: angle360
//   Purpose: Convert angle to be strictly in the rang [0, 360).

static double angle360(double degval)
{
    while (degval >= 360.0)
        degval -= 360.0;
    while (degval < 0.0)
        degval += 360.0;
    return (degval);
}

//---------------------------------------------------------------
// Procedure: radAngleWrap

static double radAngleWrap(double radval)
{
    if ((radval <= M_PI) && (radval >= -M_PI))
        return (radval);

    if (radval > M_PI)
        return (radval - (2 * M_PI));
    else
        return (radval + (2 * M_PI));
}

//---------------------------------------------------------------
// Procedure: headingToRadians
// Converts true heading (clockwize from N) to
// radians in a counterclockwize x(E) - y(N) coordinate system
// .

static double headingToRadians(double degval)
{
    return (radAngleWrap((90.0 - degval) * M_PI / 180.0));
}

EPoint SwathRecord::outerPoint(BoatSide side) const
{
    // Could have SwathRecord be a class with functions to return representation
    // as a vector or point.
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
