#pragma once

#include "common.h"
#include "spectrum.h"

namespace bulbit
{

class Integrator;
class Medium;

bool V(const Integrator* integrator, const Point3 p1, const Point3 p2);
Spectrum Tr(const Integrator* integrator, const Point3 p1, const Point3 p2, const Medium* medium, int32 wavelength);

} // namespace bulbit
