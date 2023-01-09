#ifndef FIELD_H
#define FIELD_H

#include <vector>
#include "FieldElement.h"

//!!! ALL UNITS ARE IN INCHES !!!

struct Field {
     const double length = 651.25;
     const double width = 315.5;

     std::vector<FieldElement> fieldElements;
};

#endif