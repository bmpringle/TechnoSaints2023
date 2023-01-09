#ifndef FIELDELEMENT_H
#define FIELDELEMENT_H

#include <string>

struct FieldElement {
     double xBegin;
     double yBegin;

     double xEnd;
     double yEnd;

     std::string elementIdentifier; //name/identifier of the element. EX: chargeStationRed, chargeStationBlue, gridSlot1Red, gridSlot3Blue 
};

#endif