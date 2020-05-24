//
// Created by yu on 13.05.20.
//
#include "resultSet.hpp"

std::ostream& operator << (std::ostream& os, const DistIndex& DI)
{
    os << "Distance = " << DI.distance << ", Index = " << DI.index;
    return os;
}
