#ifndef BEHAVIOR_COHESIONAREA_H
#define BEHAVIOR_COHESIONAREA_H

#include <map>
#include<array>
#include "LIMoSim/mobility/uav/reynolds/behavior.h"

namespace LIMoSim {

class UAV;
class Vehicle;
class MobilityData;
typedef std::array<double, 4> AreaLimits;
struct RectArea {
    std::array<double, 2> topLeft;
    double xLength, yLength;
};

class Behavior_CohesionArea: public Behavior
{
public:    
    Behavior_CohesionArea(AreaLimits areaLimits, double height=30.0,
                          std::string _agentId = "");

    // Behavior interface
    Steering apply();

protected:
    virtual std::vector<Vehicle*> findAgentsInArea();

    AreaLimits m_areaDefinitions;
    double m_height;
};


} // namespace LIMoSim

#endif // BEHAVIOR_COHESIONAREA_H
