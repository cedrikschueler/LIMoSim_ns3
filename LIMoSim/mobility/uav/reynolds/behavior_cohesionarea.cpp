#include "behavior_cohesionarea.h"
#include "behaviors.h"
#include "LIMoSim/world/vehiclemanager.h"

namespace LIMoSim {

Behavior_CohesionArea::Behavior_CohesionArea(AreaLimits areaLimits, double height,
                                             std::string _agentId):
    Behavior("CohesionArea", _agentId),
    m_areaDefinitions(areaLimits),
    m_height(height)
{
}

Steering Behavior_CohesionArea::apply()
{

    std::vector<Vehicle*> carsInArea = findAgentsInArea();
    Vector3d meanPosition;

    if (!carsInArea.size()) {
        return Steering(Orientation3d(), (getAgent()->getVelocity() * -1) -getAgent()->getVelocity());
    }
    // compute mean position
    for (Vehicle * car : carsInArea) {
        meanPosition = meanPosition + car->getPosition();
    }

    meanPosition = meanPosition / (double)carsInArea.size();
    meanPosition.z = m_height;

    Behavior* arrive = new Behavior_Arrive(meanPosition, m_agentId);
    Steering steering = makeSelfAligned(arrive,m_agentId)->apply();
    delete arrive;
    return steering;
}

std::vector<Vehicle*> Behavior_CohesionArea::findAgentsInArea()
{
    std::map<std::string, Vehicle *> vehicles = VehicleManager::getInstance()->getVehicles("Car");
    std::vector<Vehicle*> vehiclesInArea;
    for (auto v : vehicles) {
        auto vehiclePosition = v.second->getPosition();
        if ( this->m_areaDefinitions.at(0) < vehiclePosition.x &&
             vehiclePosition.x< this->m_areaDefinitions.at(1) &&
             this->m_areaDefinitions.at(3) < vehiclePosition.y &&
             vehiclePosition.y < this->m_areaDefinitions.at(2)){
            vehiclesInArea.push_back(v.second);
        }
    }
    return vehiclesInArea;
}

} // namespace LIMoSim
