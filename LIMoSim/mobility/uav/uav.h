#ifndef LIMOSIM_UAV_H
#define LIMOSIM_UAV_H

#include "LIMoSim/mobility/vehicle.h"
#include "reynolds/uav_reynoldsmodel.h"

namespace LIMoSim
{
class UAV_ReynoldsModel;
class UAV : public Vehicle
{
    friend class Behavior;
    friend class Steering;
    friend class Locomotion;

public:
    UAV(const std::string &_id);

    // Reynoldsmodel related methods
    UAV_ReynoldsModel *getModel();
    void setAccelerationMax(double _accMax);
    void setBehavior(Behavior* _behavior);
    void setVelocityMax(double _velMax);
    Vector3d getPredictedPosition(double _timeDelta_s) override;

protected:

    void initialize();
    void handleEvent(Event *_event);
    void move(double _timeDelta_s);
    Vector3d getWaypoint();

    virtual Vector3d predictWithTarget(Vector3d _currentData, int m_updateInterval_ms,
                               Vector3d wp0);
    virtual Vector3d
    predictWithHistory(
        std::vector<Vector3d> _historyData, std::vector<double> _historyDataTimes,int _nextTime_ms);
    double getMaxSpeed() {return getModel()->getLocomotion()->getVelocityMax();}

private:
    Vector3d m_initialPosition;
    Vector3d generateNormedNoiseVector();
};

}

#endif // LIMOSIM_UAV_H
