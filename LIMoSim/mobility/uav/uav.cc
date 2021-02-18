#include "uav.h"

#include "LIMoSim/mobility/uav/reynolds/uav_reynoldsmodel.h"
#include "LIMoSim/mobility/uav/reynolds/locomotionupdate.h"
#include "LIMoSim/mobility/uav/reynolds/behavior.h"

namespace LIMoSim
{

UAV::UAV(const std::string &_id) :
    Vehicle(_id, "UAV")
{
    setModel(new UAV_ReynoldsModel(_id));
    initialize();
}

UAV_ReynoldsModel *UAV::getModel()
{
    return dynamic_cast<UAV_ReynoldsModel*>(Vehicle::getModel());
}

void UAV::setAccelerationMax(double _accMax)
{
    getModel()->getLocomotion()->setAccelerationMax(_accMax);
}

void UAV::setBehavior(Behavior *_behavior)
{
    _behavior->setAgent(this->getId());
    getModel()->setBehavior(_behavior);
}

void UAV::setVelocityMax(double _velMax)
{
    getModel()->getLocomotion()->setVelocityMax(_velMax);
}


/*************************************
 *          PROTECTED METHODS        *
 ************************************/

void UAV::initialize()
{
    Vehicle::initialize();
}

void UAV::handleEvent(Event *_event)
{
    Vehicle::handleEvent(_event);

}

void UAV::move(double _timeDelta_s)
{
    LocomotionUpdate update = getModel()->step(_timeDelta_s);
//    if (rand() * 1.0 / RAND_MAX > 0.7) {
//        double noiseMagnitude = 0 + ((double)rand() / RAND_MAX) * 0.5;
//        update.position = update.position + generateNormedNoiseVector() * noiseMagnitude;
//    }
    setPosition(update.position);
    setVelocity(update.velocity);
    setAcceleration(update.acceleration);
    setOrientation(update.orientation);
    setOrientationVelocity(update.orientationVelocity);
}

Vector3d UAV::getWaypoint(){
    getModel()->getWaypoint();
}

Vector3d UAV::getPredictedPosition(double _timeDelta_s) {
  if (_timeDelta_s <= 0) {
    return (m_positionHistory.size() > 0) ? m_positionHistory.back().second
                                          : Vector3d(0, 0, 0);
  }
  if (m_positionHistory.size() > 0) {
    double m_updateInterval_ms = m_updateInterval_s * 1000;
    double _currentTime_ms = m_positionHistory.back().first * 1000;

    std::deque<Vector3d> predictedData;
    std::deque<std::pair<double, Vector3d>> historyData = m_positionHistory;

    Vector3d nextTarget = getWaypoint();
    bool hasWaypoint = true;

    int time_ms =
        (int)floor(_currentTime_ms / m_updateInterval_ms) * m_updateInterval_ms;
    Vector3d lastValidData = m_positionHistory.back().second;
    Vector3d currentData = lastValidData;
    for (int i = 0; i < (int)floor(_timeDelta_s / m_updateInterval_s); i++) {
      time_ms += m_updateInterval_ms;
      if (hasWaypoint) {
        currentData =
            predictWithTarget(currentData, m_updateInterval_ms, nextTarget);
        if ((nextTarget - currentData).norm() <= WP_REACHED_RANGE) {
          hasWaypoint = false;
        }
      } else {
        currentData = predictWithHistory(historyData, time_ms);
      }
      predictedData.push_back(currentData);
      historyData.push_back(std::make_pair(time_ms, currentData));
      historyData.pop_front();
    }
return predictedData[predictedData.size () - 1];
  } else {
    return Vector3d(0, 0, 0);
  }
}

Vector3d UAV::predictWithTarget(Vector3d _currentData,
                                    int m_updateInterval_ms, Vector3d wp0) {
  Vector3d nextData = _currentData;

  // compute the next position
  Vector3d position = _currentData;
  Vector3d target = wp0;
  double m_maxSpeed_mpMs = getMaxSpeed();

  nextData =
      position + (target - position) / ((target - position).norm() *
                                        (m_updateInterval_ms)*m_maxSpeed_mpMs);

  return nextData;
}

Vector3d UAV::predictWithHistory(
    std::deque<std::pair<double, Vector3d>> _historyData, int _nextTime_ms) {
  std::deque<std::pair<double, Vector3d>> historyData = _historyData;
  if (historyData.size() == 1) {
    Vector3d nextData = historyData.at(historyData.size() - 1).second;
    return nextData;
  } else {
    Vector3d lastValidData = _historyData.at(historyData.size() - 1).second;

    // compute the position increment
    Vector3d positionIncrement;
    float totalWeight = 0;
    for (int unsigned i = 1; i < _historyData.size(); i++) {
      Vector3d currentData = _historyData.at(i).second;
      Vector3d lastData = _historyData.at(i - 1).second;

      float weight = 1;
      Vector3d increment =
          (currentData - lastData) * weight /
          (_historyData.at(i).first - _historyData.at(i - 1).first);

      positionIncrement = increment + positionIncrement;
      totalWeight += weight;
    }
    positionIncrement = positionIncrement / totalWeight;

    // lastValidData [Vector3d] + positionIncrement [Vector3d] * (
    // _nextTime_ms/1000 [s] - historyData.first [s])
    Vector3d nextData =
        lastValidData +
        positionIncrement * (_nextTime_ms / 1000 -
                             _historyData.at(historyData.size() - 1).first);
    return nextData;
  }
}


/*************************************
 *          PRIVATE METHODS          *
 ************************************/


Vector3d UAV::generateNormedNoiseVector()
{
    return Vector3d(-0.5 + ((double)rand() / RAND_MAX), -0.5 + ((double)rand() / RAND_MAX), -0.5 + ((double)rand() / RAND_MAX)).normed();
}

}
