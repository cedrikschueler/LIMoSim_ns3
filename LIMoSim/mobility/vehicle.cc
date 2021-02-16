#include "vehicle.h"

#include "LIMoSim/world/localvehiclemanager.h"
#include "LIMoSim/world/vehiclemanager.h"

#define WP_REACHED_RANGE 10.0

namespace LIMoSim {

Vehicle::Vehicle(const std::string &_id, const std::string &_type)
    : EventHandler(), m_id(_id), m_type(_type), m_lastUpdate_s(0),
      m_updateTimer(nullptr), m_updateInterval_s(0.01),
      m_mobilityModel(nullptr), moveSpeedUp(1), m_positionHistorySize(15)
{
    if (s_enableMobilityExport)
        m_mobilityDataExporter = new MobilityDataExporter(this, 0.1);
}

Vehicle::~Vehicle() {
  delete m_mobilityDataExporter;
  if (m_updateTimer) {
    deleteEvent(m_updateTimer);
  }
}

void Vehicle::setAcceleration(const Vector3d &_acceleration) {
  m_acceleration_mps2 = _acceleration;
}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void Vehicle::setPosition(const Vector3d &_position) {
  m_position = _position;
  auto lvm = getLocalVehicleManager();
  if (lvm) {
    lvm->updateMobilityData();
  }
}

void Vehicle::setOrientation(const Orientation3d &_orientation) {
  m_orientation = _orientation;
}

void Vehicle::setVelocity(const Vector3d &_velocity) {
  m_velocity_mps = _velocity;
  auto lvm = getLocalVehicleManager();
  if (lvm) {
    lvm->updateMobilityData();
  }
}

void Vehicle::setOrientationVelocity(
    const Orientation3d &_orientationVelocity) {
  m_orientationVelocity_radps = _orientationVelocity;
}

std::string Vehicle::getId() { return m_id; }

std::string Vehicle::getType() { return m_type; }

Vector3d Vehicle::getAcceleration() { return m_acceleration_mps2; }

Vector3d Vehicle::getPosition() { return m_position; }

Vector3d Vehicle::getPredictedPosition(double _timeDelta_s) {
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
  } else {
    return Vector3d(0, 0, 0);
  }
}

Vector3d Vehicle::predictWithTarget(Vector3d _currentData,
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

Vector3d Vehicle::predictWithHistory(
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

std::deque<std::pair<double, Vector3d>> Vehicle::getPositionHistory() {
  return m_positionHistory;
}

Orientation3d Vehicle::getOrientation() { return m_orientation; }

Vector3d Vehicle::getVelocity() { return m_velocity_mps; }

Orientation3d Vehicle::getOrientationVelocity() {
  return m_orientationVelocity_radps;
}

MobilityModel *Vehicle::getModel() { return m_mobilityModel; }

LocalVehicleManager *Vehicle::getLocalVehicleManager() {
  return VehicleManager::getInstance()->getLocalVehicleManager(this->getId());
}

void Vehicle::initialize() {
  m_updateTimer = new Event(m_updateInterval_s, this, "Move");
  scheduleEvent(m_updateTimer);
  if (s_enableMobilityExport)
    m_mobilityDataExporter->start();
}

void Vehicle::handleEvent(Event *_event) {
  if (_event->getInfo() == m_updateTimer->getInfo()) {
    handleMoveEvent(_event);
  } else {
    delete _event;
  }
}

void Vehicle::handleMoveEvent(Event *_event) {
  double timeDelta_s = _event->getTimestamp() - m_lastUpdate_s;
  move(timeDelta_s * moveSpeedUp);
  m_lastUpdate_s = _event->getTimestamp();

  m_positionHistory.push_back(
      std::make_pair(m_lastUpdate_s, this->getPosition()));
  if (m_positionHistory.size() > m_positionHistorySize)
    m_positionHistory.pop_front();
  //
  scheduleEvent(_event, m_updateInterval_s);
}

void Vehicle::setModel(MobilityModel *_mobilityModel) {
  if (m_mobilityModel) {
    delete m_mobilityModel;
  }
  m_mobilityModel = _mobilityModel;
}

} // namespace LIMoSim
