#include "car.h"
#include "LIMoSim/mobility/car/following/idm.h"
#include "LIMoSim/mobility/car/strategic/randomdirection.h"
#include "LIMoSim/mobility/car/strategic/randomwaypoint.h"
#include "LIMoSim/mobility/car/strategic/samerandompath.h"
#include "LIMoSim/mobility/car/strategic/deterministicpath.h"
#include "LIMoSim/world/road/intersection.h"
#include "LIMoSim/world/vehiclemanager.h"
#include "LIMoSim/world/world.h"

namespace LIMoSim
{

Car::Car(const std::string &_id) :
    Vehicle(_id, "Car"),
    m_movementState(MOVEMENT_STATE::DRIVING_FREE),
    m_width_m(2.5),
    m_length_m(5),
    m_acceleration_mpss(0),
    m_speed_mps(0),
    m_maxSpeed_mps(250.0 / 3.6),
    p_world(World::getInstance()),
    m_following(new IDM(this)),
    //m_strategic(new RandomWaypoint(this))
    m_strategic(new RandomDirection(this))
{
    m_roadPosition.intersection = 0;
    if (VehicleManager::getInstance()->getDeterministicCarPath()) {
        m_strategic = new DeterministicPath(this);
    } else {
        m_strategic = new RandomDirection(this);
    }
}

Car::~Car()
{
    delete m_strategic;
    delete m_following;
}

/*************************************
 *            PUBLIC METHODS         *
 ************************************/

void Car::setRoadPosition(const RoadPosition &_position)
{
    m_roadPosition = _position;

    updatePosition();
}

void Car::setMovementState(int _state)
{
    m_movementState = _state;
}

void Car::setSpeed(double _speed_mps)
{
    m_speed_mps = _speed_mps;
}

void Car::setMaxSpeed(double _speed_mps)
{
    m_maxSpeed_mps = _speed_mps;
}

RoadPosition Car::getRoadPosition()
{
    return m_roadPosition;
}

RoadAwarenessEntry Car::getLeader()
{
    return m_leader;
}

int Car::getMovementState()
{
    return m_movementState;
}

double Car::getWidth()
{
    return m_width_m;
}

double Car::getLength()
{
    return m_length_m;
}

double Car::getAcceleration()
{
    return m_acceleration_mpss;
}

double Car::getSpeed()
{
    return m_speed_mps;
}

double Car::getMaxSpeed()
{
    return m_maxSpeed_mps;
}

StrategicModel* Car::getStrategicModel()
{
    return m_strategic;
}

/*************************************
 *          PROTECTED METHODS        *
 ************************************/

void Car::initialize()
{
    Vehicle::initialize();
    m_strategic->initialize();

    m_speed_mps = 0;
}

void Car::handleEvent(Event *_event)
{
    Vehicle::handleEvent(_event);
}

void Car::move(double _timeDelta_s)
{
    //
    m_leader = m_awareness.findLeader(this);
    updateAcceleration();
    updateTravelledDistance(_timeDelta_s);

    // intersection handling            // TODO: we only need to find the possible lanes once -> TurnIntent class
    updateIntersectionAwareness();
}

Vector3d Car::getWaypoint(){
    // Get current segment
    LaneSegment *segment = m_roadPosition.laneSegment;
    // Return Endnode of segment as appraoched waypoint
    return segment->getEndNode()->getPosition();
}

Vector3d Car::getPredictedPosition(double _timeDelta_s) {
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

Vector3d Car::predictWithTarget(Vector3d _currentData,
                                    int m_updateInterval_ms, Vector3d wp0) {
  Vector3d nextData = _currentData;

  // compute the next position
  Vector3d position = _currentData;
  Vector3d target = wp0;
  double m_maxSpeed_mpMs = getMaxSpeed()/1000;

  nextData =
      position + (target - position) / ((target - position).norm() *
                                        (m_updateInterval_ms)*m_maxSpeed_mpMs);

  return nextData;
}

Vector3d Car::predictWithHistory(
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
 *           PRIVATE METHODS         *
 ************************************/

void Car::updateAcceleration()
{
    // at first, it is assumed that the leader vehicle is the dominant factor
    bool hasLeader = m_leader.car;
    double leaderDistance_m = m_leader.distance_m;
    double leaderSpeedDelta_mps = m_leader.speedDelta_mps;
    if(!hasLeader)
    {
        leaderDistance_m = 1000;
        setMovementState(MOVEMENT_STATE::DRIVING_FREE);
    }
    else
        setMovementState(MOVEMENT_STATE::DRIVING_FOLLOWING);

    //
    if(m_roadPosition.intersection && m_roadPosition.intersection->getOutSize()>1 && m_roadPosition.distanceToIntersection_m<50 && m_roadPosition.distanceToIntersection_m<leaderDistance_m)
    {
        CarEntry entry = m_roadPosition.intersection->getCarEntry(this);

        IntersectionHandler handler;
        bool drive = handler.drive(this, entry.distance_m, entry.segment, m_roadPosition.turnIntent, m_roadPosition.intersection);

        if(!drive) //intersection distance is dominant
        {
            hasLeader = true;
            leaderDistance_m = m_roadPosition.distanceToIntersection_m - 10;    // TODO: the intersection distance should depend on the size of the intersection
            leaderSpeedDelta_mps = m_speed_mps;

            setMovementState(MOVEMENT_STATE::WAITING_VEHICLE);
            //std::cout << "waiting " << m_id << std::endl;
        }
    }

    // compute the acceleration
    double desiredSpeed_mps = computeDesiredSpeed();
    m_acceleration_mpss = m_following->computeAcceleration(m_speed_mps, desiredSpeed_mps, hasLeader, leaderSpeedDelta_mps, leaderDistance_m);
    m_speed_mps += m_acceleration_mpss;
    if(m_speed_mps<-1 && false)
    {

        std::cout << getId() << " escapes\t" << leaderDistance_m << "\t" << m_speed_mps << "\t" << leaderSpeedDelta_mps << "\t" << m_position.toString();
        std::cout << "\t" << desiredSpeed_mps;
        if(m_leader.car)
            std::cout << "\t" << m_leader.car->getId();
        std::cout << std::endl;
    }
}

void Car::updateTravelledDistance(double _timeDelta_s)
{
    double distance_m = m_speed_mps * _timeDelta_s;
    double remainingDistance_m = m_roadPosition.laneSegment->computeLength() - m_roadPosition.offset_m;

    // move forward on the lane or switch the segment
    if(distance_m < remainingDistance_m)
    {
        m_roadPosition.offset_m += distance_m;
        setRoadPosition(m_roadPosition);
    }
    else
        handleSegmentSwitch();
    m_roadPosition.laneSegment->updateVehicle(this);
}

void Car::updatePosition()
{
    // compute the cartesian position from the road position
    LaneSegment *segment = m_roadPosition.laneSegment;
    Vector3d dir = segment->computeDirection().normed();

    setPosition(segment->getStart() + dir * m_roadPosition.offset_m);
    m_orientation.z = dir.computePhi();
}

void Car::updateIntersectionAwareness()
{
    LaneSegment *ls = m_roadPosition.laneSegment;
    m_roadPosition.distanceToIntersection_m = ls->computeLength() - m_roadPosition.offset_m;
    Intersection *intersection = 0;
    RoadSegment *segment = 0;
    while(!intersection)
    {
        // intersection detecion
        intersection = p_world->getIntersection(ls->getEndNode());
        if(intersection)
        {
            if(m_roadPosition.path.size()>0 && intersection->getOutSegmentForGate(m_roadPosition.path.at(0)))
            {
                Gate *gate = m_roadPosition.path.at(0);
                RoadSegment *out = intersection->getOutSegmentForGate(gate);
                segment = ls->getSegment();

                m_roadPosition.turnIntent = IntersectionHandler::getTurnType(ls->getSegment(), out, intersection);
            }
        }
        else
        {
            ls = ls->getNext();
            if(!ls)
                std::cout << "INVALID LANE - Car::updateIntersectionAwareness" << std::endl;
            else
                m_roadPosition.distanceToIntersection_m += ls->computeLength();
        }
    }

    if(intersection)
    {
        if(m_roadPosition.intersection!=intersection)
        {
            if(m_roadPosition.intersection)
                m_roadPosition.intersection->deregisterCar(this);
        }
        intersection->updateCar(this, segment, m_roadPosition.distanceToIntersection_m);
    }

    m_roadPosition.intersection = intersection;
}


double Car::computeDesiredSpeed()
{
    // determine the desired speed with respect to the traffic rules and vehicle capabilities
    double desiredSpeed_mps = std::min(m_roadPosition.laneSegment->getSegment()->getRoad()->getMaxSpeed(), m_maxSpeed_mps);
    float intersectionDistance_m = m_roadPosition.distanceToIntersection_m;

    // traffic rules
    if(intersectionDistance_m<80 && m_roadPosition.turnIntent!=TURN_TYPE::STRAIGHT)
    {
        double speed_mps = (intersectionDistance_m/2)/3.6;
        if(speed_mps<10/3.6)
            speed_mps = 10/3.6;

        desiredSpeed_mps = std::min(desiredSpeed_mps, speed_mps);
    }

    return desiredSpeed_mps;
}

void Car::handleSegmentSwitch()
{
    LaneSegment *currentSegment = m_roadPosition.laneSegment;
    LaneSegment *nextSegment = currentSegment->getNext();
    Node *node = currentSegment->getEndNode();
    Intersection* intersection = p_world->getIntersection(node);
    Gate *gate = 0;
    if(intersection)
        gate = intersection->getInGate(currentSegment->getSegment());

    // call the node update handler for statistics etc.
    m_strategic->handleNodeReached(node);

    //
    if(gate)
        m_strategic->handleGateReached(gate, intersection, currentSegment);
    else if(nextSegment)
    {
        m_roadPosition.laneSegment = nextSegment;
        m_roadPosition.offset_m = 0;
        setRoadPosition(m_roadPosition);
    }

    // update the lane registration
    currentSegment->deregisterVehicle(this);
}

}
