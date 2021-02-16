#include "parrotaerialbasestation.h"

#include <iostream>
#include <qcommandlineparser.h>

#include <ns3/core-module.h>
#include <ns3/applications-module.h>
#include <ns3/mobility-module.h>

#include "LIMoSim/mobility/uav/reynolds/behavior_cohesionarea.h"
#include "LIMoSim/simulation/simulation.h"
#include "LIMoSim/world/world.h"
#include "LIMoSim/world/road/roadutils.h"
#include "LIMoSim/world/vehiclemanager.h"
#include "ui/data/uidatamanager.h"
#include "ui/uimanagerservice.h"
#include "standalone/qclidecorators.h"

#include "ns3/ns3setuphelpers.h"

namespace LIMoSim {
namespace NS3 {
namespace Examples {
namespace ParrotAerialBasestation {

using namespace Standalone;

void parseQCmdLine(uint &numUavs, uint &numCars) {
    QCommandLineParser parser;
    parser.setApplicationDescription("LIMoSim - ns3 - Parrot AerialBaseStation");
    parser.addHelpOption();
    parser.addVersionOption();
    qclidecorators::addGeneralOptions(parser);
    qclidecorators::addSimulationOptions(parser);
    QCommandLineOption uavCountOption(
                "uav",
                "Number of UAVs - max 10",
                "number");
    QCommandLineOption carCountOption(
                "car",
                "Number of cars",
                "number");
    parser.addOptions({uavCountOption, carCountOption});
    parser.process(QCoreApplication::arguments());
    bool ok = false;
    numUavs = parser.isSet(uavCountOption) ? parser.value(uavCountOption).toUInt(&ok) : 2;
    if (!ok) {
        std::cout<<"Scenarios::Standalone::scenarios::AerialBaseStation: setting uav count fallback value: " << 2 << std::endl;
        numUavs = 2;
    }

    numCars = parser.isSet(carCountOption) ? parser.value(carCountOption).toUInt(&ok) : 50;
    if (!ok) {
        std::cout<<"Scenarios::Standalone::scenarios::AerialBaseStation: setting car count fallback value: "<< 50 << std::endl;
        numCars = 50;
    }
}

void createLIMoSimVehicles(uint numCars, uint numUavs) {

    using namespace UI::Data;
    VehicleManager *vehicleManager = VehicleManager::getInstance();
    UIDataManager *uidm = UIDataManager::getInstance();

    RoadPosition roadPosition;
    for (uint8_t carIdx = 0; carIdx < numCars; carIdx++) {
        Car *car = vehicleManager->createCar("C" + std::to_string(carIdx));
        uidm->getVehicleData(car->getId())->getShape()->setColor("red");
        roadPosition = RoadUtils::randomValidRoadPosition();
        car->setRoadPosition(roadPosition);
        car->initialize();
    }

    std::vector<std::vector<uint>> divisionSchemas = {
        {1},
        {2},
        {2,1},
        {2,2},
        {3,2},
        {3,3},
        {3,2,2},
        {4,4},
        {5,4},
        {5,5}
    };
    std::vector<uint> schema = divisionSchemas.at(numUavs-1);
    size_t numRows = schema.size();
    uint schemaCheckSum = 0;
    for (uint i : schema) {
        schemaCheckSum += i;
    }
    if (schemaCheckSum != numUavs) {
        std::cerr << "Schema does not match uav count" << std::endl;
    }


    // Generate Cohesion Areas
    double xLength = World::getInstance()->getBoxMax().x - World::getInstance()->getBoxMin().x;
    double yLength = World::getInstance()->getBoxMax().y - World::getInstance()->getBoxMin().y;
    double yInc = -yLength / numRows;
    double startX = World::getInstance()->getBoxMin().x;
    double startY = World::getInstance()->getBoxMax().y;

    std::vector<AreaLimits> areas;
    for (uint j = 0; j < numRows; j++) {
        double xInc = xLength / schema.at(j);
        for (uint i = 0; i < schema.at(j); i ++) {
            // create rectangular area
            areas.push_back({startX, startX + xInc, startY, startY + yInc});
            startX += xInc;
        }
        startX = World::getInstance()->getBoxMin().x;
        startY += yInc;
    }

    for (uint i = 0; i < numUavs; i++) {
        UAV *uav = vehicleManager->createUAV("U"+std::to_string(i));
        uidm->getVehicleData(uav->getId())->getShape()->setColor("yellow");
        auto area = areas.at(i);
        uav->setPosition(Vector3d((area.at(0) + area.at(1))/2,
                                  (area.at(2) + area.at(3))/2,
                                  30));
        uav->setBehavior(new Behavior_CohesionArea(areas.at(i)));
    }
}

void connectLIMoSimVehicleType(ns3::NodeContainer &vehicleNodes, std::string type) {
    ns3::MobilityHelper mobilityHelper;
    auto limoVehicles = VehicleManager::getInstance()->getVehicles(type);
    uint vehicleIdx = 0;
    for (auto vehicleEntry : limoVehicles) {
        mobilityHelper.SetMobilityModel("LIMoSim::NS3::LIMoSimMobility",
                                              "VehicleId", ns3::StringValue(vehicleEntry.first));
        mobilityHelper.Install(vehicleNodes.Get(vehicleIdx));
        vehicleIdx++;
    }
}

void setup(uint8_t _run)
{
    using namespace ns3;
    std::string scenarioName = "ParrotAerialBaseStation";


    uint numUavs, numCars;
    parseQCmdLine(numUavs, numCars);

    std::cout << "starting scenario " << scenarioName
                  << " " << std::to_string(numUavs) << " uavs"
                  << " " << std::to_string(numCars) << " cars"
                  << " run "
                  << std::to_string(_run) << std::endl;

    Simulation::getInstance()->setName(scenarioName + "_" +
                                       std::to_string(numUavs) + "u_"+
                                       std::to_string(numCars) + "c_")
            ->setRunCount(_run);

    NodeContainer uavNodes, carNodes;
    uavNodes.Create(numUavs);
    carNodes.Create(numCars);

    // Connect LIMoSim Cars and UAVs to ns3
    createLIMoSimVehicles(numCars, numUavs);
    connectLIMoSimVehicleType(carNodes, "Car");
    connectLIMoSimVehicleType(uavNodes, "UAV");


    Simulator::Stop (Seconds(300));
    ui::UiManagerService::getInstance()->setUiDataInitialized(true);
    LIMoSim::Simulation::getInstance()->run();

}



} // namespace ParrotAerialBasestation
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim
