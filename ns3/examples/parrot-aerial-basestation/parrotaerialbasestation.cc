#include "parrotaerialbasestation.h"

#include <iostream>
#include <qcommandlineparser.h>

#include <ns3/aodv-module.h>
#include <ns3/applications-module.h>
#include <ns3/core-module.h>
#include <ns3/dsdv-module.h>
#include <ns3/dsr-module.h>
#include <ns3/flow-monitor-helper.h>
#include <ns3/internet-module.h>
#include <ns3/mobility-module.h>
#include <ns3/network-module.h>
#include <ns3/olsr-module.h>
#include <ns3/parrot-module.h>
#include <ns3/yans-wifi-helper.h>

#include "LIMoSim/mobility/uav/reynolds/behavior_cohesionarea.h"
#include "LIMoSim/simulation/simulation.h"
#include "LIMoSim/world/road/roadutils.h"
#include "LIMoSim/world/vehiclemanager.h"
#include "LIMoSim/world/world.h"
#include "standalone/qclidecorators.h"
#include "ui/data/uidatamanager.h"
#include "ui/uimanagerservice.h"

#include "ns3/ns3setuphelpers.h"

#define PORT 9
namespace LIMoSim {
namespace NS3 {
namespace Examples {
namespace ParrotAerialBasestation {

using namespace Standalone;

void parseQCmdLine(uint &numUavs, uint &numCars, std::string &scenarioName,
                   int &protocol, int &seed) {
  QCommandLineParser parser;
  parser.setApplicationDescription("LIMoSim - ns3 - Parrot AerialBaseStation");
  parser.addHelpOption();
  parser.addVersionOption();
  qclidecorators::addGeneralOptions(parser);
  qclidecorators::addSimulationOptions(parser);
  QCommandLineOption uavCountOption("uav", "Number of UAVs - max 10", "number");
  QCommandLineOption carCountOption("car", "Number of cars", "number");
  QCommandLineOption scenarioNameOption("CSVfileName", "CSVfileName", "string");
  QCommandLineOption protocolOption("protocol", "Protocol", "number");
  QCommandLineOption seedOption("seed", "Seed", "number");
  parser.addOptions({uavCountOption, carCountOption, scenarioNameOption,
                     protocolOption, seedOption});
  parser.process(QCoreApplication::arguments());
  bool ok = false;
  if (parser.isSet(uavCountOption)) {
    std::cout << "uavCountOption set" << std::endl;
  }
  numUavs = parser.isSet(uavCountOption)
                ? parser.value(uavCountOption).toUInt(&ok)
                : 2;
  if (!ok) {
    std::cout << "Scenarios::Standalone::scenarios::AerialBaseStation: setting "
                 "uav count fallback value: "
              << 2 << std::endl;
    numUavs = 2;
  }

  numCars = parser.isSet(carCountOption)
                ? parser.value(carCountOption).toUInt(&ok)
                : 50;
  if (!ok) {
    std::cout << "Scenarios::Standalone::scenarios::AerialBaseStation: setting "
                 "car count fallback value: "
              << 50 << std::endl;
    numCars = 50;
  }

  scenarioName = parser.isSet(scenarioNameOption)
                     ? parser.value(scenarioNameOption).toStdString()
                     : "";
  ok=true;
  if (!ok) {
    std::cout << "Scenarios::Standalone::scenarios::AerialBaseStation: setting "
                 "scenarioName fallback value: "
              << "\"\"" << std::endl;
    scenarioName = "";
  }

  protocol = parser.isSet(protocolOption)
                 ? parser.value(protocolOption).toInt(&ok)
                 : 4;
  if (!ok) {
    std::cout << "Scenarios::Standalone::scenarios::AerialBaseStation: setting "
                 "protocol fallback value: "
              << 4 << std::endl;
    protocol = 4;
  }

  seed = parser.isSet(seedOption) ? parser.value(seedOption).toInt(&ok) : 1;
  if (!ok) {
    std::cout << "Scenarios::Standalone::scenarios::AerialBaseStation: setting "
                 "seed fallback value: "
              << 1 << std::endl;
    seed = 1;
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
      {1},    {2},       {2, 1}, {2, 2}, {3, 2},
      {3, 3}, {3, 2, 2}, {4, 4}, {5, 4}, {5, 5}};
  std::vector<uint> schema = divisionSchemas.at(numUavs - 1);
  size_t numRows = schema.size();
  uint schemaCheckSum = 0;
  for (uint i : schema) {
    schemaCheckSum += i;
  }
  if (schemaCheckSum != numUavs) {
    std::cerr << "Schema does not match uav count" << std::endl;
  }

  // Generate Cohesion Areas
  double xLength =
      World::getInstance()->getBoxMax().x - World::getInstance()->getBoxMin().x;
  double yLength =
      World::getInstance()->getBoxMax().y - World::getInstance()->getBoxMin().y;
  double yInc = -yLength / numRows;
  double startX = World::getInstance()->getBoxMin().x;
  double startY = World::getInstance()->getBoxMax().y;

  std::vector<AreaLimits> areas;
  for (uint j = 0; j < numRows; j++) {
    double xInc = xLength / schema.at(j);
    for (uint i = 0; i < schema.at(j); i++) {
      // create rectangular area
      areas.push_back({startX, startX + xInc, startY, startY + yInc});
      startX += xInc;
    }
    startX = World::getInstance()->getBoxMin().x;
    startY += yInc;
  }

  for (uint i = 0; i < numUavs; i++) {
    UAV *uav = vehicleManager->createUAV("U" + std::to_string(i));
    uidm->getVehicleData(uav->getId())->getShape()->setColor("yellow");
    auto area = areas.at(i);
    uav->setPosition(Vector3d((area.at(0) + area.at(1)) / 2,
                              (area.at(2) + area.at(3)) / 2, 30));
    uav->setBehavior(new Behavior_CohesionArea(areas.at(i)));
  }
}

void connectLIMoSimVehicleType(ns3::NodeContainer &vehicleNodes,
                               std::string type) {
  ns3::MobilityHelper mobilityHelper;
  auto limoVehicles = VehicleManager::getInstance()->getVehicles(type);
  uint vehicleIdx = 0;
  for (auto vehicleEntry : limoVehicles) {
    mobilityHelper.SetMobilityModel("LIMoSim::NS3::LIMoSimMobility",
                                    "VehicleId",
                                    ns3::StringValue(vehicleEntry.first));
    mobilityHelper.Install(vehicleNodes.Get(vehicleIdx));
    vehicleIdx++;
  }
}

struct PARRoTSettings {
  double ChirpInterval = 0.5;
  double NeighborReliabilityTimeout = 2.0;
  int MaxHops = 32;
  double LearningRate = 0.5;
  double DiscountFactor = 0.7;
  std::string CombinationMethod = "M";
  int HistorySize = 5;
  std::string PredictionMethod = "limosim";
  double RangeOffset = -12.5;
};

void setup(uint8_t _run) {
  using namespace ns3;

  int m_nSinks = 1;
  double m_txp = 20.0;
  std::string m_CSVfileName;
  int m_protocol;
  int m_seed;

  uint numUavs, numCars;
  parseQCmdLine(numUavs, numCars, m_CSVfileName, m_protocol, m_seed);
  std::string scenarioName = m_CSVfileName;
  RngSeedManager::SetSeed(m_seed);

  std::cout << "starting scenario " << scenarioName << " "
            << std::to_string(numUavs) << " uavs"
            << " " << std::to_string(numCars) << " cars"
            << " run " << std::to_string(_run) << std::endl;

  Simulation::getInstance()
      ->setName(scenarioName + "_" + std::to_string(numUavs) + "u_" +
                std::to_string(numCars) + "c_")
      ->setRunCount(_run);

  NodeContainer uavNodes, carNodes;
  uavNodes.Create(numUavs);
  carNodes.Create(numCars);

  // Connect LIMoSim Cars and UAVs to ns3
  createLIMoSimVehicles(numCars, numUavs);
  connectLIMoSimVehicleType(carNodes, "Car");
  connectLIMoSimVehicleType(uavNodes, "UAV");
  std::cout << "\n\n"
            << "Finished Parsing\nSeed:\t" << m_seed << " Name:\t"
            << m_CSVfileName << " Protocol:\t" << m_protocol << "\n";
  /*
   *
   * Simulation part
   *
   *
   */
  double TotalTime = 900.0;
  std::string rate("2Mbps");
  std::string tr_name = m_CSVfileName;
  std::string m_protocolName = "protocol";
  Config::SetDefault("ns3::OnOffApplication::PacketSize", StringValue("1460"));
  Config::SetDefault("ns3::OnOffApplication::DataRate", StringValue(rate));
  // setting up wifi phy and channel using helpers
  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211g);

  YansWifiPhyHelper wifiPhy;
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss("ns3::EtaFriisPropagationLossModel");
  wifiPhy.SetChannel(wifiChannel.Create());

  // Add a mac and disable rate control
  WifiMacHelper wifiMac;

  wifi.SetRemoteStationManager("ns3::IdealWifiManager");

  wifiPhy.Set("TxPowerStart", DoubleValue(m_txp));
  wifiPhy.Set("TxPowerEnd", DoubleValue(m_txp));

  wifiPhy.Set("RxSensitivity", DoubleValue(-85.0));
  wifiPhy.Set("CcaEdThreshold", DoubleValue(-85.0));

  // No tx/rx gains are used, so make sure, that no noise is added
  wifiPhy.Set("RxNoiseFigure", DoubleValue(0.0));
  wifiPhy.SetPreambleDetectionModel("ns3::ThresholdPreambleDetectionModel",
                                    "MinimumRssi", DoubleValue(-85.0));
  wifiPhy.SetErrorRateModel("ns3::NistErrorRateModel");

  wifiMac.SetType("ns3::AdhocWifiMac");

  // Create merged NodeContainer
  NodeContainer adhocNodes(uavNodes, carNodes);
  NetDeviceContainer adhocDevices = wifi.Install(wifiPhy, wifiMac, adhocNodes);

  AodvHelper aodv;
  OlsrHelper olsr;
  DsdvHelper dsdv;
  DsrHelper dsr;
  PARRoTHelper parrot;
  PARRoTSettings m_parrotSettings;
  DsrMainHelper dsrMain;
  Ipv4ListRoutingHelper list;
  InternetStackHelper internet;
 std::cout << "WiFi ok" << std::endl;
  switch (m_protocol) {
  case 1:
    // Configure OLSR according to PARRoT v1 Paper
    olsr.Set("HelloInterval", TimeValue(Seconds(0.5)));
    olsr.Set("TcInterval", TimeValue(Seconds(1.0)));

    // These two parameters were not further specified, but, GetTypeId() says,
    // that they're usually equal to TcInterval
    olsr.Set("MidInterval", TimeValue(Seconds(1.0)));
    olsr.Set("HnaInterval", TimeValue(Seconds(1.0)));

    list.Add(olsr, 100);
    m_protocolName = "OLSR";
    break;
  case 2:
    // Better don't touch?
    list.Add(aodv, 100);
    m_protocolName = "AODV";
    break;
  case 3:
    dsdv.Set("PeriodicUpdateInterval", TimeValue(Seconds(0.5)));
    dsdv.Set("EnableBuffering", BooleanValue(false));
    list.Add(dsdv, 100);
    m_protocolName = "DSDV";
    break;
  case 4:
    parrot.Set("ChirpInterval",
               TimeValue(Seconds(m_parrotSettings.ChirpInterval)));
    parrot.Set("NeighborReliabilityTimeout",
               TimeValue(Seconds(m_parrotSettings.NeighborReliabilityTimeout)));
    parrot.Set("MaxHops", IntegerValue(m_parrotSettings.MaxHops));
    parrot.Set("LearningRate", DoubleValue(m_parrotSettings.LearningRate));
    parrot.Set("DiscountFactor", DoubleValue(m_parrotSettings.DiscountFactor));
    parrot.Set("CombinationMethod",
               StringValue(m_parrotSettings.CombinationMethod));
    parrot.Set("HistorySize", IntegerValue(m_parrotSettings.HistorySize));
    parrot.Set("PredictionMethod",
               StringValue(m_parrotSettings.PredictionMethod));
    parrot.Set("RangeOffset", DoubleValue(m_parrotSettings.RangeOffset));
    std::cout << "Prediction method: " << m_parrotSettings.PredictionMethod << "\n";
    list.Add(parrot, 100);
    m_protocolName = "PARRoT";
    break;
  case 5:
    m_protocolName = "DSR";
    break;
  default:
    NS_FATAL_ERROR("No such protocol:" << m_protocol);
  }

  if (m_protocol < 5) {
    internet.SetRoutingHelper(list);
    internet.Install(adhocNodes);
  } else if (m_protocol == 5) {
    internet.Install(adhocNodes);
    dsrMain.Install(dsr, adhocNodes);
  }
  std::cout << "Protocol ok" << std::endl;
  NS_LOG_INFO("assigning ip address");

  Ipv4AddressHelper addressAdhoc;
  addressAdhoc.SetBase("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer adhocInterfaces;
  adhocInterfaces = addressAdhoc.Assign(adhocDevices);

  OnOffHelper onoff1("ns3::UdpSocketFactory", Address());
  onoff1.SetAttribute("OnTime",
                      StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
  onoff1.SetAttribute("OffTime",
                      StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
  /*
   *
   *
   *
   *
   */

//   for (int i = 0; i < m_nSinks; i++) {
//     TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
//     Ptr<Socket> sink = Socket::CreateSocket(adhocNodes.Get(i), tid);
//     InetSocketAddress local =
//         InetSocketAddress(adhocInterfaces.GetAddress(i), PORT);
//     sink->Bind(local);
//     //   sink->SetRecvCallback (MakeCallback (&ReceivePacket, this));
//     AddressValue remoteAddress(
//         InetSocketAddress(adhocInterfaces.GetAddress(i), PORT));
//     onoff1.SetAttribute("Remote", remoteAddress);

//     Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable>();
//     ApplicationContainer temp = onoff1.Install(adhocNodes.Get(i + m_nSinks));
//     temp.Start(Seconds(5.0));
//     temp.Stop(Seconds(TotalTime));
//   }

//   Ptr<FlowMonitor> flowmon;
//   FlowMonitorHelper flowmonHelper;
//   flowmon = flowmonHelper.InstallAll();

  Simulator::Stop(Seconds(TotalTime));
  ui::UiManagerService::getInstance()->setUiDataInitialized(true);
  LIMoSim::Simulation::getInstance()->run();
//   flowmon->SerializeToXmlFile((tr_name + ".flowmon").c_str(), false, false);

//   Simulator::Destroy();
}

} // namespace ParrotAerialBasestation
} // namespace Examples
} // namespace NS3
} // namespace LIMoSim
