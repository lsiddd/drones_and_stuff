#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility> // std::pair
#include <vector>

#include "ns3/applications-module.h"
#include "ns3/config-store.h"
#include "ns3/core-module.h"
#include "ns3/evalvid-client-server-helper.h"
#include "ns3/evalvid-client.h"
#include "ns3/evalvid-server.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/stats-module.h"
#include <ns3/buildings-helper.h>

#include <boost/algorithm/string/classification.hpp> // Include boost::for is_any_of
#include <boost/algorithm/string/split.hpp>          // Include for boost::split

#define LOG(x) std::cout << x << std::endl
#define wait std::cin.get()

using namespace ns3;

std::default_random_engine generator;

NS_LOG_COMPONENT_DEFINE("LteEvalvid");

const float PI = 3.14159265; // pi
void migrate(Ptr<Node>, Ptr<Node>, Ipv4Address, Ipv4Address);
int getCellId(int nodeId);

// COnfigurable parameters
string algorithm = "mosca";

// scenario variables
uint16_t numPeds = 1;

uint16_t numEnbs = 10;
uint16_t numEdgeNodes = numEnbs;
uint16_t numFogNodes = 10;
uint16_t numMistNodes = 20;
uint16_t numCloudNodes = 1;

std::vector<double> ues_sinr;
std::ofstream ues_sinr_file;
std::vector<double> time_to_centroid;
std::ofstream time_to_centroid_file;
std::ofstream ue_positions_log;
uint32_t active_drones = 0;
std::string clustering_algoritm = "kmeans";
bool disableDl = false;
bool disableUl = false;
bool enablePrediction = true;
bool verbose = false;
bool enableHandover = false;
std::string ns3_dir;

bool useCa = false;
const uint32_t numUAVs = 3;
const uint32_t numUes = 4;
const uint32_t numStaticCells = 10;
// uint32_t numCars = 10;
uint32_t seedValue = 10000;
uint32_t SimTime = 30;
int eNodeBTxPower = 46;
const uint32_t numBSs = numUAVs + numStaticCells;
uint16_t node_remote = 1; // HOST_REMOTO

/* connection management structures */
int connections[numBSs][numUes]{{0}};
int neighbors[numBSs][numUes]{{0}};
std::map<int, std::map<int, int>> rnti_cells;

// mapping nodelist ids to actual imsis
std::map<int, int> path_imsi;

// node containers as global objects to make mobility management possible
NodeContainer UAVNodes;
NodeContainer ueNodes;
NodeContainer BSNodes;
// todo: test servers in the BS nodes
NodeContainer ServerNodes;

NetDeviceContainer enbDevs;
NetDeviceContainer ueDevs;


// migration variables
// server characteristics
// the first index is the metric: lat, bw, and cost
// second index is the type of server: mist, edge, fog, cloud
int serverReqs[4][3] = {{1, 1, 4}, {4, 2, 3}, {10, 10, 2}, {100, 100, 1}};

// applications class 1, 2, 3, and 4
// latency in ms and bw in mbps, and prioritary
int applicationReqs[3][3] = {{1, 10, 1}, {10, 100, 1}, {1000, 1, 0}};
uint16_t applicationType = 0;

//-----VARIABLES THAT DEPEND ON THE NUMBER OF SERVERS----
// The resources variable tells which server has one or
// more of the recources needed in this simulation
// the resources are:
vector<uint16_t> resources;
int initialEdgeResources = 5;
int initialFogResources = 10;

// type of cell position allocation
bool rowTopology = false;
bool randomCellAlloc = true;

// ----------VARIABLES TO CALCULATE METRICS-----------
std::vector<int> latency;
std::vector<int> cost;


Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();

std::string exec(std::string cmd) {
  std::array<char, 128> buffer;
  std::string result;
  std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
  if (!pipe)
    throw std::runtime_error("popen() failed!");
  while (!feof(pipe.get())) {
    if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
      result += buffer.data();
  }
  return result;
}

// populate pairing from nodeid to imsi
int populate_path_imsi(std::string path, int imsi) {
  int nodeid;

  std::vector<std::string> split_path;
  boost::split(split_path, path, boost::is_any_of("/"));
  nodeid = stoi(split_path[2]);

  if (imsi != -1)
    path_imsi[nodeid] = imsi;

  return nodeid;
}

int get_imsi(int cellId, int rnti) {
  return rnti_cells[cellId][rnti] == 0 ? -1 : rnti_cells[cellId][rnti];
}

void print_meas() {
  for (uint32_t i = 0; i < numBSs; i++) {
    for (uint32_t j = 0; j < numUes; j++) {
      std::cout << connections[i][j] << "\t";
    }
    std::cout << std::endl;
  }
  Simulator::Schedule(Seconds(1), &print_meas);
}

int getServingcell(int imsi) {
  int servingCell = 0;
  for (uint32_t i = 0; i < numBSs; i++) {
    if (connections[i][imsi - 1] != 0) {
      servingCell = i;
    }
  }
  return servingCell;
}

void handoverManager() {
  // random handover
  // get serving cell of user

  for (uint32_t i = 0; i < numUes; i++) {
    int imsi = i + 1;
    uint32_t servingCell = getServingcell(imsi);
    int rsrp = std::numeric_limits<int>::lowest();
    int strongestNeighborCell = 0;

    for (uint32_t cell = 0; cell < numBSs; cell++) {
      if (neighbors[cell][i] > rsrp && cell != servingCell) {
        rsrp = neighbors[cell][i];
        strongestNeighborCell = cell;
      }
    }
    // Randomly handover to next strongest cell
    if (rsrp > neighbors[servingCell][imsi - 1]) {
      lteHelper->HandoverRequest(Simulator::Now(), ueDevs.Get(i),
                                 enbDevs.Get(servingCell),
                                 enbDevs.Get(strongestNeighborCell));
    }
    // LOG("Strongest cell for user " << i << " is " << strongestNeighborCell);
  }
}

void ReportUeMeasurementsCallback(std::string path, uint16_t rnti,
                                  uint16_t cellId, double rsrp, double rsrq,
                                  bool servingCell, uint8_t componentCarrierId)

{
  int imsi = get_imsi(cellId, rnti);

  int node_id = populate_path_imsi(path, imsi);

  if (verbose) {
    LOG("Simulation time: " << Simulator::Now().GetSeconds());
    LOG(path);
    LOG("rnti " << rnti);
    LOG("cellid " << cellId);
    LOG("rsrp " << rsrp);
    LOG("rsrq " << rsrq);
    LOG("imsi " << imsi);
    LOG("path imsi " << path_imsi[node_id]);
    LOG("serving cell " << servingCell);
    LOG("cc id " << (int)componentCarrierId);
    LOG("\n");

    if (path_imsi[node_id] != 0) {
      neighbors[cellId - 1][path_imsi[node_id] - 1] = rsrp;
    }
  }
  // call handover manager upon receiving new measurements
  // todo: add some fail checks for signal levels
  handoverManager();
}

void RecvMeasurementReportCallback(std::string path, uint64_t imsi,
                                   uint16_t cellId, uint16_t rnti,
                                   LteRrcSap::MeasurementReport meas) {
  if (verbose) {
    LOG("Simulation time: " << Simulator::Now().GetSeconds());
    LOG(path);
    LOG(imsi);
    LOG(cellId);
    LOG(rnti);
    LOG((int)meas.measResults.measId);
    LOG("\n");
  }
}

// todo: implement a random handover as a proof of concept

Ptr<ListPositionAllocator>
positionAllocator(Ptr<ListPositionAllocator> HpnPosition) {

  std::uniform_int_distribution<int> distribution(0, 1000);

  std::cout << "allocationg cells positions\n";

  // add cells in a unifor distribution on the scenario
  // void areas can be adjusted with the number of cells and trasmission power
  for (uint32_t i = 0; i < numStaticCells; i++) {
    HpnPosition->Add(
        Vector3D(distribution(generator), distribution(generator), 45));
  }

  return HpnPosition;
}

void NotifyConnectionEstablishedUe(std::string context, uint64_t imsi,
                                   uint16_t cellid, uint16_t rnti) {
  LOG(Simulator::Now().GetSeconds()
      << " " << context << " UE IMSI " << imsi << ": connected to CellId "
      << cellid << " with RNTI " << rnti << "\n");
  for (uint32_t i = 0; i < numBSs; ++i) {
    connections[i][imsi - 1] = 0;
  }
  connections[cellid - 1][imsi - 1] = rnti;
  rnti_cells[cellid][rnti] = imsi;
}

void NotifyHandoverStartUe(std::string context, uint64_t imsi, uint16_t cellId,
                           uint16_t rnti, uint16_t targetCellId) {
  std::cout << Simulator::Now().GetSeconds() << " " << context << " UE IMSI "
            << imsi << ": previously connected to CellId " << cellId
            << " with RNTI " << rnti << ", doing handover to CellId "
            << targetCellId << std::endl;
}
void NotifyHandoverEndOkUe(std::string context, uint64_t imsi, uint16_t cellId,
                           uint16_t rnti) {
  std::cout << Simulator::Now().GetSeconds() << " " << context << " UE IMSI "
            << imsi << ": successful handover to CellId " << cellId

            << " with RNTI " << rnti << std::endl;
}
void NotifyHandoverStartEnb(std::string context, uint64_t imsi, uint16_t cellId,
                            uint16_t rnti, uint16_t targetCellId) {
  std::cout << Simulator::Now().GetSeconds() << " " << context << " eNB CellId "
            << cellId << ": start handover of UE with IMSI " << imsi << " RNTI "
            << rnti << " to CellId " << targetCellId << std::endl;
}
void NotifyHandoverEndOkEnb(std::string context, uint64_t imsi, uint16_t cellId,
                            uint16_t rnti) {
  std::cout << Simulator::Now().GetSeconds() << " " << context << " eNB CellId "
            << cellId << ": completed handover of UE with IMSI " << imsi
            << " RNTI " << rnti << std::endl;
}

void ns3::PhyStatsCalculator::ReportUeSinr(uint16_t cellId, uint64_t imsi,
                                           uint16_t rnti, double sinrLinear,
                                           uint8_t componentCarrierId) {
  double sinrdB = 10 * log(sinrLinear);
  ues_sinr[imsi - 1] = sinrdB;
}

void CourseChange(std::string context, Ptr<const MobilityModel> model) {
  static std::unordered_map<const MobilityModel *, double> start_times;
  double now = Simulator::Now().GetSeconds();
  double start_time;
  if (start_times.count(PeekPointer(model)) > 0) { // check if key exists
    start_time = start_times[PeekPointer(model)];
    if (start_time >= 0) { // check if node started movement
      if (model->GetVelocity().GetLength() == 0) { // Drone stopped?
        time_to_centroid.push_back(now - start_time);
        start_times[PeekPointer(model)] = -1;
        LOG("drone arrived after " << now - start_time << " seconds");
      }
    } else {
      if (model->GetVelocity().GetLength() > 0) { // Drone moving?
        start_times[PeekPointer(model)] = now;
      }
    }
  } else {
    if (model->GetVelocity().GetLength() > 0) { // Drone moving?
      start_times[PeekPointer(model)] = now;
      active_drones++; // New drone moving;
    }
  }
}

void write_metrics() {
  std::stringstream sinrdata;
  unsigned int qtyUEs = ues_sinr.size();
  unsigned int qtyUEsCovered = 0;
  float coverageRatio = 0;
  for (unsigned int id = 0; id < qtyUEs; ++id) {
    if (ues_sinr[id] >= 3)
      qtyUEsCovered++;
    sinrdata << "Id: " << id << ", SINR: " << ues_sinr[id] << "dB" << std::endl;
  }
  coverageRatio = (float)qtyUEsCovered / qtyUEs;
  ues_sinr_file << "Coverage ratio: " << coverageRatio * 100 << "%"
                << std::endl;
  ues_sinr_file << sinrdata.str();

  std::stringstream timedata;
  double total_time = 0;
  double mean_time;
  for (auto time : time_to_centroid) {
    total_time += time;
    timedata << time << std::endl;
  }
  mean_time = total_time / active_drones;
  time_to_centroid_file << "Mean time to centroid: " << mean_time << std::endl;
  time_to_centroid_file << timedata.str();
}

void print_position(NodeContainer ueNodes) {
  std::cout << '\n';
  for (uint32_t j = 0; j < ueNodes.GetN(); ++j) {
    Ptr<MobilityModel> mob = ueNodes.Get(j)->GetObject<MobilityModel>();
    Vector pos = mob->GetPosition();
    Vector3D speed = mob->GetVelocity();
    std::cout << " Node " << j << " | POS: (x=" << pos.x << ", y=" << pos.y
              << ") | Speed(" << speed.x << ", " << speed.y << ")" << std::endl;
  }
}

void save_user_positions(NodeContainer nodes) {
  std::ofstream pos_file("positions.txt");
  for (uint32_t i = 0; i < nodes.GetN(); ++i) {
    Ptr<MobilityModel> mob = nodes.Get(i)->GetObject<MobilityModel>();
    Vector pos = mob->GetPosition();
    pos_file << pos.x << " " << pos.y << " " << pos.z << "\n";
  }
  pos_file.close();
}

void save_user_positions(NodeContainer nodes,
                         std::vector<Vector2D> predicted_coords) {
  Vector2D coords;
  std::ofstream pos_file("positions.txt");
  for (uint32_t i = 0; i < nodes.GetN(); ++i) {
    Ptr<MobilityModel> mob = nodes.Get(i)->GetObject<MobilityModel>();
    Vector pos = mob->GetPosition();
    coords = predicted_coords[i];
    pos_file << coords.x << " " << coords.y << " " << pos.z << "\n";
  }
  pos_file.close();
}

void log_ue_positions(NodeContainer UEs, std::ofstream *ue_positions_log) {
  double now = Simulator::Now().GetSeconds();
  for (NodeContainer::Iterator i = UEs.Begin(); i != UEs.End(); ++i) {
    Ptr<Node> UE = *i;
    Ptr<MobilityModel> UEposition = UE->GetObject<MobilityModel>();
    Vector pos = UEposition->GetPosition();
    *ue_positions_log << now << "," << UE->GetId() << "," << pos.x << ","
                      << pos.y << "\n";
  }
  ue_positions_log->flush();
  Simulator::Schedule(Seconds(1), &log_ue_positions, UEs, ue_positions_log);
}

std::vector<Vector2D> do_predictions() {
  static double last_time;
  static std::vector<Vector2D> predicted_coords;
  std::string prediction;
  std::vector<std::string> split;
  Vector2D coordinate;

  if (last_time == Simulator::Now().GetSeconds())
    return predicted_coords; // return cached prediction

  predicted_coords.clear();
  last_time = Simulator::Now().GetSeconds();
  prediction = exec(std::string("python3 ") + ns3_dir +
                    std::string("/prediction.py 2>>prediction_errors.txt"));
  boost::split(split, prediction, boost::is_any_of(" "),
               boost::token_compress_on);
  for (unsigned int i = 0; i < split.size() - 1; i += 3) {
    coordinate = Vector2D(std::stod(split[i + 1]), std::stod(split[i + 2]));
    predicted_coords.push_back(coordinate);
  }
  return predicted_coords;
}

// move node "smoothly" towards the given position
void move_drones(Ptr<Node> drone, Vector position, double n_vel) {
  // get mobility model for drone
  Ptr<WaypointMobilityModel> mob = drone->GetObject<WaypointMobilityModel>();
  Vector m_position = mob->GetPosition();
  double distance = CalculateDistance(position, m_position);
  // 1 meter of accuracy is acceptable
  if (distance <= 1)
    return;

  unsigned int nodeId = drone->GetId();
  double currentTime = Simulator::Now().GetSeconds();
  double nWaypointTime;
  LOG("moving drone with nodeId: " << nodeId << " from " << m_position << " to "
                                   << position << " time: " << currentTime);

  if (mob->GetVelocity().GetLength() > 0) {
    mob->EndMobility();
    mob->AddWaypoint(Waypoint(Simulator::Now(), m_position));
  }

  nWaypointTime = distance / n_vel + currentTime;
  mob->AddWaypoint(Waypoint(Seconds(nWaypointTime), position));
}

std::list<Vector> get_cluster_centers() {
  std::ifstream infile("centers.txt");
  std::list<Vector> centers;
  double x, y, z;
  while (infile >> x >> y >> z) {
    Vector tmp = {x, y, z};
    centers.push_back(tmp);
  }
  infile.close();
  return centers;
}

Vector closest_center(Ptr<Node> drone, std::list<Vector> centers) {
  double min = numeric_limits<double>::infinity();
  double distance;
  Vector drone_position = drone->GetObject<MobilityModel>()->GetPosition();
  Vector closest;
  for (auto &&center_position : centers) {
    distance = CalculateDistance(drone_position, center_position);
    if (distance < min) {
      min = distance;
      closest = center_position;
    }
  }
  return closest;
}

std::list<Vector>::iterator find_center(Vector center,
                                        std::list<Vector> &centers) {
  std::list<Vector>::iterator position;
  for (position = centers.begin(); position != centers.end(); ++position) {
    if (position->x == center.x && position->y == center.y &&
        position->z == center.z) {
      break;
    }
  }
  return position;
}

void send_drones_to_cluster_centers(NodeContainer nodes, NodeContainer drones) {
  double now = Simulator::Now().GetSeconds();
  std::vector<Vector2D> predicted_coords;
  // save user positions to file
  if (enablePrediction && now >= 10) {
    predicted_coords = do_predictions();
    save_user_positions(nodes, predicted_coords);
  } else {
    save_user_positions(nodes);
  }
  // generate custering file
  exec(std::string("python3 ") + ns3_dir + std::string("/clustering.py ") +
       clustering_algoritm);
  // read cluster centers
  std::list<Vector> centers = get_cluster_centers();
  Vector center;
  std::list<Vector>::iterator erase_position;

  // iterate custer centers and send drones
  NodeContainer::Iterator drone;
  for (drone = drones.Begin(); drone != drones.End() && centers.size() > 0;
       ++drone) {
    center = closest_center(*drone, centers);
    erase_position = find_center(center, centers);
    centers.erase(erase_position);
    move_drones(*drone, center, 20);
  }

  // repeat
  Simulator::Schedule(Seconds(1), &send_drones_to_cluster_centers, nodes,
                      drones);
}

// initialize drones position
void set_drones(NodeContainer drones) {
  std::uniform_real_distribution<double> distribution(0, 1000);

  for (uint32_t i = 0; i < drones.GetN(); ++i) {
    Ptr<WaypointMobilityModel> mob =
        drones.Get(i)->GetObject<WaypointMobilityModel>();
    mob->AddWaypoint(
        Waypoint(Simulator::Now(),
                 Vector(distribution(generator), distribution(generator), 35)));
  }
}

void ThroughputMonitor(FlowMonitorHelper *fmhelper, Ptr<FlowMonitor> flowMon) {
  flowMon->CheckForLostPackets();
  uint32_t LostPacketsum = 0;
  float PDR, PLR, Delay, Throughput;
  auto flowStats = flowMon->GetFlowStats();
  Ptr<Ipv4FlowClassifier> classing =
      DynamicCast<Ipv4FlowClassifier>(fmhelper->GetClassifier());
  std::ofstream qos_file;
  qos_file.open("qos.txt", std::ofstream::out | std::ofstream::trunc);

  for (auto stats : flowStats) {
    Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow(stats.first);
    PDR = (100 * stats.second.rxPackets) / (stats.second.txPackets);
    LostPacketsum = (stats.second.txPackets) - (stats.second.rxPackets);
    PLR = ((LostPacketsum * 100) / stats.second.txPackets);
    Delay = (stats.second.delaySum.GetSeconds()) / (stats.second.txPackets);
    Throughput = stats.second.rxBytes * 8.0 /
                 (stats.second.timeLastRxPacket.GetSeconds() -
                  stats.second.timeFirstTxPacket.GetSeconds()) /
                 1024 / 1024;

    std::cout << "Flow ID			: " << stats.first << " ; "
              << fiveTuple.sourceAddress << " -----> "
              << fiveTuple.destinationAddress << std::endl;
    std::cout << "Tx Packets = " << stats.second.txPackets << std::endl;
    std::cout << "Rx Packets = " << stats.second.rxPackets << std::endl;
    std::cout << "Lost Packets = "
              << (stats.second.txPackets) - (stats.second.rxPackets)
              << std::endl;
    std::cout << "Packets Delivery Ratio (PDR) = " << PDR << "%" << std::endl;
    std::cout << "Packets Lost Ratio (PLR) = " << PLR << "%" << std::endl;
    std::cout << "Delay = " << Delay << " Seconds" << std::endl;
    std::cout << "Total Duration		: "
              << stats.second.timeLastRxPacket.GetSeconds() -
                     stats.second.timeFirstTxPacket.GetSeconds()
              << " Seconds" << std::endl;
    std::cout << "Last Received Packet	: "
              << stats.second.timeLastRxPacket.GetSeconds() << " Seconds"
              << std::endl;
    std::cout << "Throughput: " << Throughput << " Mbps" << std::endl;
    std::cout << "-------------------------------------------------------------"
                 "--------------"
              << std::endl;
    qos_file << fiveTuple.sourceAddress << " --> "
             << fiveTuple.destinationAddress << "," << PDR << "," << PLR << ","
             << Delay << "," << Throughput << "\n";
  }

  qos_file.close();
}

void request_video(Ptr<Node> sender_node, Ptr<Node> receiver_node) {
  static uint16_t m_port = 2000;
  static int request_id = 0;

  Ptr<Ipv4> ipv4 = sender_node->GetObject<Ipv4>();
  Ipv4InterfaceAddress iaddr = ipv4->GetAddress(1, 0);
  Ipv4Address ipAddr = iaddr.GetLocal();

  EvalvidServerHelper server(m_port);
  server.SetAttribute("SenderTraceFilename",
                      StringValue(ns3_dir + std::string("/st_highway_cif.st")));
  // server.SetAttribute("SenderDumpFilename", StringValue("evalvid_sd_" +
  // std::to_string(request_id)));
  server.SetAttribute("SenderDumpFilename",
                      StringValue("sd_" + std::to_string(request_id) + ".txt"));
  server.SetAttribute("PacketPayload", UintegerValue(512));
  ApplicationContainer apps = server.Install(sender_node);
  apps.Start(Seconds(5));

  EvalvidClientHelper client(ipAddr, m_port);
  // client.SetAttribute("ReceiverDumpFilename", StringValue("evalvid_rd_" +
  // std::to_string(request_id)));
  client.SetAttribute("ReceiverDumpFilename",
                      StringValue("rd_" + std::to_string(request_id) + ".txt"));
  apps = client.Install(receiver_node);
  apps.Start(Seconds(5));

  request_id++;
  m_port++;
}

void UDPApp(Ptr<Node> remoteHost, NodeContainer ueNodes) {
  // Install and start applications on UEs and remote host

  ApplicationContainer serverApps;
  ApplicationContainer clientApps;
  Time interPacketInterval = MilliSeconds(50);
  uint16_t dlPort = 1100;
  uint16_t ulPort = 2000;
  int startTime = 2;
  Ptr<Ipv4> remoteIpv4 = remoteHost->GetObject<Ipv4>();
  Ipv4Address remoteIpAddr =
      remoteIpv4->GetAddress(1, 0).GetLocal(); // Interface 0 is loopback

  for (uint32_t u = 0; u < ueNodes.GetN(); ++u) {
    Ptr<Node> ue = ueNodes.Get(u);
    Ptr<Ipv4> ueIpv4 = ue->GetObject<Ipv4>();
    Ipv4Address ueIpAddr = ueIpv4->GetAddress(1, 0).GetLocal();
    ulPort++;

    if (!disableDl) {
      PacketSinkHelper dlPacketSinkHelper(
          "ns3::UdpSocketFactory",
          InetSocketAddress(Ipv4Address::GetAny(), dlPort));
      serverApps.Add(dlPacketSinkHelper.Install(ue));

      UdpClientHelper dlClient(ueIpAddr, dlPort);
      dlClient.SetAttribute("Interval", TimeValue(interPacketInterval));
      dlClient.SetAttribute("MaxPackets", UintegerValue(1000000));
      dlClient.SetAttribute("PacketSize", UintegerValue(1024));
      clientApps.Add(dlClient.Install(remoteHost));
    }

    if (!disableUl) {
      ++ulPort;
      PacketSinkHelper ulPacketSinkHelper(
          "ns3::UdpSocketFactory",
          InetSocketAddress(Ipv4Address::GetAny(), ulPort));
      serverApps.Add(ulPacketSinkHelper.Install(remoteHost));

      UdpClientHelper ulClient(remoteIpAddr, ulPort);
      ulClient.SetAttribute("Interval", TimeValue(interPacketInterval));
      ulClient.SetAttribute("MaxPackets", UintegerValue(1000000));
      ulClient.SetAttribute("PacketSize", UintegerValue(1024));
      clientApps.Add(ulClient.Install(ue));
    }
  }

  serverApps.Start(Seconds(1));
  clientApps.Start(Seconds(startTime));
}

bool IsTopLevelSourceDir(std::string path) {
  bool haveVersion = false;
  bool haveLicense = false;

  //
  // If there's a file named VERSION and a file named LICENSE in this
  // directory, we assume it's our top level source directory.
  //

  std::list<std::string> files = SystemPath::ReadFiles(path);
  for (std::list<std::string>::const_iterator i = files.begin();
       i != files.end(); ++i) {
    if (*i == "VERSION") {
      haveVersion = true;
    } else if (*i == "LICENSE") {
      haveLicense = true;
    }
  }

  return haveVersion && haveLicense;
}

std::string GetTopLevelSourceDir(void) {
  std::string self = SystemPath::FindSelfDirectory();
  std::list<std::string> elements = SystemPath::Split(self);
  while (!elements.empty()) {
    std::string path = SystemPath::Join(elements.begin(), elements.end());
    if (IsTopLevelSourceDir(path)) {
      return path;
    }
    elements.pop_back();
  }
  NS_FATAL_ERROR("Could not find source directory from self=" << self);
}

int main(int argc, char *argv[]) {
  // LogComponentEnable("Config", LOG_LEVEL_ALL);
  LogComponentEnable("EvalvidClient", LOG_INFO);
  LogComponentEnable("EvalvidServer", LOG_INFO);

  CommandLine cmd;

  // Open file for writing and overwrite if it already exists
  // ues_sinr_file.open("ues_sinr.txt", std::ofstream::out |
  // std::ofstream::trunc); time_to_centroid_file.open("time_to_centroid.txt",
  // std::ofstream::out | std::ofstream::trunc);
  // ue_positions_log.open("ue_positions_log.txt",
  // std::ofstream::out | std::ofstream::trunc);
  // ue_positions_log << numUes << std::endl;

  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults();

  cmd.Parse(argc, argv);

  ns3::RngSeedManager::SetSeed(
      seedValue); // valor de seed para geração de números aleatórios
  ns3_dir = GetTopLevelSourceDir();

  if (useCa) {
    Config::SetDefault("ns3::LteHelper::UseCa", BooleanValue(useCa));
    Config::SetDefault("ns3::LteHelper::NumberOfComponentCarriers",
                       UintegerValue(2));
    Config::SetDefault("ns3::LteHelper::EnbComponentCarrierManager",
                       StringValue("ns3::RrComponentCarrierManager"));
  }

  // ues_sinr.resize(numUes);

  // Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
  lteHelper->SetEpcHelper(epcHelper);
  Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(320));

  // error modes for ctrl and data planes
  Config::SetDefault("ns3::LteSpectrumPhy::CtrlErrorModelEnabled",
                     BooleanValue(false));
  Config::SetDefault("ns3::LteSpectrumPhy::DataErrorModelEnabled",
                     BooleanValue(false));

  // todo: add this on verbose
  Ptr<Node> pgw = epcHelper->GetPgwNode();

  NS_LOG_UNCOND("Pathloss model: NakagamiPropagationLossModel ");
  // Config::SetDefault("ns3::ItuR1411NlosOverRooftopPropagationLossModel::StreetsOrientation",
  // DoubleValue(10)); lteHelper->SetAttribute("PathlossModel",
  // StringValue("ns3::HybridBuildingsPropagationLossModel"));
  // lteHelper->SetPathlossModelAttribute("Frequency", DoubleValue(2.0e9));
  // lteHelper->SetPathlossModelAttribute("ShadowSigmaExtWalls",
  // DoubleValue(0)); lteHelper->SetPathlossModelAttribute("ShadowSigmaOutdoor",
  // DoubleValue(3.0)); lteHelper->SetPathlossModelAttribute("Los2NlosThr",
  // DoubleValue(200));
  lteHelper->SetHandoverAlgorithmType("ns3::NoOpHandoverAlgorithm");
  // lteHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(0));
  // lteHelper->SetHandoverAlgorithmAttribute("TimeToTrigger",
  // TimeValue(MilliSeconds(10)));

  // todo: substitute for multiple MEC servers
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create(node_remote);
  Ptr<Node> remoteHost = remoteHostContainer.Get(0);

  InternetStackHelper internet;
  internet.Install(remoteHostContainer);

  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
  p2ph.SetDeviceAttribute("Mtu", UintegerValue(1400));
  p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
  // p2ph.EnablePcapAll("zob");
  NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces;
  internetIpIfaces = ipv4h.Assign(internetDevices);

  Ipv4Address remoteHostAddr;
  remoteHostAddr = internetIpIfaces.GetAddress(1);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
      ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
  remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
                                             Ipv4Mask("255.0.0.0"), 1);
  remoteHostStaticRouting->AddNetworkRouteTo(
      Ipv4Address("10.0.0.0"), Ipv4Mask("255.255.255.192"), 1); // Route to UAVs

  Ptr<Ipv4StaticRouting> pgwStaticRouting =
      ipv4RoutingHelper.GetStaticRouting(pgw->GetObject<Ipv4>());
  pgwStaticRouting->AddNetworkRouteTo(Ipv4Address("10.0.0.0"),
                                      Ipv4Mask("255.255.255.192"), 2);

  UAVNodes.Create(numUAVs);
  ueNodes.Create(numUes);
  BSNodes.Create(numStaticCells);
  ServerNodes.Create(numStaticCells); // todo

  internet.Install(ueNodes);

  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(remoteHost);
  mobility.Install(pgw);
  BuildingsHelper::Install(remoteHost);
  BuildingsHelper::Install(pgw);

  MobilityHelper UAVmobility;
  UAVmobility.SetMobilityModel("ns3::WaypointMobilityModel");
  UAVmobility.Install(UAVNodes);
  BuildingsHelper::Install(UAVNodes);

  mobility.SetPositionAllocator(
      "ns3::RandomDiscPositionAllocator", "X",
      DoubleValue(
          500), // The x coordinate of the center of the random position disc.
      "Y",
      DoubleValue(
          500), // The y coordinate of the center of the random position disc.
      "Z", DoubleValue(1.5), // The z coordinate of all positions in the disc.
      "Rho",
      StringValue(
          "ns3::UniformRandomVariable[Min=0|Max=400]")); // A random variable
                                                         // which represents the
                                                         // radius of a position
                                                         // in a random disc.

  mobility.SetMobilityModel(
      "ns3::RandomWalk2dMobilityModel", "Bounds",
      RectangleValue(Rectangle(-1000, 1000, -1000, 1000)));
  mobility.Install(ueNodes);

  // enb positioning
  Ptr<ListPositionAllocator> HpnPosition =
      CreateObject<ListPositionAllocator>();
  HpnPosition = positionAllocator(HpnPosition);

  MobilityHelper mobilityEnb;
  mobilityEnb.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobilityEnb.SetPositionAllocator(HpnPosition);
  mobilityEnb.Install(BSNodes);
  mobilityEnb.Install(ServerNodes);

  for (uint32_t u = 0; u < ueNodes.GetN(); ++u) {
    Ptr<Node> ueNode = ueNodes.Get(u);
    Ptr<Ipv4StaticRouting> ueStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
    ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(),
                                     1);
  }

  // todo: change for uavs
  lteHelper->SetEnbDeviceAttribute("DlBandwidth",
                                   UintegerValue(25)); // Set Download BandWidth
  lteHelper->SetEnbDeviceAttribute("UlBandwidth",
                                   UintegerValue(25)); // Set Upload Bandwidth
  enbDevs = lteHelper->InstallEnbDevice(NodeContainer(BSNodes, UAVNodes));

  ueDevs = lteHelper->InstallUeDevice(ueNodes);

  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

  // attach to cells with the highest sinr
  lteHelper->Attach(ueDevs);

  // todo: change this to set power individually instead of globally
  Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(eNodeBTxPower));
  // Config::SetDefault("ns3::LteEnbPhy::NoiseFigure", DoubleValue(5)); //
  // Default 5

  for (uint32_t u = 0; u < UAVNodes.GetN(); ++u) {
    Ptr<Node> UAVNode = UAVNodes.Get(u);
    Ptr<Ipv4> UAVIpv4 = UAVNode->GetObject<Ipv4>();
    Ptr<Ipv4StaticRouting> UAVStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(UAVIpv4);
    UAVStaticRouting->AddNetworkRouteTo(Ipv4Address("1.0.0.0"),
                                        Ipv4Mask("255.0.0.0"), 1);
  }
  Ptr<Node> sgw = epcHelper->GetSgwNode();
  Ptr<Ipv4> sgwIpv4 = sgw->GetObject<Ipv4>();
  Ptr<Ipv4StaticRouting> sgwStaticRouting =
      ipv4RoutingHelper.GetStaticRouting(sgwIpv4);
  sgwStaticRouting->AddNetworkRouteTo(Ipv4Address("1.0.0.0"),
                                      Ipv4Mask("255.0.0.0"), 1);

  // Setup Applications
  // UDPApp(remoteHost, NodeContainer(ueNodes));

  for (uint32_t i = 0; i < UAVNodes.GetN(); ++i) {
    request_video(UAVNodes.Get(i), remoteHost);
  }

  AnimationInterface animator("lte_animation.xml");
  animator.SetMobilityPollInterval(Seconds(1));
  for (uint32_t i = 0; i < UAVNodes.GetN(); ++i) {
    animator.UpdateNodeDescription(UAVNodes.Get(i), "UAV " + std::to_string(i));
    animator.UpdateNodeColor(UAVNodes.Get(i), 250, 200, 45);
    animator.UpdateNodeSize(UAVNodes.Get(i)->GetId(), 10,
                            10); // to change the node size in the animation.
  }

  for (uint32_t j = 0; j < ueNodes.GetN(); ++j) {
    animator.UpdateNodeDescription(ueNodes.Get(j), "UE " + std::to_string(j));
    animator.UpdateNodeColor(ueNodes.Get(j), 20, 10, 145);
    animator.UpdateNodeSize(ueNodes.Get(j)->GetId(), 10, 10);
  }

  for (uint32_t j = 0; j < BSNodes.GetN(); ++j) {
    animator.UpdateNodeDescription(BSNodes.Get(j), "Cell " + std::to_string(j));
    animator.UpdateNodeColor(BSNodes.Get(j), 20, 10, 145);
    animator.UpdateNodeSize(BSNodes.Get(j)->GetId(), 10, 10);
  }

  for (uint32_t k = 0; k < remoteHostContainer.GetN(); ++k) {
    animator.UpdateNodeDescription(remoteHostContainer.Get(k),
                                   "RemoteHost " + std::to_string(k));
    animator.UpdateNodeColor(remoteHostContainer.Get(k), 110, 150, 45);
    animator.UpdateNodeSize(remoteHostContainer.Get(k)->GetId(), 10, 10);
  }

  unsigned int id;
  std::ostringstream oss;
  for (uint32_t u = 0; u < UAVNodes.GetN(); ++u) {
    id = UAVNodes.Get(u)->GetId();
    oss << "/NodeList/" << id << "/$ns3::MobilityModel/CourseChange";
    Config::Connect(oss.str(), MakeCallback(&CourseChange));
    oss.str("");
  }

  // intall flow monitor and get stats
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();
  Ptr<Ipv4FlowClassifier> classifier =
      DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());

  // for (uint32_t ib = 0; ib <= SimTime; ib++) {
  //     Simulator::Schedule(Seconds(ib), &print_position, ueNodes);
  // }

  // Simulator::Schedule(Seconds(SimTime - 0.001), &write_metrics);
  Simulator::Schedule(Seconds(SimTime - 0.01), ThroughputMonitor, &flowmon,
                      monitor);
  if (enablePrediction) {
    Simulator::Schedule(Seconds(1), &log_ue_positions, NodeContainer(ueNodes),
                        &ue_positions_log);
  }
  Simulator::Schedule(Seconds(1), &send_drones_to_cluster_centers,
                      NodeContainer(ueNodes), UAVNodes);
  // Simulator::Schedule(Seconds(1), &print_meas);

  // set initial positions of drones
  // set_drones(UAVNodes);

  // save user positions to file
  // save_user_positions(NodeContainer(ueNodes));

  // BuildingsHelper::MakeMobilityModelConsistent();

  /* handover reporting callbacks*/
  Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                  MakeCallback(&NotifyHandoverStartEnb));
  Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
                  MakeCallback(&NotifyConnectionEstablishedUe));
  Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
                  MakeCallback(&NotifyHandoverStartUe));
  Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                  MakeCallback(&NotifyHandoverEndOkEnb));
  Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
                  MakeCallback(&NotifyHandoverEndOkUe));

  /* signal reporting callbacks */
  Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/LteUePhy/"
                  "ReportUeMeasurements",
                  MakeCallback(&ReportUeMeasurementsCallback));
  Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/RecvMeasurementReport",
                  MakeCallback(&RecvMeasurementReportCallback));

  lteHelper->EnableTraces(); // enable all traces

  // Ptr<RadioEnvironmentMapHelper> remHelper =
  // CreateObject<RadioEnvironmentMapHelper>();
  // remHelper->SetAttribute("ChannelPath", StringValue("/ChannelList/0"));
  // remHelper->SetAttribute("OutputFile", StringValue("rem.out"));
  // remHelper->SetAttribute("XMin", DoubleValue(-400.0));
  // remHelper->SetAttribute("XMax", DoubleValue(400.0));
  // remHelper->SetAttribute("XRes", UintegerValue(100));
  // remHelper->SetAttribute("YMin", DoubleValue(-300.0));
  // remHelper->SetAttribute("YMax", DoubleValue(300.0));
  // remHelper->SetAttribute("YRes", UintegerValue(75));
  // remHelper->SetAttribute("Z", DoubleValue(0.0));
  // remHelper->SetAttribute("UseDataChannel", BooleanValue(true));
  // remHelper->SetAttribute("RbId", IntegerValue(10));
  // remHelper->Install();

  Simulator::Stop(Seconds(SimTime));
  Simulator::Run();

  monitor->CheckForLostPackets();
  FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
  for (map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin();
       i != stats.end(); ++i) {
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
    std::cout << "Flow " << i->first << " (" << t.sourceAddress << " ->"
              << t.destinationAddress << ")\n";
    std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
    std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
    std::cout << "  TxOffered:  " << i->second.txBytes * 8.0 / 9.0 / 1000 / 1000
              << " Mbps\n";
    std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
    std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
    std::cout << "  Lost Packets:   " << i->second.lostPackets << "\n";
    std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / 9.0 / 1000 / 1000
              << " Mbps\n";
    if (i->second.rxBytes)
      std::cout << "  DelaySum: "
                << i->second.jitterSum /
                       (i->second.rxPackets + i->second.txPackets)
                << "\n";
    std::cout << "......................................\n";
  }

  // serialize flow monitor to xml
  flowmon.SerializeToXmlFile("drones_flowmon.xml", true, true);

  Simulator::Destroy();
  // BuildingsHelper::Install(ueNodes);

  // ues_sinr_file.close();
  // time_to_centroid_file.close();
  // ue_positions_log.close();

  return 0;
}
