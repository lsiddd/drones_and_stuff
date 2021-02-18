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
#include <ns3/csma-module.h>

#include <boost/algorithm/string/classification.hpp> // Include boost::for is_any_of
#include <boost/algorithm/string/split.hpp>          // Include for boost::split

#define LOG(x) std::cout << x << std::endl
#define wait std::cin.get()

using namespace ns3;

std::default_random_engine generator;

NS_LOG_COMPONENT_DEFINE("LteEvalvid");

// scenario variables
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
const uint32_t numUAVs = 10;
const uint32_t number_of_surges = 10;
const uint32_t numUes = 20;
const uint32_t numStaticCells = 30;
const uint32_t numEdgeServers = numStaticCells;
// uint32_t numCars = 10;
uint32_t seedValue = 10000;
uint32_t SimTime = 10;
int eNodeBTxPower = 46;
const uint32_t numBSs = numUAVs + numStaticCells;
uint16_t node_remote = 1; // HOST_REMOTO
std::string mobil_trace = "koln.tcl";
std::string requests_trace = "requests.tcl_but_not_really";
float distance_multiplier = 1.0 / 10;
double cell_thr_average = 0; // average throughput of used cells

/* connection management structures */
int connections[numBSs][numUes]{{0}};
int neighbors[numBSs][numUes]{{0}};
int handoverPredictions[numUes][3]{{0}};
int edgeUe[numEdgeServers][numUes]{{0}};
int edgeMigrationChart[numUes][numEdgeServers]{{0}};
Ipv4Address serverNodesAddresses[numEdgeServers][2];
std::unordered_map<int, Ipv4Address> user_ip;
std::unordered_map<int, double> cell_throughput;

std::map<int, std::map<int, int>> rnti_cells;

// mapping nodelist ids to actual imsis
std::map<int, int> path_imsi;

// /*MIGRATION VARIABLES*/
// enable logs
// perform migrations
bool doMigrate = true;
// uint32_t numEdgeServers;
Time managerInterval = Seconds(1);
std::string algorithm = "mosca";

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
vector<uint16_t> resources{{10}};

// type of cell position allocation
bool rowTopology = false;
bool randomCellAlloc = true;

// ----------VARIABLES TO CALCULATE METRICS-----------
std::vector<int> latency;
/* END OF MIGRATION VARIABLES */

std::vector<int> cost;
// node containers as global objects to make mobility management possible
NodeContainer UAVNodes;
NodeContainer ueNodes;
NodeContainer BSNodes;
// todo: test servers in the BS nodes
NodeContainer serverNodes;

NetDeviceContainer enbDevs;
NetDeviceContainer ueDevs;

// global lte helper for mobility management
Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();

// populate pairing from nodeid to imsi
int populate_path_imsi(std::string path, int imsi)
{
  int nodeid;

  std::vector<std::string> split_path;
  boost::split(split_path, path, boost::is_any_of("/"));
  nodeid = stoi(split_path[2]);

  if (imsi != -1)
    path_imsi[nodeid] = imsi;

  return nodeid;
}

Vector get_node_position(Ptr<Node> node)
{
  Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
  return mob->GetPosition();
}

int get_cell(int user_id)
{
  for (uint32_t i = 0; i < numBSs + numUAVs; i++)
  {
    if (connections[i][user_id])
      return i;
  }
  return -1;
}

int get_imsi(int cellId, int rnti)
{
  return rnti_cells[cellId][rnti] == 0 ? -1 : rnti_cells[cellId][rnti];
}

void print_meas()
{
  for (uint32_t i = 0; i < numBSs; i++)
  {
    for (uint32_t j = 0; j < numUes; j++)
    {
      std::cout << connections[i][j] << "\t";
    }
    std::cout << std::endl;
  }
  Simulator::Schedule(Seconds(1), &print_meas);
}

int getServingcell(int imsi)
{
  int servingCell = 0;
  for (uint32_t i = 0; i < numBSs; i++)
  {
    if (connections[i][imsi - 1] != 0)
    {
      servingCell = i;
    }
  }
  return servingCell;
}

int getNodeId(Ptr<Node> node, string type = "server")
{

  // seleced the desired node container
  NodeContainer tmpNodesContainer;
  if (type == "server")
    tmpNodesContainer = serverNodes;
  else if (type == "ue")
    tmpNodesContainer = ueNodes;
  else if (type == "enb")
    tmpNodesContainer = BSNodes;

  // find th enode id
  for (uint32_t i = 0; i < tmpNodesContainer.GetN(); ++i)
  {
    if (node == tmpNodesContainer.Get(i))
    {
      // NS_LOG_UNCOND("node " << node << " is " << tmpNodesContainer.Get(i) <<
      // " ?");
      return i;
    }
  }

  return -1;
}

void handoverManager(std::string path)
{
  // random handover
  // get serving cell of user

  for (uint32_t i = 0; i < numUes; i++)
  {
    int imsi = i + 1;
    uint32_t servingCell = getServingcell(imsi);
    int rsrp = std::numeric_limits<int>::lowest();
    int strongestNeighborCell = 0;

    for (uint32_t cell = 0; cell < numBSs; cell++)
    {
      if (neighbors[cell][i] > rsrp && cell != servingCell)
      {
        rsrp = neighbors[cell][i];
        strongestNeighborCell = cell;
      }
    }
    // Randomly handover to next strongest cell
    if (rsrp > neighbors[servingCell][imsi - 1])
    {
      LOG("Executing handover for path " << path);
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

  if (verbose)
  {
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

    if (path_imsi[node_id] != -1)
    {
      neighbors[cellId - 1][path_imsi[node_id] - 1] = rsrp;
    }
  }
  // call handover manager upon receiving new measurements
  // todo: add some fail checks for signal levels
  handoverManager(path);
}

void RecvMeasurementReportCallback(std::string path, uint64_t imsi,
                                   uint16_t cellId, uint16_t rnti,
                                   LteRrcSap::MeasurementReport meas)
{
  if (verbose)
  {
    LOG("Simulation time: " << Simulator::Now().GetSeconds());
    LOG(path);
    LOG(imsi);
    LOG(cellId);
    LOG(rnti);
    LOG((int)meas.measResults.measId);
    LOG("\n");
  }
}

void NotifyConnectionEstablishedUe(std::string context, uint64_t imsi,
                                   uint16_t cellid, uint16_t rnti)
{
  LOG(Simulator::Now().GetSeconds()
      << " " << context << " UE IMSI " << imsi << ": connected to CellId "
      << cellid << " with RNTI " << rnti << "\n");
  for (uint32_t i = 0; i < numBSs; ++i)
  {
    connections[i][imsi - 1] = 0;
  }
  connections[cellid - 1][imsi - 1] = rnti;
  rnti_cells[cellid][rnti] = imsi;

  // // migration connection stablished
  // if (resources[cellid - 1] == 0)
  // {
  //   int fallbackServer;
  //   // iterate untill an edge with available resources has been chosen
  //   do
  //   {
  //     fallbackServer = rand() % (numEdgeServers);
  //     NS_LOG_UNCOND("fallbackServer " << fallbackServer);
  //     // make while condition true to reiterate
  //     if (resources[fallbackServer] == 0)
  //       fallbackServer = cellid - 1;
  //   } while (fallbackServer == cellid - 1);

  //   std::cout << "Failed to allocate user" << imsi << " in edge " << cellid -
  //   1
  //             << "\n";
  //   std::cout << "allocating to random fog " << fallbackServer << endl;
  //   resources[fallbackServer]--;
  //   edgeUe[fallbackServer][imsi - 1] = 1;
  // }
  // else
  // {
  //   std::cout << "User " << imsi << " connected to edge " << cellid - 1 <<
  //   endl; edgeUe[cellid - 1][imsi - 1] = 1; resources[cellid - 1]--;
  // }
}

void NotifyHandoverStartUe(std::string context, uint64_t imsi, uint16_t cellId,
                           uint16_t rnti, uint16_t targetCellId)
{
  std::cout << Simulator::Now().GetSeconds() << " " << context << " UE IMSI "
            << imsi << ": previously connected to CellId " << cellId
            << " with RNTI " << rnti << ", doing handover to CellId "
            << targetCellId << std::endl;
}
void NotifyHandoverEndOkUe(std::string context, uint64_t imsi, uint16_t cellId,
                           uint16_t rnti)
{
  std::cout << Simulator::Now().GetSeconds() << " " << context << " UE IMSI "
            << imsi << ": successful handover to CellId " << cellId

            << " with RNTI " << rnti << std::endl;
}
void NotifyHandoverStartEnb(std::string context, uint64_t imsi, uint16_t cellId,
                            uint16_t rnti, uint16_t targetCellId)
{
  std::cout << Simulator::Now().GetSeconds() << " " << context << " eNB CellId "
            << cellId << ": start handover of UE with IMSI " << imsi << " RNTI "
            << rnti << " to CellId " << targetCellId << std::endl;
}
void NotifyHandoverEndOkEnb(std::string context, uint64_t imsi, uint16_t cellId,
                            uint16_t rnti)
{
  std::cout << Simulator::Now().GetSeconds() << " " << context << " eNB CellId "
            << cellId << ": completed handover of UE with IMSI " << imsi
            << " RNTI " << rnti << std::endl;
}

Ptr<ListPositionAllocator>
generatePositionAllocator(int number_of_nodes = 300, int area = 1000,
                          std::string allocation = "random")
{

  Ptr<ListPositionAllocator> HpnPosition =
      CreateObject<ListPositionAllocator>();
  std::uniform_int_distribution<int> distribution(0, area);

  if (allocation == "koln")
  {
    std::ifstream cellList("cellList_koln");
    double a, b, c;
    while (cellList >> a >> b >> c)
    {
      LOG("adding cell to position " << b * distance_multiplier << " "
                                     << c * distance_multiplier);
      HpnPosition->Add(
          Vector3D(b * distance_multiplier, c * distance_multiplier, 45));
    }
  }

  else
  {
    for (int i = 0; i < number_of_nodes; i++)
    {
      HpnPosition->Add(
          Vector3D(distribution(generator), distribution(generator), 45));
    }
  }
  return HpnPosition;
}

std::string exec(std::string cmd)
{
  std::array<char, 128> buffer;
  std::string result;
  std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
  if (!pipe)
    throw std::runtime_error("popen() failed!");
  while (!feof(pipe.get()))
  {
    if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
      result += buffer.data();
  }
  return result;
}

/*  MIGRATION FUNCTIONS */

int getEdge(int nodeId)
{
  int edgeId = -1;
  for (uint32_t i = 0; i < numEdgeServers; ++i)
    if (edgeUe[i][nodeId])
    {
      edgeId = i;
    }
  return edgeId;
}

void HandoverPrediction(int nodeId, int timeWindow)
{
  std::string mobilityTrace = mobil_trace;
  // means no connection has been found
  // happens if it's called too early in the simulation
  int imsi = nodeId - 1;
  int servingCell = getServingcell(imsi);
  if (servingCell == -1)
    return;

  // receive a nodeId, and a time window, and return if a handover is going to
  // happen in this time window.
  std::ifstream mobilityFile(mobilityTrace);
  NS_ASSERT_MSG(mobilityFile.is_open(), "Error opening prediction file.");

  string nodeColumn;
  string fileLines;

  // coordinate variables
  double node_x, node_y, node_z, node_position_time;
  double shortestDistance = numeric_limits<int>::max();
  int closestCell = numeric_limits<int>::max();

  // tmp veriables to read file
  // node_position_time = time of the position
  string aux1, aux2, aux4, aux5;
  string cell_id;

  while (getline(mobilityFile, fileLines))
  {
    if (fileLines.find("setdest") != string::npos)
    {

      std::stringstream ss(fileLines);
      // cout << ss.str();
      ss >> aux1 >> aux2 >> node_position_time >> aux4 >> aux5 >> node_x >>
          node_y >> node_z;

      nodeColumn = "\"$node_(" + to_string(nodeId) + ")";
      // cout << "nodeColumn" << nodeColumn << "\n";
      // cout << "aux: " << aux4 << "\n";
      // cin.get();

      // for (int time_offset = 0; time_offset < timeWindow; time_offset++)
      if (aux4 == nodeColumn && Simulator::Now().GetSeconds() + timeWindow ==
                                    round(node_position_time))
      {
        Vector uePos = Vector(node_x, node_y, node_z);

        // double distanceServingCell = CalculateDistance(uePos,
        // enbNodes.Get(getCellId(nodeId))->GetObject<MobilityModel>()->GetPosition
        // ());

        // calculate distance from node to each enb
        for (uint32_t i = 0; i < numStaticCells; ++i)
        {
          // get Ith enb  position
          Vector enbPos =
              BSNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
          // get distance
          double distanceUeEnb = CalculateDistance(uePos, enbPos);

          // get closest enb
          if (distanceUeEnb < shortestDistance)
          {
            closestCell = i;
            shortestDistance = distanceUeEnb;
          }
        }

        // if closest enb != current, predict handover
        if (closestCell != servingCell)
        {
          std::cout << "Handover to happen at " << node_position_time << endl;
          std::cout << "Node " << nodeId << " from cell " << servingCell
                    << " to cell " << closestCell << endl;
          handoverPredictions[nodeId][0] = node_position_time;
          handoverPredictions[nodeId][1] = servingCell;
          handoverPredictions[nodeId][2] = closestCell;
        }
      }
    }
  }
}

void migrate(Ptr<Node> sourceServer, Ptr<Node> targetServer,
             Ipv4Address sourceServerAddress, Ipv4Address targetServerAddress)
{
  static int migrationPort = 10000;
  // return if migration is not available
  if (!doMigrate)
  {
    std::cout << "Migration not enabled. :(\n";
    // return;
  }

  if (resources[getNodeId(targetServer)] <= 0)
  {
    NS_LOG_UNCOND("MIGRATION FAILED FOR LACK OF RESOURCES");
    return;
  }
  if (getNodeId(targetServer) < 0)
    return;

  NS_LOG_UNCOND("Migration from node " << getNodeId(sourceServer) << " to node "
                                       << getNodeId(targetServer));

  resources[getNodeId(sourceServer)]++;
  resources[getNodeId(targetServer)]--;

  // stop before any traffic is actually sent
  // do this to speed up the simulation
  return;

  // cout << "Starting migration from node " << sourceServerAddress << " to node
  // " << targetServerAddress << ".\n";
  ++migrationPort;
  UdpServerHelper server(migrationPort);
  ApplicationContainer apps = server.Install(targetServer);
  apps.Start(Simulator::Now());
  // apps.Stop (Simulator::Now()+Seconds(5));

  //
  // Create one UdpClient application to send UDP datagrams from node zero to
  // node one.
  //

  uint32_t MaxPacketSize = 1024;
  // uint32_t maxPacketCount = migrationSize / MaxPacketSize;
  uint32_t maxPacketCount = 10000;
  // tyr to migrate this in 10 senconds at most
  Time interPacketInterval = MilliSeconds(1);
  UdpClientHelper client(targetServerAddress, migrationPort);
  client.SetAttribute("MaxPackets", UintegerValue(maxPacketCount));
  client.SetAttribute("Interval", TimeValue(interPacketInterval));
  client.SetAttribute("PacketSize", UintegerValue(MaxPacketSize));
  apps = client.Install(sourceServer);
  apps.Start(Simulator::Now());
}

void requestApplication(Ptr<Node> ueNode, Ptr<Node> targetServer,
                        double payload = 0)
{

  static int applicationPort = 11000;

  Ptr<Ipv4> remoteIpv4 = targetServer->GetObject<Ipv4>();
  Ipv4Address remoteIpAddr =
      remoteIpv4->GetAddress(1, 0).GetLocal(); // Interface 0 is loopback

  ++applicationPort;
  UdpServerHelper server(applicationPort);
  ApplicationContainer apps = server.Install(targetServer);
  apps.Start(Simulator::Now());

  Time interPacketInterval = MilliSeconds(10);
  uint32_t maxPacketCount;
  uint32_t MaxPacketSize = 1024;
  // uint32_t maxPacketCount = migrationSize / MaxPacketSize;
  if (payload != 0)
  {
    maxPacketCount = payload / 1024;

    //call a default value in case no valid amount is available
    if (maxPacketCount == 0) {
      maxPacketCount = 100;
    }
    interPacketInterval = Seconds(1) / maxPacketCount;
  }
  else
  {
    maxPacketCount = 50;
    interPacketInterval = MilliSeconds(10);
  }

  // tyr to migrate this in 10 senconds at most
  UdpClientHelper client(remoteIpAddr, applicationPort);
  client.SetAttribute("MaxPackets", UintegerValue(maxPacketCount));
  client.SetAttribute("Interval", TimeValue(interPacketInterval));
  client.SetAttribute("PacketSize", UintegerValue(MaxPacketSize));
  apps = client.Install(ueNode);
  apps.Start(Simulator::Now());
}

void UAVManager()
{
  Time warm_up = Seconds(10);
  if (Simulator::Now() < warm_up)
  {
    int counter = 0;
    double throughput{0};
    for (auto &thr : cell_throughput)
    {
      if (thr.second)
      {
        counter++;
        throughput += thr.second;
      }
    }
    // only calculate if there are user flows
    if (counter)
    {
      cell_thr_average = throughput / counter;
      LOG(cell_thr_average);
      wait;
    }
  }

  return;
}

// migrations manager
void manager()
{
  double weights[3] = {57, 14, 28};

  Simulator::Schedule(managerInterval, &manager);

  std::cout << "manager started at " << Simulator::Now().GetSeconds() << " \n";

  for (uint32_t i = 0; i < serverNodes.GetN(); ++i)
  {
    std::cout << "server n " << i << " with " << resources[i]
              << " resource units\n";
  }

  std::cout << "..................................\n\n\n";

  for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
  {
    // check if node is being served

    int serving_node = getEdge(i);
    NS_LOG_UNCOND("Serving node: " << serving_node);

    if (serving_node != -1)
    {

      if (algorithm == "nomigration" || algorithm == "greedy")
        continue;

      int bestEdgeServer = -1;
      int greatestScore = -1;
      int edgeId = 0;

      // I will just assume that the predictions are right
      HandoverPrediction(i, 5);

      // if a handover is going to happen
      if (Seconds(handoverPredictions[i][0]) > Simulator::Now())
      {
        // for (int edgeId = 0; edgeId < numEdgeNodes; ++edgeId) {
        while ((uint32_t)edgeId < serverNodes.GetN())
        {
          double score = 0;

          // server characteristics
          double latency_score = 0;
          double cost_score = 0;
          double resource_score = 0;
          // if I cared a bit more I'd use the limits.h
          //  library to set this to the max, but I don't
          int server_latency = 999999;

          // get server metrics

          latency_score = (1.0 / serverReqs[1][0]) + weights[0];
          resource_score = (serverReqs[1][1]) + weights[1];
          cost_score = (1.0 / serverReqs[1][2]) + weights[2];
          server_latency = serverReqs[1][0];

          // Get the last insertion to the latency container
          // and check if requirements are being met
          // or if server is out of resources
          if (server_latency > applicationReqs[applicationType][0] ||
              resources[edgeId] == 0)
            score = 0;
          else
            score = latency_score + cost_score + resource_score;

          // in the case of qos-based
          if (algorithm == "qos")
            score = 1.0 / serverReqs[1][0];

          LOG(Simulator::Now().GetSeconds()
              << " -- server " << edgeId << " score: " << score);

          // get greated score
          if (score > greatestScore)
          {
            greatestScore = score;
            bestEdgeServer = edgeId;
          }
          edgeId++;
        }
        if (bestEdgeServer != serving_node)
        {
          if (edgeMigrationChart[i][bestEdgeServer] + 5 >
              Simulator::Now().GetSeconds())
            ; // do nothing
          // return;
          else
          {
            migrate(serverNodes.Get(serving_node),
                    serverNodes.Get(bestEdgeServer),
                    serverNodesAddresses[serving_node][1],
                    serverNodesAddresses[bestEdgeServer][1]);
            edgeMigrationChart[i][bestEdgeServer] =
                Simulator::Now().GetSeconds();
            edgeUe[serving_node][i] = 0;
            edgeUe[bestEdgeServer][i] = 1;
          }
        }
      }

      // renew applications periodically
      requestApplication(ueNodes.Get(i), serverNodes.Get(serving_node));
    }
    else
    {
      NS_LOG_UNCOND("Node " << i << " not being served?");
    }
  }
}

void write_metrics()
{
  std::stringstream sinrdata;
  unsigned int qtyUEs = ues_sinr.size();
  unsigned int qtyUEsCovered = 0;
  float coverageRatio = 0;
  for (unsigned int id = 0; id < qtyUEs; ++id)
  {
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
  for (auto time : time_to_centroid)
  {
    total_time += time;
    timedata << time << std::endl;
  }
  mean_time = total_time / active_drones;
  time_to_centroid_file << "Mean time to centroid: " << mean_time << std::endl;
  time_to_centroid_file << timedata.str();
}

void print_position(NodeContainer ueNodes)
{
  std::cout << '\n';
  for (uint32_t j = 0; j < ueNodes.GetN(); ++j)
  {
    Ptr<MobilityModel> mob = ueNodes.Get(j)->GetObject<MobilityModel>();
    Vector pos = mob->GetPosition();
    Vector3D speed = mob->GetVelocity();
    std::cout << " Node " << j << " | POS: (x=" << pos.x << ", y=" << pos.y
              << ") | Speed(" << speed.x << ", " << speed.y << ")" << std::endl;
  }
}

void save_user_positions(NodeContainer nodes)
{
  std::ofstream pos_file("positions.txt");
  for (uint32_t i = 0; i < nodes.GetN(); ++i)
  {
    Ptr<MobilityModel> mob = nodes.Get(i)->GetObject<MobilityModel>();
    Vector pos = mob->GetPosition();
    pos_file << pos.x << " " << pos.y << " " << pos.z << "\n";
  }
  pos_file.close();
}

void save_user_positions(NodeContainer nodes,
                         std::vector<Vector2D> predicted_coords)
{
  Vector2D coords;
  std::ofstream pos_file("positions.txt");
  for (uint32_t i = 0; i < nodes.GetN(); ++i)
  {
    Ptr<MobilityModel> mob = nodes.Get(i)->GetObject<MobilityModel>();
    Vector pos = mob->GetPosition();
    coords = predicted_coords[i];
    pos_file << coords.x << " " << coords.y << " " << pos.z << "\n";
  }
  pos_file.close();
}

void log_ue_positions(NodeContainer UEs, std::ofstream *ue_positions_log)
{
  double now = Simulator::Now().GetSeconds();
  for (NodeContainer::Iterator i = UEs.Begin(); i != UEs.End(); ++i)
  {
    Ptr<Node> UE = *i;
    Ptr<MobilityModel> UEposition = UE->GetObject<MobilityModel>();
    Vector pos = UEposition->GetPosition();
    *ue_positions_log << now << "," << UE->GetId() << "," << pos.x << ","
                      << pos.y << "\n";
  }
  ue_positions_log->flush();
  Simulator::Schedule(Seconds(1), &log_ue_positions, UEs, ue_positions_log);
}

std::vector<Vector2D> do_predictions()
{
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
  for (unsigned int i = 0; i < split.size() - 1; i += 3)
  {
    coordinate = Vector2D(std::stod(split[i + 1]), std::stod(split[i + 2]));
    predicted_coords.push_back(coordinate);
  }
  return predicted_coords;
}

// move node "smoothly" towards the given position
void move_drones(Ptr<Node> drone, Vector position, double n_vel)
{
  double interval = 0.1;
  double new_n_vel = interval * n_vel;

  // get mobility model for drone
  Vector m_position = get_node_position(drone);
  double distance = CalculateDistance(position, m_position);

  // 1meter of accuracy is acceptable
  if (distance > 1)
  {
    Vector diff = position - m_position;

    double len = diff.GetLength();
    Vector new_pos = m_position + Vector((diff.x / len) * new_n_vel,
                                         (diff.y / len) * new_n_vel,
                                         (diff.z / len) * new_n_vel);
    // making sure not to overshoot
    if (CalculateDistance(new_pos, position) >
        CalculateDistance(position, m_position))
    {
      new_pos = position;
      return;
    }

    // set new node position for a smoother movement
    auto mob = drone->GetObject<MobilityModel>();
    mob->SetPosition(new_pos);

    Simulator::Schedule(Seconds(interval), &move_drones, drone, position,
                        n_vel);
    return;
  }
  LOG("drone arrived at " << Simulator::Now().GetSeconds());
}

int get_user_id_from_ipv4(Ipv4Address ip)
{
  for (auto &u : user_ip)
  {
    if (u.second == ip)
      return u.first;
  }
  return -1;
}

void ThroughputMonitor(FlowMonitorHelper *fmhelper, Ptr<FlowMonitor> flowMon)
{
  flowMon->CheckForLostPackets();
  uint32_t LostPacketsum = 0;
  float PDR, PLR, Delay, Throughput;
  auto flowStats = flowMon->GetFlowStats();
  Ptr<Ipv4FlowClassifier> classing =
      DynamicCast<Ipv4FlowClassifier>(fmhelper->GetClassifier());
  std::ofstream qos_file;
  qos_file.open("qos.txt", std::ofstream::out | std::ofstream::trunc);

  for (auto stats : flowStats)
  {
    Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow(stats.first);
    PDR = (double)(100 * stats.second.rxPackets) / (stats.second.txPackets);
    LostPacketsum = (double)(stats.second.txPackets) - (stats.second.rxPackets);
    PLR = (double)(LostPacketsum * 100) / stats.second.txPackets;
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
    LOG("target node = " << get_user_id_from_ipv4(
            fiveTuple.destinationAddress));
    if (get_user_id_from_ipv4(fiveTuple.destinationAddress) != -1)
      cell_throughput[get_user_id_from_ipv4(fiveTuple.destinationAddress)] +=
          Throughput;
  }

  qos_file.close();

  Simulator::Schedule(Seconds(1), &ThroughputMonitor, fmhelper, flowMon);
}

void request_video(Ptr<Node> sender_node, Ptr<Node> receiver_node,
                   Ipv4Address targetServerAddress)
{
  static uint16_t m_port = 2000;
  static int request_id = 0;

  EvalvidServerHelper server(m_port);
  server.SetAttribute("SenderTraceFilename", StringValue("st_highway_cif.st"));

  server.SetAttribute("SenderDumpFilename",
                      StringValue("sd_" + std::to_string(request_id) + ".txt"));
  // server.SetAttribute("PacketPayload", UintegerValue(512));

  ApplicationContainer apps = server.Install(sender_node);
  apps.Start(Seconds(0));

  EvalvidClientHelper client(targetServerAddress, m_port);
  client.SetAttribute("ReceiverDumpFilename",
                      StringValue("rd_" + std::to_string(request_id) + ".txt"));
  apps = client.Install(receiver_node);
  apps.Start(Seconds(0));

  request_id++;
  m_port++;
}

void UDPApp(Ptr<Node> remoteHost, NodeContainer ueNodes)
{
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

  for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
  {
    Ptr<Node> ue = ueNodes.Get(u);
    Ptr<Ipv4> ueIpv4 = ue->GetObject<Ipv4>();
    Ipv4Address ueIpAddr = ueIpv4->GetAddress(1, 0).GetLocal();
    ulPort++;

    if (!disableDl)
    {
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

    if (!disableUl)
    {
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

bool IsTopLevelSourceDir(std::string path)
{
  bool haveVersion = false;
  bool haveLicense = false;

  //
  // If there's a file named VERSION and a file named LICENSE in this
  // directory, we assume it's our top level source directory.
  //

  std::list<std::string> files = SystemPath::ReadFiles(path);
  for (std::list<std::string>::const_iterator i = files.begin();
       i != files.end(); ++i)
  {
    if (*i == "VERSION")
    {
      haveVersion = true;
    }
    else if (*i == "LICENSE")
    {
      haveLicense = true;
    }
  }

  return haveVersion && haveLicense;
}

std::string GetTopLevelSourceDir(void)
{
  std::string self = SystemPath::FindSelfDirectory();
  std::list<std::string> elements = SystemPath::Split(self);
  while (!elements.empty())
  {
    std::string path = SystemPath::Join(elements.begin(), elements.end());
    if (IsTopLevelSourceDir(path))
    {
      return path;
    }
    elements.pop_back();
  }
  NS_FATAL_ERROR("Could not find source directory from self=" << self);
}

std::vector<std::pair<int, int>> create_hot_spots()
{
  std::vector<std::pair<int, int>> centers;
  std::uniform_int_distribution<int> distribution(0, 2000);

  for (uint32_t i = 0; i < number_of_surges; i++)
  {
    std::pair<int, int> center;
    center.first = distribution(generator);
    center.second = distribution(generator);
    LOG("Adding centroid to " << center.first << " " << center.second);
    centers.push_back(center);
  }

  return centers;
}

int get_closest_center_index(Ptr<Node> node,
                             std::vector<std::pair<int, int>> centers)
{
  Vector m_position = get_node_position(node);
  double dist = INT_MAX;
  int closest = -1;
  for (uint32_t i = 0; i < number_of_surges; i++)
  {
    if (dist > CalculateDistance(m_position, Vector3D(centers[i].first,
                                                      centers[i].second, 1)))
    {
      closest = i;
    }
  }
  return closest;
}

void generate_requests(Ptr<Node> remoteHost,
                       std::vector<std::pair<int, int>> centers)
{
  // at 978 user 99 requests 1043.98848 bytes
  std::ifstream trace_file(requests_trace);
  // int currentTime = Simulator::Now().GetSeconds();
  std::unordered_map<int, double> requests;
  std::string str;

  for (uint32_t i = 0; i < ueNodes.GetN(); i++)
  {
    int serving_node = 0;
    double max_payload = 10 * 1024; // request 2mb for every user and see behavior

    // get distance to closest hot spot and calculate payload
    int index_to_closest = get_closest_center_index(ueNodes.Get(i), centers);
    std::pair<int, int> closest_surge = centers[index_to_closest];
    Vector3D surge(closest_surge.first, closest_surge.second, 1);

    // payload follows an exponential function with max when dist to hot spot =
    // 0
    Vector node_position = get_node_position(ueNodes.Get(i));
    double dist = CalculateDistance(surge, node_position);
    int payload = max_payload * exp((dist * -1) / 1000);

    if (payload)
    {
      LOG("requesting app from user "
          << i << " to server " << serving_node << " at address "
          << serverNodesAddresses[serving_node][1] << " with payload "
          << payload);

      requestApplication(remoteHost, ueNodes.Get(i), payload);
    }
  }

  trace_file.close();
  Simulator::Schedule(Seconds(1), &generate_requests, remoteHost, centers);
}

int main(int argc, char *argv[])
{
  // LogComponentEnable("Config", LOG_LEVEL_ALL);
  LogComponentEnable("EvalvidClient", LOG_LEVEL_ALL);
  LogComponentEnable("EvalvidServer", LOG_LEVEL_ALL);

  CommandLine cmd;

  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults();

  cmd.Parse(argc, argv);

  ns3::RngSeedManager::SetSeed(seedValue);
  ns3_dir = GetTopLevelSourceDir();

  // create epc helper and set as default
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
  lteHelper->SetEpcHelper(epcHelper);
  // get pgw node
  Ptr<Node> pgw = epcHelper->GetPgwNode();

  // lte specific config
  lteHelper->SetAttribute("PathlossModel",
                          StringValue("ns3::NakagamiPropagationLossModel"));
  lteHelper->SetHandoverAlgorithmType("ns3::NoOpHandoverAlgorithm");
  // todo: change for uavs
  lteHelper->SetEnbDeviceAttribute("DlBandwidth",
                                   UintegerValue(25)); // Set Download BandWidth
  lteHelper->SetEnbDeviceAttribute("UlBandwidth",
                                   UintegerValue(25)); // Set Upload Bandwidth

  // Network config
  if (useCa)
  {
    Config::SetDefault("ns3::LteHelper::UseCa", BooleanValue(useCa));
    Config::SetDefault("ns3::LteHelper::NumberOfComponentCarriers",
                       UintegerValue(2));
    Config::SetDefault("ns3::LteHelper::EnbComponentCarrierManager",
                       StringValue("ns3::RrComponentCarrierManager"));
  }

  // Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(eNodeBTxPower));
  Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue(320));
  // error modes for ctrl and data planes
  Config::SetDefault("ns3::LteSpectrumPhy::CtrlErrorModelEnabled",
                     BooleanValue(false));
  Config::SetDefault("ns3::LteSpectrumPhy::DataErrorModelEnabled",
                     BooleanValue(false));

  // create nodes in global containers
  UAVNodes.Create(numUAVs);
  ueNodes.Create(numUes);
  BSNodes.Create(numStaticCells);
  serverNodes.Create(numStaticCells);

  // create and install internet
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create(1);
  Ptr<Node> remoteHost = remoteHostContainer.Get(0);
  InternetStackHelper internet;
  internet.Install(remoteHost);

  // Create the Internet
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
  p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
  p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010))); // 0.010
  NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
  // interface 0 is localhost, 1 is the p2p device
  // Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);

  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
      ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
  remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
                                             Ipv4Mask("255.0.0.0"), 1);

  // set up mobility
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  // mobility.Install(remoteHost);
  mobility.Install(pgw);
  mobility.Install(remoteHostContainer);

  /*user device mobility according to ns2 trace*/
  Ns2MobilityHelper ped_mobil = Ns2MobilityHelper(mobil_trace);
  ped_mobil.Install(ueNodes.Begin(), ueNodes.End());

  MobilityHelper mobilityEnb;
  mobilityEnb.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  auto BSPosition = generatePositionAllocator(numStaticCells, 2000, "koln");
  mobilityEnb.SetPositionAllocator(BSPosition);
  mobilityEnb.Install(BSNodes);

  mobilityEnb.Install(serverNodes);

  /*reassign for uav random positioning*/
  MobilityHelper mobilityUAV;
  auto UAVPosition = generatePositionAllocator(numUAVs, 1000, "random");
  mobilityUAV.SetMobilityModel("ns3::WaypointMobilityModel");
  mobilityUAV.SetPositionAllocator(UAVPosition);
  mobilityUAV.Install(UAVNodes);

  enbDevs = lteHelper->InstallEnbDevice(NodeContainer(BSNodes, UAVNodes));
  ueDevs = lteHelper->InstallUeDevice(ueNodes);

  // set up different transmission powers for drones
  for (uint32_t i = 0; (unsigned)i < enbDevs.GetN(); i++)
  {
    auto enb0Phy = enbDevs.Get(i)->GetObject<LteEnbNetDevice>()->GetPhy();
    if (i < numStaticCells)
    {
      enb0Phy->SetTxPower(43);
    }
    else
    {
      enb0Phy->SetTxPower(23);
    }
  }

  // set default gateway for users
  // Install the IP stack on the UEs
  internet.Install(ueNodes);
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));
  // Assign IP address to UEs, and install applications
  for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
  {
    Ptr<Node> ueNode = ueNodes.Get(u);
    // Set the default gateway for the UE
    Ptr<Ipv4StaticRouting> ueStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
    ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(),
                                     1);
  }

  // attach to cells with the highest sinr
  lteHelper->Attach(ueDevs);
  lteHelper->AddX2Interface(BSNodes);

  // populate user ip map
  for (uint32_t i = 0; i < ueNodes.GetN(); i++)
  {
    Ptr<Ipv4> remoteIpv4 = ueNodes.Get(i)->GetObject<Ipv4>();
    Ipv4Address remoteIpAddr = remoteIpv4->GetAddress(1, 0).GetLocal();
    user_ip[i] = remoteIpAddr;
  }

  AnimationInterface animator("lte_animation.xml");
  // animator.SetMobilityPollInterval(Seconds(1));
  for (uint32_t i = 0; i < UAVNodes.GetN(); ++i)
  {
    animator.UpdateNodeDescription(UAVNodes.Get(i), "UAV " + std::to_string(i));
    animator.UpdateNodeColor(UAVNodes.Get(i), 250, 200, 45);
    animator.UpdateNodeSize(UAVNodes.Get(i)->GetId(), 10,
                            10); // to change the node size in the animation.
  }

  for (uint32_t j = 0; j < ueNodes.GetN(); ++j)
  {
    animator.UpdateNodeDescription(ueNodes.Get(j), "UE " + std::to_string(j));
    animator.UpdateNodeColor(ueNodes.Get(j), 20, 10, 145);
    animator.UpdateNodeSize(ueNodes.Get(j)->GetId(), 10, 10);
  }

  for (uint32_t j = 0; j < BSNodes.GetN(); ++j)
  {
    animator.UpdateNodeDescription(BSNodes.Get(j), "Cell " + std::to_string(j));
    animator.UpdateNodeColor(BSNodes.Get(j), 20, 10, 145);
    animator.UpdateNodeSize(BSNodes.Get(j)->GetId(), 10, 10);
  }

  // intall flow monitor and get stats
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();
  Ptr<Ipv4FlowClassifier> classifier =
      DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());

  // populate centroids
  auto centers = create_hot_spots();

  /* all scheduled functions*/
  Simulator::Schedule(Seconds(1), &ThroughputMonitor, &flowmon, monitor);
  Simulator::Schedule(Seconds(1), &UAVManager);
  Simulator::Schedule(Seconds(1), &generate_requests, remoteHost, centers);
  for (uint32_t i = 0; i < UAVNodes.GetN(); i++)
    Simulator::Schedule(Seconds(2), &move_drones, UAVNodes.Get(i),
                        Vector(1000, 1000, 20), 100);

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

  Simulator::Stop(Seconds(SimTime));
  Simulator::Run();

  monitor->CheckForLostPackets();
  FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
  for (map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin();
       i != stats.end(); ++i)
  {
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

  return 0;
}
