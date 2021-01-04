#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"

#include <ns3/internet-stack-helper.h>
#include <ns3/ipv4-address-helper.h>
#include <ns3/ipv4-static-routing-helper.h>
#include <ns3/lte-helper.h>
#include <ns3/mobility-helper.h>
#include <ns3/point-to-point-epc-helper.h>
#include <ns3/point-to-point-helper.h>

#include <ns3/ipv4-interface-container.h>
#include <ns3/net-device-container.h>
#include <ns3/node-container.h>

#define LOG(x) std::cout << x << std::endl

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ConnectionManagerMeas");

bool verbose = true;

void connectionManagerCallback() {}

void ReportUeMeasurementsCallback(std::string path, uint16_t rnti,
                                  uint16_t cellId, double rsrp, double rsrq,
                                  bool servingCell,
                                  uint8_t componentCarrierId) {
  if (verbose) {
    LOG("Simulation time: " << Simulator::Now().GetSeconds());
    LOG("rnti " << rnti);
    LOG("cellid" << cellId);
    LOG("rsrp " << rsrp);
    LOG("rsrq " << rsrq);
    LOG("serving cell " << servingCell);
    LOG("cc id " << (int)componentCarrierId);
    LOG("\n");
  }
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

int main() {

  Config::SetDefault("ns3::LteSpectrumPhy::CtrlErrorModelEnabled",
                     BooleanValue(false));
  Config::SetDefault("ns3::LteSpectrumPhy::DataErrorModelEnabled",
                     BooleanValue(false));
  Config::SetDefault("ns3::LteAmc::AmcModel", EnumValue(LteAmc::PiroEW2010));
  Config::SetDefault("ns3::LteAmc::Ber", DoubleValue(0.00005));
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
  lteHelper->SetAttribute(
      "PathlossModel", StringValue("ns3::FriisSpectrumPropagationLossModel"));
  lteHelper->SetAttribute("UseIdealRrc", BooleanValue(false));
  // lteHelper->EnableTraces();

  // Disable Uplink Power Control
  Config::SetDefault("ns3::LteUePhy::EnableUplinkPowerControl",
                     BooleanValue(false));

  // LogComponentEnable ("LteUeMeasurementsTest", LOG_LEVEL_ALL);

  // Create Nodes: eNodeB and UE
  NodeContainer enbNodes;
  NodeContainer ueNodes1;
  NodeContainer ueNodes2;
  enbNodes.Create(2);
  ueNodes1.Create(1);
  ueNodes2.Create(1);
  NodeContainer allNodes = NodeContainer(enbNodes, ueNodes1, ueNodes2);

  // the topology is the following:
  //         d2
  //  UE1-----------eNB2
  //   |             |
  // d1|             |d1
  //   |     d2      |
  //  eNB1----------UE2
  //
  Ptr<ListPositionAllocator> positionAlloc =
      CreateObject<ListPositionAllocator>();
  positionAlloc->Add(Vector(0.0, 0.0, 0.0)); // eNB1
  positionAlloc->Add(Vector(100, 100, 0.0)); // eNB2
  positionAlloc->Add(Vector(0.0, 100, 0.0)); // UE1
  positionAlloc->Add(Vector(100, 0.0, 0.0)); // UE2
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator(positionAlloc);
  mobility.Install(allNodes);

  // Create Devices and install them in the Nodes (eNB and UE)
  NetDeviceContainer enbDevs;
  NetDeviceContainer ueDevs1;
  NetDeviceContainer ueDevs2;
  lteHelper->SetSchedulerType("ns3::RrFfMacScheduler");
  lteHelper->SetSchedulerAttribute("UlCqiFilter",
                                   EnumValue(FfMacScheduler::PUSCH_UL_CQI));
  enbDevs = lteHelper->InstallEnbDevice(enbNodes);
  ueDevs1 = lteHelper->InstallUeDevice(ueNodes1);
  ueDevs2 = lteHelper->InstallUeDevice(ueNodes2);

  // Attach UEs to eNodeBs
  lteHelper->Attach(ueDevs1, enbDevs.Get(0));
  lteHelper->Attach(ueDevs2, enbDevs.Get(1));

  // Activate an EPS bearer
  enum EpsBearer::Qci q = EpsBearer::GBR_CONV_VOICE;
  EpsBearer bearer(q);
  lteHelper->ActivateDataRadioBearer(ueDevs1, bearer);
  lteHelper->ActivateDataRadioBearer(ueDevs2, bearer);

  Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/LteUePhy/"
                  "ReportUeMeasurements",
                  MakeCallback(&ReportUeMeasurementsCallback));
  Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/RecvMeasurementReport",
                  MakeCallback(&RecvMeasurementReportCallback));

  lteHelper->EnablePhyTraces();
  lteHelper->EnableMacTraces();
  lteHelper->EnableRlcTraces();
  lteHelper->EnablePdcpTraces();

  // need to allow for RRC connection establishment + SRS
  Simulator::Stop(Seconds(20));
  Simulator::Run();

  Simulator::Destroy();
}
