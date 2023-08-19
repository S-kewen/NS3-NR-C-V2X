/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 *   Copyright (c) 2020 University of Padova, Dep. of Information Engineering,
 *   SIGNET lab.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation;
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "ns3/mmwave-sidelink-spectrum-phy.h"
#include "ns3/mmwave-vehicular-net-device.h"
#include "ns3/mmwave-vehicular-helper.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/mobility-module.h"
#include "ns3/config.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/spectrum-helper.h"
#include "ns3/mmwave-spectrum-value-helper.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"
#include "ns3/buildings-module.h"
#include "ns3/command-line.h"
#include "ns3/node-list.h"
#include "ns3/packet.h"
#include "ns3/ethernet-header.h"
#include <random>

NS_LOG_COMPONENT_DEFINE("VehicularSimpleOne");

using namespace ns3;
using namespace millicar;

void PrintGnuplottableNodeListToFile(std::string filename);

uint32_t g_rxPackets; // total number of received packets
uint32_t g_txPackets; // total number of transmitted packets

Time g_firstReceived; // timestamp of the first time a packet is received
Time g_lastReceived;  // timestamp of the last received packet

static void Rx(Ptr<OutputStreamWrapper> stream, Ptr<const Packet> p)
{
  g_rxPackets++;
  SeqTsHeader header;
  Ipv4Header ipv4Header;
  // if (p->PeekHeader(header))
  // {
  //   std::cout << "uid" << (uint32_t)p->GetUid() << std::endl;
  // }
  // p->PeekHeader(ipv4Header);

  // ipv4Header.Print(std::cout);

  // std::cout << "source: " << ipv4Header.GetSource() << "destination: " << ipv4Header.GetDestination() << std::endl;

  // UdpHeader udpHeader;
  // p->PeekHeader(udpHeader);
  // std::cout << "source port: " << udpHeader.GetSourcePort() << "destination port: " << udpHeader.GetDestinationPort() << std::endl;

  *stream->GetStream() << Simulator::Now().GetSeconds() << "\t" << p->GetSize() << "\t" << header.GetSeq() << "\t" << header.GetTs().GetSeconds() << std::endl;

  if (g_rxPackets > 1)
  {

    g_lastReceived = Simulator::Now();
  }
  else
  {
    g_firstReceived = Simulator::Now();
  }
}

int main(int argc, char *argv[])
{
  // This script creates two nodes moving at 20 m/s, placed at a distance of 10 m.
  // These nodes exchange packets through a UDP application,
  // and they communicate using a wireless channel.

  // system parameters
  double bandwidth = 1e8;  // bandwidth in Hz
  double frequency = 28e9; // the carrier frequency
  uint32_t numerology = 3; // the numerology

  // applications
  uint32_t packetSize = 1024;        // UDP packet size in bytes
  uint32_t startTime = 50;           // application start time in milliseconds
  uint32_t endTime = 2000;           // application end time in milliseconds
  uint32_t interPacketInterval = 30; // interpacket interval in microseconds

  std::string scenario = "V2V-Urban";

  Config::SetDefault("ns3::MmWaveSidelinkMac::UseAmc", BooleanValue(true));
  Config::SetDefault("ns3::MmWaveSidelinkMac::Mcs", UintegerValue(28));
  Config::SetDefault("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue(frequency));

  Config::SetDefault("ns3::MmWaveVehicularHelper::Bandwidth", DoubleValue(bandwidth));
  Config::SetDefault("ns3::MmWaveVehicularHelper::Numerology", UintegerValue(numerology));
  Config::SetDefault("ns3::MmWaveVehicularHelper::ChannelModelType", StringValue(scenario));

  Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(10)));
  Config::SetDefault("ns3::ThreeGppChannelConditionModel::UpdatePeriod", TimeValue(MilliSeconds(10)));

  Config::SetDefault("ns3::MmWaveVehicularNetDevice::RlcType", StringValue("LteRlcUm"));
  Config::SetDefault("ns3::MmWaveVehicularHelper::SchedulingPatternOption", EnumValue(2)); // use 2 for SchedulingPatternOption=OPTIMIZED, 1 or SchedulingPatternOption=DEFAULT
  Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(500 * 1024));

  // create the nodes
  NodeContainer gContainer;
  gContainer.Create(2);
  // create the mobility models
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(gContainer);

  for (int i = 0; i < 2; i++)
  {
    gContainer.Get(i)->GetObject<MobilityModel>()->SetPosition(Vector(0, i, 0));
  }

  // create and configure the helper
  Ptr<MmWaveVehicularHelper> helper = CreateObject<MmWaveVehicularHelper>();
  helper->SetNumerology(3);
  NetDeviceContainer devices = helper->InstallMmWaveVehicularNetDevices(gContainer);

  // Install the TCP/IP stack in the two nodes
  InternetStackHelper internet;
  internet.Install(gContainer);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO("Assign IP Addresses.");
  ipv4.SetBase("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign(devices);

  // Mandatory to install buildings helper even if there are no buildings,
  // otherwise V2V-Urban scenario does not work
  BuildingsHelper::Install(gContainer);

  // Need to pair the devices in order to create a correspondence between transmitter and receiver
  // and to populate the < IP addr, RNTI > map.
  helper->PairDevices(devices); // [skewen]: going to pair the devices for send the packets

  // Set the routing table
  // Ipv4StaticRoutingHelper ipv4RoutingHelper;
  // Ptr<Ipv4StaticRouting> staticRouting = ipv4RoutingHelper.GetStaticRouting(gContainer.Get(0)->GetObject<Ipv4>());
  // staticRouting->SetDefaultRoute(gContainer.Get(1)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal(), 2);

  NS_LOG_DEBUG("IPv4 Address node 0: " << gContainer.Get(0)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal());
  NS_LOG_DEBUG("IPv4 Address node 1: " << gContainer.Get(1)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal());

  Ptr<mmwave::MmWaveAmc> m_amc = CreateObject<mmwave::MmWaveAmc>(helper->GetConfigurationParameters());

  // setup the applications
  Config::SetDefault("ns3::UdpClient::MaxPackets", UintegerValue(0xFFFFFFFF));
  Config::SetDefault("ns3::UdpClient::Interval", TimeValue(MicroSeconds(interPacketInterval)));
  Config::SetDefault("ns3::UdpClient::PacketSize", UintegerValue(packetSize));

  Ptr<NetDevice> d0 = devices.Get(0);
  Ptr<MmWaveVehicularNetDevice> wd = DynamicCast<MmWaveVehicularNetDevice>(d0); // client

  Ipv4Address address = gContainer.Get(1)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal(); // server address

  for (int i = 0; i < 1000; i++)
  {
    std::random_device rd;                      // 使用硬件随机数生成器生成种子
    std::mt19937 gen(rd());                     // 使用 Mersenne Twister 引擎
    std::uniform_int_distribution<> dis(1, 2000); // 生成 1 到 10 之间的均匀分布整数

    int packetSize = dis(gen);

    packetSize = 8;

    Ptr<Packet> p = Create<Packet>(packetSize);
    UdpHeader udpHeader;
    udpHeader.SetSourcePort(12354);
    udpHeader.SetDestinationPort(4000);

    Ipv4Header ipv4Header;
    ipv4Header.SetSource(gContainer.Get(0)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal());
    ipv4Header.SetDestination(gContainer.Get(1)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal());
    ipv4Header.SetProtocol(17);
    ipv4Header.SetPayloadSize(packetSize);

    EthernetHeader ethernetHeader;
    ethernetHeader.SetSource(Mac48Address("00:00:00:00:00:01"));
    ethernetHeader.SetDestination(Mac48Address("00:00:00:00:00:02"));
    // ethernetHeader.SetType (Ethertype::IPv4);

    p->AddHeader(ethernetHeader);
    p->AddHeader(udpHeader);
    p->AddHeader(ipv4Header);
    Simulator::Schedule(Seconds(i * 0.001), &MmWaveVehicularNetDevice::Send, wd, p->Copy(), address, Ipv4L3Protocol::PROT_NUMBER);
  }

  // Simulator::Schedule(Seconds(1.1), &MmWaveVehicularNetDevice::Send, wd, p->Copy(), address, Ipv4L3Protocol::PROT_NUMBER);
  // create the applications
  uint32_t port = 4000;

  UdpEchoServerHelper server(port);
  ApplicationContainer echoApps = server.Install(gContainer.Get(1));
  echoApps.Start(Seconds(0.0));

  AsciiTraceHelper asciiTraceHelper;
  Ptr<OutputStreamWrapper> stream = asciiTraceHelper.CreateFileStream("simple-one-stats.txt");
  echoApps.Get(0)->TraceConnectWithoutContext("Rx", MakeBoundCallback(&Rx, stream));

  // UdpClientHelper client(gContainer.Get(1)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal(), port);
  // ApplicationContainer apps = client.Install(gContainer.Get(0));

  // // set the application start/end time
  // apps.Start(MilliSeconds(startTime));
  // apps.Stop(MilliSeconds(endTime));

  PrintGnuplottableNodeListToFile("scenario.txt");

  Simulator::Stop(MilliSeconds(endTime + 1000));
  Simulator::Run();
  Simulator::Destroy();

  std::cout << "----------- Statistics -----------" << std::endl;
  std::cout << "Packets size:\t\t" << packetSize << " Bytes" << std::endl;
  std::cout << "Packets received:\t" << g_rxPackets << std::endl;
  std::cout << "Average Throughput:\t" << (double(g_rxPackets) * (double(packetSize) * 8) / double(g_lastReceived.GetSeconds() - g_firstReceived.GetSeconds())) / 1e6 << " Mbps" << std::endl;

  return 0;
}

void PrintGnuplottableNodeListToFile(std::string filename)
{
  std::ofstream outFile;
  outFile.open(filename.c_str(), std::ios_base::out | std::ios_base::trunc);
  if (!outFile.is_open())
  {
    NS_LOG_ERROR("Can't open file " << filename);
    return;
  }
  outFile << "set xrange [-200:200]; set yrange [-200:200]" << std::endl;
  for (NodeList::Iterator it = NodeList::Begin(); it != NodeList::End(); ++it)
  {
    Ptr<Node> node = *it;
    int nDevs = node->GetNDevices();
    for (int j = 0; j < nDevs; j++)
    {
      Ptr<MmWaveVehicularNetDevice> vdev = node->GetDevice(j)->GetObject<MmWaveVehicularNetDevice>();
      if (vdev)
      {
        Vector pos = node->GetObject<MobilityModel>()->GetPosition();
        outFile << "set label \"" << vdev->GetMac()->GetRnti()
                << "\" at " << pos.x << "," << pos.y << " left font \"Helvetica,8\" textcolor rgb \"black\" front point pt 7 ps 0.3 lc rgb \"black\" offset 0,0"
                << std::endl;

        // Simulator::Schedule (Seconds (1), &PrintHelper::UpdateGnuplottableNodeListToFile, filename, node);
      }
    }
  }

  uint32_t index = 0;
  for (BuildingList::Iterator it = BuildingList::Begin(); it != BuildingList::End(); ++it)
  {
    ++index;
    Box box = (*it)->GetBoundaries();
    outFile << "set object " << index
            << " rect from " << box.xMin << "," << box.yMin
            << " to " << box.xMax << "," << box.yMax
            << std::endl;
  }
}
