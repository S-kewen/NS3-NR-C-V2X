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
#include "zmq.hpp"
#include <map>
#include <time.h>
#include <json/json.h>
#include <iostream>
#include <set>

NS_LOG_COMPONENT_DEFINE("VehicularSimpleOne");
using namespace std;
using namespace ns3;
using namespace millicar;

#define PROTOCOL_UDP 17
#define PROTOCOL_TCP 6

struct Packetitem
{
  int fromDev = -1;
  int toDev = -1;
  int frame = -1;
  int segment = -1;
  double sendT = -1.0;
  double recvT = -1;
  double x = -1;
  double y = -1;
  double z = -1;
  double rx = -1;
  double ry = -1;
  double rz = -1;
  int type = -1;
  std::string messageType = "unknown";
};

NodeContainer gContainer;
NetDeviceContainer devices;

double m_simTime = 0;

double m_syncTime = 0.5;

int m_nodes = 0;

std::time_t expStartTime = 0;

vector<string> split(const string &str, const string &delim)
{
  vector<string> res;
  if ("" == str)
    return res;
  char *strs = new char[str.length() + 1];
  strcpy(strs, str.c_str());

  char *d = new char[delim.length() + 1];
  strcpy(d, delim.c_str());

  char *p = strtok(strs, d);
  while (p)
  {
    string s = p;
    res.push_back(s);
    p = strtok(NULL, d);
  }

  return res;
}

static void Rx(zmq::socket_t *pkgSendSocket, std::map<uint64_t, Packetitem> *umap, std::set<uint32_t> *receivedUids, Ptr<const Packet> packet)
{
  SeqTsHeader hdr;
  uint32_t uid = packet->GetUid();
  if (packet->PeekHeader(hdr))
  {
    char str[1000];
    Packetitem pt = umap->operator[](uid);
    if (pt.type > 0 && receivedUids->find(uid) == receivedUids->end())
    {
      receivedUids->insert(uid);
      // std::cout << "send time: " << pt.sendT << " receive time: " << Simulator::Now().GetSeconds() << endl;
      sprintf(str, "{\"type\":\"rx\",\"messageType\":\"%s\",\"sendTime\":%lf,\"recvTime\":%lf,\"packetuid\":%lu,\"size\":%d,\"receiveMac\":\"%d\",\"sourceMac\":\"%d\",\"frame\":\"%d\",\"segment\":\"%d\",\"x\":%lf,\"y\":%lf,\"z\":%lf,\"rx\":%lf,\"ry\":%lf,\"rz\":%lf,\"id\":%d}", pt.messageType.c_str(), pt.sendT, Simulator::Now().GetSeconds(), packet->GetUid(), packet->GetSize(), pt.toDev, pt.fromDev, pt.frame, pt.segment, pt.x, pt.y, pt.z, pt.rx, pt.ry, pt.rz, pt.fromDev);
      std::string s = string(str);
      zmq::message_t message(s.size());
      memcpy((uint8_t *)(message.data()), s.data(), s.size());
      pkgSendSocket->send(message, zmq::send_flags::none);
      std::cout << str << endl;
    }
  }
}

void zmq_simulation(NetDeviceContainer ds, zmq::socket_t *zmqRecvSocket, std::map<uint64_t, Packetitem> *umap)
{

  zmq::message_t message;
  Json::Reader reader;
  Json::Value json_object;
  while (true)
  {
    zmqRecvSocket->recv(message, zmq::recv_flags::none);
    std::string json_data(static_cast<char *>(message.data()), message.size());
    if (reader.parse(json_data, json_object))
    {
      if (json_object["type"].asString() == "updateStatus" && json_object["status"].asInt() == 0)
      {
        break;
      }
      else if (json_object["type"].asString() == "pushEvent")
      {
        // std::cout << json_object << std::endl;
        double sendTime = json_object["timestamp"].asDouble();
        double scheduleTime = sendTime - m_simTime;

        Ptr<NetDevice> d0 = ds.Get(json_object["id"].asInt());
        Ptr<MmWaveVehicularNetDevice> wd = DynamicCast<MmWaveVehicularNetDevice>(d0);

        Ptr<Packet> p = Create<Packet>(json_object["size"].asInt());
        Ipv4Address receiverAddress = gContainer.Get(json_object["receiverId"].asInt())->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();

        // The destination address must be the serverId
        Ipv4Header ipv4Header;
        ipv4Header.SetSource(gContainer.Get(json_object["id"].asInt())->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal());
        ipv4Header.SetDestination(gContainer.Get(json_object["receiverId"].asInt())->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal());
        ipv4Header.SetProtocol(PROTOCOL_UDP);
        ipv4Header.SetPayloadSize(json_object["size"].asInt());

        UdpHeader udpHeader;
        udpHeader.SetSourcePort(12345);
        udpHeader.SetDestinationPort(8888);

        p->AddHeader(udpHeader);
        p->AddHeader(ipv4Header);

        Packetitem pt;
        pt.type = 1;
        pt.frame = json_object["frame"].asInt();
        pt.segment = json_object["segment"].asInt();
        pt.fromDev = json_object["id"].asInt();
        pt.toDev = json_object["receiverId"].asInt();
        pt.x = json_object["x"].asDouble();
        pt.y = json_object["y"].asDouble();
        pt.z = json_object["z"].asDouble();
        pt.rx = json_object["rx"].asDouble();
        pt.ry = json_object["ry"].asDouble();
        pt.rz = json_object["rz"].asDouble();
        pt.sendT = sendTime;
        pt.messageType = json_object["messageType"].asString();
        umap->operator[](p->GetUid()) = pt;
        
        // Mac48Address::GetBroadcast()
        
        Simulator::Schedule(Seconds(scheduleTime), &MmWaveVehicularNetDevice::Send, wd, p->Copy(), receiverAddress, Ipv4L3Protocol::PROT_NUMBER);
        // 0x88dc is the ethertype corresponding to WSMP. IPv4's etherType is 0x0800, and IPv6 is 0x86DD
      }
    }
  }
}

void zmq_update_position(zmq::socket_t *zmqRecvSocket)
{
  zmq::message_t message;
  Json::Reader reader;
  Json::Value json_object;
  while (true)
  {
    zmqRecvSocket->recv(message, zmq::recv_flags::dontwait);
    std::string json_data(static_cast<char *>(message.data()), message.size());
    if (reader.parse(json_data, json_object))
    {
      if (json_object["type"].asString() == "updateStatus" && json_object["status"].asInt() == 0)
      {
        break;
      }
      else if (json_object["type"].asString() == "setPosition")
      {
        Ptr<MobilityModel> wnd = DynamicCast<MobilityModel>(gContainer.Get(json_object["id"].asInt())->GetObject<MobilityModel>());

        double time = json_object["timestamp"].asDouble() - m_simTime;
        if (time < 0)
        {
          std::cout << "error!!!!" << json_object["timestamp"].asDouble() << "-" << m_simTime << "=" << time << endl;
        }

        Simulator::Schedule(Seconds(time), &ConstantPositionMobilityModel::SetPosition, wnd, Vector(Vector(json_object["x"].asDouble(), json_object["y"].asDouble(), json_object["z"].asDouble())));
      }
    }
  }
}

void takeTurn(NetDeviceContainer devices, zmq::socket_t *pkgRecvSocket, zmq::socket_t *posRecvSocket, zmq::socket_t *rsmSendSocket, std::map<uint64_t, Packetitem> *umap, double syncTime)
{
  std::cout << "start receive zmq_simulation" << std::endl;
  zmq_simulation(devices, pkgRecvSocket, umap);
  std::cout << "start receive zmq_update_position" << std::endl;
  zmq_update_position(posRecvSocket);
  m_simTime += syncTime;

  char str[1000];
  sprintf(str, "{\"type\":\"resume\",\"timestamp\":%lf,\"status\":1}", Simulator::Now().GetSeconds());
  std::string s = string(str);
  zmq::message_t message(s.size());
  memcpy((uint8_t *)(message.data()), s.data(), s.size());
  rsmSendSocket->send(message, zmq::send_flags::none);
  Simulator::Schedule(Seconds(syncTime), &takeTurn, devices, pkgRecvSocket, posRecvSocket, rsmSendSocket, umap, syncTime);
  std::cout << "m_simTime: " << m_simTime << std::endl;
}

void PhyRxDrop(zmq::socket_t *pkgSendSocket, std::map<uint64_t, Packetitem> *umap, Ptr<const Packet> packet)
{
  std::cout << "PhyRxDrop" << std::endl;
}

void PhyTxDrop(zmq::socket_t *pkgSendSocket, std::map<uint64_t, Packetitem> *umap, Ptr<const Packet> packet)
{
  std::cout << "PhyTxDrop" << std::endl;
}

void MacRxDrop(zmq::socket_t *pkgSendSocket, std::map<uint64_t, Packetitem> *umap, Ptr<const Packet> packet)
{
  std::cout << "MacRxDrop" << std::endl;
}

void MacTxDrop(zmq::socket_t *pkgSendSocket, std::map<uint64_t, Packetitem> *umap, Ptr<const Packet> packet)
{
  std::cout << "MacTxDrop" << std::endl;
}

int main(int argc, char *argv[])
{
  int zmqPort = 5557;
  uint32_t serverId = -1;
  CommandLine cmd;
  cmd.AddValue("nodes", "nodes number", m_nodes);
  cmd.AddValue("syncTime", "sync time", m_syncTime);
  cmd.AddValue("serverId", "edge server id", serverId);
  cmd.AddValue("zmqPort", "zmq port", zmqPort);
  cmd.Parse(argc, argv);
  
  if ((int) serverId == -1){
    serverId = m_nodes - 1;
  }

  // m_nodes = 4;
  // m_syncTime = 0.5;
  // start by 0

  std::cout << "m_nodes: " << m_nodes << std::endl;
  std::cout << "m_syncTime: " << m_syncTime << std::endl;
  std::cout << "serverId: " << serverId << std::endl;
  std::cout << "zmqPort: " << zmqPort << std::endl;



  if (m_nodes <= 0)
  {
    std::cout << "Nodes parms error !!!" << std::endl;
    return 0;
  }

  if (m_syncTime <= 0)
  {
    std::cout << "SyncTime Parms error !!!" << std::endl;
    return 0;
  }

  if (serverId < 0)
  {
    std::cout << "serverId Parms error !!!" << std::endl;
    return 0;
  }

  // init config
  // This script creates two nodes moving at 20 m/s, placed at a distance of 10 m.
  // These nodes exchange packets through a UDP application,
  // and they communicate using a wireless channel.

  // system parameters
  double bandwidth = 1e8;  // bandwidth in Hz 100MHz
  double frequency = 28e9; // the carrier frequency 28GHz
  uint32_t numerology = 3; // the numerology

  // applications
  // uint32_t packetSize = 1024;        // UDP packet size in bytes
  // uint32_t startTime = 50;           // application start time in milliseconds
  // uint32_t endTime = 2000;           // application end time in milliseconds
  // uint32_t interPacketInterval = 30; // interpacket interval in microseconds

  uint32_t port = 8888;

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

  std::map<uint64_t, Packetitem> umap;
  std::set<uint32_t> receivedUids;

  // init ZMQ
  zmq::context_t context;

  zmq::socket_t posRecvSocket = zmq::socket_t(context, ZMQ_PULL);
  posRecvSocket.connect("tcp://127.0.0.1:" + std::to_string(zmqPort)); // recv position

  zmq::socket_t pkgRecvSocket = zmq::socket_t(context, ZMQ_PULL);
  pkgRecvSocket.connect("tcp://127.0.0.1:" + std::to_string(zmqPort+1)); // recv packet event

  zmq::socket_t pkgSendSocket = zmq::socket_t(context, ZMQ_PUSH);
  pkgSendSocket.bind("tcp://127.0.0.1:" + std::to_string(zmqPort+2)); // send simulation result

  zmq::socket_t rsmSendSocket = zmq::socket_t(context, ZMQ_PUSH);
  rsmSendSocket.bind("tcp://127.0.0.1:" + std::to_string(zmqPort+3)); // send resume signal

  // create the nodes
  gContainer.Create(m_nodes);

  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(gContainer);
  for (int i = 0; i < m_nodes; i++)
  {
    gContainer.Get(i)->GetObject<MobilityModel>()->SetPosition(Vector(0, i * 0.1, 0)); // the position of the nodes must be different
  }

  // create and configure the helper
  Ptr<MmWaveVehicularHelper> helper = CreateObject<MmWaveVehicularHelper>();
  helper->SetNumerology(3); // why is 3?
  devices = helper->InstallMmWaveVehicularNetDevices(gContainer);

  // Install the TCP/IP stack in the two nodes
  InternetStackHelper internet;
  internet.Install(gContainer);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign(devices);

  BuildingsHelper::Install(gContainer);

  helper->PairDevices(devices);

  // Set the routing table
  // Ipv4StaticRoutingHelper ipv4RoutingHelper;
  // Ptr<Ipv4StaticRouting> staticRouting = ipv4RoutingHelper.GetStaticRouting(gContainer.Get(0)->GetObject<Ipv4>());
  // staticRouting->SetDefaultRoute(gContainer.Get(1)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal(), 2);

  Ptr<mmwave::MmWaveAmc> m_amc = CreateObject<mmwave::MmWaveAmc>(helper->GetConfigurationParameters());
  // init server
  UdpEchoServerHelper server(port);
  ApplicationContainer echoApps = server.Install(gContainer.Get(serverId));
  echoApps.Start(Seconds(0.0));
  echoApps.Get(0)->TraceConnectWithoutContext("Rx", MakeBoundCallback(&Rx, &pkgSendSocket, &umap, &receivedUids));
  // echoApps.Get(0)->TraceConnectWithoutContext("SinrReport", MakeBoundCallback(&MonitorSnifferRx, &pkgSendSocket, &umap, &receivedUids));
  // echoApps.Get(0)->TraceConnectWithoutContext("PhyRxDrop", MakeBoundCallback(&PhyRxDrop, &pkgSendSocket, &umap));
  // echoApps.Get(0)->TraceConnectWithoutContext("PhyTxDrop", MakeBoundCallback(&PhyTxDrop, &pkgSendSocket, &umap));
  // echoApps.Get(0)->TraceConnectWithoutContext("MacRxDrop", MakeBoundCallback(&MacRxDrop, &pkgSendSocket, &umap));
  // echoApps.Get(0)->TraceConnectWithoutContext("MacTxDrop", MakeBoundCallback(&MacTxDrop, &pkgSendSocket, &umap));
  // echoApps.Get(0)->TraceConnectWithoutContext("Tx", MakeBoundCallback(&MacTxDrop, &pkgSendSocket, &umap));

  Simulator::ScheduleNow(&takeTurn, devices, &pkgRecvSocket, &posRecvSocket, &rsmSendSocket, &umap, m_syncTime);
  Simulator::Run();
  Simulator::Destroy();
  return 0;
}
