/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2006 INRIA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */

#include <iomanip>
#include <iostream>
#include <fstream>
#include <vector>
#include <string.h>

#include "ns3/core-module.h"

using namespace ns3;

/** Flag to write debugging output. */
bool g_debug = false;

/** Name of this program. */
std::string g_me;
/** Log to std::cout */
#define LOG(x)   std::cout << x << std::endl
/** Log with program name prefix. */
#define LOGME(x) LOG (g_me << x)
/** Log debugging output. */
#define DEB(x) if (g_debug) { LOGME (x); }

/** Output field width for numeric data. */
int g_fwidth = 6;

/**
 *  Benchmark instance which can do a single run.
 *
 *  The run is controlled by the event population size and
 *  total number of events, which are set at construction.
 *
 *  The event distribution in time is set by SetRandomStream()
 */
class Bench
{
public:
  /**
   * Constructor
   * \param [in] population The number of events to keep in the scheduler.
   * \param [in] total The total number of events to execute.
   */
  Bench (const uint32_t population, const uint32_t total)
    : m_population (population),
      m_total (total),
      m_count (0)
  {
  }

  /**
   * Set the event delay interval random stream.
   *
   * \param [in] stream The random variable stream to be used to generate
   *              delays for future events.
   */
  void SetRandomStream (Ptr<RandomVariableStream> stream)
  {
    m_rand = stream;
  }

  /**
   * Set the number of events to populate the scheduler with.
   * Each event executed schedules a new event, maintaining the population.
   * \param [in] population The number of events to keep in the scheduler.
   */
  void SetPopulation (const uint32_t population)
  {
    m_population = population;
  }

  /**
   * Set the total number of events to execute.
   * \param [in] total The total number of events to execute.
   */
  void SetTotal (const uint32_t total)
  {
    m_total = total;
  }

  /** Run the benchmark as configure. */
  void RunBench (void);

private:
  /**
   *  Event function. This checks for completion (total number of events
   *  executed) and schedules a new event if not complete.
   */
  void Cb (void);

  Ptr<RandomVariableStream> m_rand;    /**< Stream for event delays. */
  uint32_t m_population;               /**< Event population size. */
  uint32_t m_total;                    /**< Total number of events to execute. */
  uint32_t m_count;                    /**< Count of events executed so far. */
};

void
Bench::RunBench (void)
{
  SystemWallClockMs time;
  double init, simu;

  DEB ("initializing");
  m_count = 0;


  time.Start ();
  for (uint32_t i = 0; i < m_population; ++i)
    {
      Time at = NanoSeconds (m_rand->GetValue ());
      Simulator::Schedule (at, &Bench::Cb, this);
    }
  init = time.End ();
  init /= 1000;
  DEB ("initialization took " << init << "s");

  DEB ("running");
  time.Start ();
  Simulator::Run ();
  simu = time.End ();
  simu /= 1000;
  DEB ("run took " << simu << "s");

  LOG (std::setw (g_fwidth) << init <<
       std::setw (g_fwidth) << (m_population / init) <<
       std::setw (g_fwidth) << (init / m_population) <<
       std::setw (g_fwidth) << simu <<
       std::setw (g_fwidth) << (m_count / simu)
                            << (simu / m_count)
       );

}

void
Bench::Cb (void)
{
  if (m_count >= m_total)
    {
      Simulator::Stop ();
      return;
    }
  DEB ("event at " << Simulator::Now ().GetSeconds () << "s");

  Time after = NanoSeconds (m_rand->GetValue ());
  Simulator::Schedule (after, &Bench::Cb, this);
  ++m_count;
}


/**
 *  Create a RandomVariableStream to generate next event delays.
 *
 *  If the \p filename parameter is empty a default exponential time
 *  distribution will be used, with mean delay of 100 ns.
 *
 *  If the \p filename is `-` standard input will be used.
 *
 *  \param [in] filename The delay interval source file name.
 *  \returns The RandomVariableStream.
 */
Ptr<RandomVariableStream>
GetRandomStream (std::string filename)
{
  Ptr<RandomVariableStream> stream = 0;

  if (filename == "")
    {
      LOG ("  Event time distribution:      default exponential");
      Ptr<ExponentialRandomVariable> erv = CreateObject<ExponentialRandomVariable> ();
      erv->SetAttribute ("Mean", DoubleValue (100));
      stream = erv;
    }
  else
    {
      std::istream *input;

      if (filename == "-")
        {
          LOG ("  Event time distribution:      from stdin");
          input = &std::cin;
        }
      else
        {
          LOG ("  Event time distribution:      from " << filename);
          input = new std::ifstream (filename.c_str ());
        }

      double value;
      std::vector<double> nsValues;

      while (!input->eof ())
        {
          if (*input >> value)
            {
              uint64_t ns = (uint64_t) (value * 1000000000);
              nsValues.push_back (ns);
            }
          else
            {
              input->clear ();
              std::string line;
              *input >> line;
            }
        }
      LOG ("    Found " << nsValues.size () << " entries");
      auto drv = CreateObject<DeterministicRandomVariable> ();
      drv->SetValueArray (&nsValues[0], nsValues.size ());
      stream = drv;
    }

  return stream;
}

/**
 * Perform the runs for a single scheduler type.
 *
 * This will create and set the scheduler, then execute a priming run
 * followed by the number of data runs requsted.
 *
 * Output will be in the form of a table showing performance for each run.
 *
 * \param [in] factory Factory pre-configured to create the desired Scheduler.
 * \param [in] pop The event population size.
 * \param [in] total The total number of events to execute.
 * \param [in] runs The number of replications.
 * \param [in] eventStream The random stream of event delays.
 * \param [in] calRev For the CalendarScheduler, whether to set the Reverse attribute.
 */
void
Run (ObjectFactory & factory, uint32_t pop, uint32_t total, uint32_t runs,
     Ptr<RandomVariableStream> eventStream, bool calRev)
{
  Simulator::SetScheduler (factory);

  std::string schedType = factory.GetTypeId ().GetName ();
  if (schedType == "ns3::CalendarScheduler")
    {
      schedType += ": insertion order: "
        + std::string (calRev ? "reverse" : "normal");
    }

  DEB ("scheduler: " << schedType);
  DEB ("population: " << pop);
  DEB ("total events: " << total);
  DEB ("runs: " << runs);

  Bench *bench = new Bench (pop, total);
  bench->SetRandomStream (eventStream);

  // table header
  LOG ("");
  LOG (schedType);
  LOG (std::left << std::setw (g_fwidth) << "Run #" <<
       std::left << std::setw (3 * g_fwidth) << "Initialization:" <<
       std::left                         << "Simulation:"
       );
  LOG (std::left << std::setw (g_fwidth) << "" <<
       std::left << std::setw (g_fwidth) << "Time (s)" <<
       std::left << std::setw (g_fwidth) << "Rate (ev/s)" <<
       std::left << std::setw (g_fwidth) << "Per (s/ev)" <<
       std::left << std::setw (g_fwidth) << "Time (s)" <<
       std::left << std::setw (g_fwidth) << "Rate (ev/s)" <<
       std::left                         << "Per (s/ev)"
       );
  LOG (std::setfill ('-') <<
       std::right << std::setw (g_fwidth) << " " <<
       std::right << std::setw (g_fwidth) << " " <<
       std::right << std::setw (g_fwidth) << " " <<
       std::right << std::setw (g_fwidth) << " " <<
       std::right << std::setw (g_fwidth) << " " <<
       std::right << std::setw (g_fwidth) << " " <<
       std::right << std::setw (g_fwidth) << " " <<
       std::setfill (' ')
       );

  // prime
  DEB ("priming");
  std::cout << std::left << std::setw (g_fwidth) << "(prime)";
  bench->RunBench ();

  bench->SetPopulation (pop);
  bench->SetTotal (total);
  for (uint32_t i = 0; i < runs; i++)
    {
      std::cout << std::setw (g_fwidth) << i;

      bench->RunBench ();
    }

  LOG ("");
  Simulator::Destroy ();
  delete bench;
}


int main (int argc, char *argv[])
{

  bool allSched  = false;
  bool schedCal  = false;
  bool schedHeap = false;
  bool schedList = false;
  bool schedMap  = false;   // default scheduler
  bool schedPQ   = false;

  uint32_t pop   =  100000;
  uint32_t total = 1000000;
  uint32_t runs  =       1;
  std::string filename = "";
  bool calRev = false;

  CommandLine cmd (__FILE__);
  cmd.Usage ("Benchmark the simulator scheduler.\n"
             "\n"
             "Event intervals are taken from one of:\n"
             "  an exponential distribution, with mean 100 ns,\n"
             "  an ascii file, given by the --file=\"<filename>\" argument,\n"
             "  or standard input, by the argument --file=\"-\"\n"
             "In the case of either --file form, the input is expected\n"
             "to be ascii, giving the relative event times in ns.");
  cmd.AddValue ("all",   "use all schedulers",            allSched);
  cmd.AddValue ("cal",   "use CalendarSheduler",          schedCal);
  cmd.AddValue ("calrev", "reverse ordering in the CalendarScheduler", calRev);
  cmd.AddValue ("heap",  "use HeapScheduler",             schedHeap);
  cmd.AddValue ("list",  "use ListSheduler",              schedList);
  cmd.AddValue ("map",   "use MapScheduler (default)",    schedMap);
  cmd.AddValue ("pri",   "use PriorityQueue",             schedPQ);
  cmd.AddValue ("debug", "enable debugging output",       g_debug);
  cmd.AddValue ("pop",   "event population size (default 1E5)",         pop);
  cmd.AddValue ("total", "total number of events to run (default 1E6)", total);
  cmd.AddValue ("runs",  "number of runs (default 1)",    runs);
  cmd.AddValue ("file",  "file of relative event times",  filename);
  cmd.AddValue ("prec",  "printed output precision",      g_fwidth);
  cmd.Parse (argc, argv);

  g_me = cmd.GetName () + ": ";
  g_fwidth += 6;  // 5 extra chars in '2.000002e+07 ': . e+0 _

  LOG (std::setprecision (g_fwidth - 6));  // prints blank line
  LOGME (" Benchmark the simulator scheduler");
  LOG ("  Event population size:        " << pop);
  LOG ("  Total events per run:         " << total);
  LOG ("  Number of runs per scheduler: " << runs);
  DEB ("debugging is ON");

  if (allSched)
    {
      schedCal = schedHeap = schedList = schedMap = schedPQ = true;
    }
  // Set the default case if nothing else is set
  if (! (schedCal || schedHeap || schedList || schedMap || schedPQ))
    {
      schedMap = true;
    }

  Ptr<RandomVariableStream> eventStream = GetRandomStream (filename);


  ObjectFactory factory ("ns3::MapScheduler");
  if (schedCal)
    {
      factory.SetTypeId ("ns3::CalendarScheduler");
      factory.Set ("Reverse", BooleanValue (calRev));
      Run (factory, pop, total, runs, eventStream, calRev);
      if (allSched)
        {
          factory.Set ("Reverse", BooleanValue (!calRev));
          Run (factory, pop, total, runs, eventStream, !calRev);
        }
    }
  if (schedHeap)
    {
      factory.SetTypeId ("ns3::HeapScheduler");
      Run (factory, pop, total, runs, eventStream, calRev);
    }
  if (schedList)
    {
      factory.SetTypeId ("ns3::ListScheduler");
      Run (factory, pop, total, runs, eventStream, calRev);
    }
  if (schedMap)
    {
      factory.SetTypeId ("ns3::MapScheduler");
      Run (factory, pop, total, runs, eventStream, calRev);
    }
  if (schedPQ)
    {
      factory.SetTypeId ("ns3::PriorityQueueScheduler");
      Run (factory, pop, total, runs, eventStream, calRev);
    }

  return 0;
}
