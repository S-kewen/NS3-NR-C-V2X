/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2021 DERONNE SOFTWARE ENGINEERING
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
 * Author: Sébastien Deronne <sebastien.deronne@gmail.com>
 */

#include "eht-capabilities.h"

namespace ns3 {

EhtCapabilities::EhtCapabilities ()
  : m_ehtSupported (0)
{
}

WifiInformationElementId
EhtCapabilities::ElementId () const
{
  return IE_EXTENSION;
}

WifiInformationElementId
EhtCapabilities::ElementIdExt () const
{
  return IE_EXT_EHT_CAPABILITIES;
}

void
EhtCapabilities::SetEhtSupported (uint8_t ehtSupported)
{
  m_ehtSupported = ehtSupported;
}

uint8_t
EhtCapabilities::GetInformationFieldSize () const
{
  //we should not be here if EHT is not supported
  NS_ASSERT (m_ehtSupported > 0);
  return 0; //FIXME
}

Buffer::Iterator
EhtCapabilities::Serialize (Buffer::Iterator i) const
{
  if (m_ehtSupported < 1)
    {
      return i;
    }
  return WifiInformationElement::Serialize (i);
}

uint16_t
EhtCapabilities::GetSerializedSize () const
{
  if (m_ehtSupported < 1)
    {
      return 0;
    }
  return WifiInformationElement::GetSerializedSize ();
}

void
EhtCapabilities::SerializeInformationField (Buffer::Iterator start) const
{
  if (m_ehtSupported == 1)
    {
      //TODO
    }
}

uint8_t
EhtCapabilities::DeserializeInformationField (Buffer::Iterator start, uint8_t length)
{
  //TODO
  return length;
}

std::ostream &
operator << (std::ostream &os, const EhtCapabilities &ehtCapabilities)
{
  //TODO
  return os;
}

} //namespace ns3
