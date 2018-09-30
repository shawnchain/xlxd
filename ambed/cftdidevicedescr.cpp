//
//  cftdidevicedescr.cpp
//  ambed
//
//  Created by Jean-Luc Deltombe (LX3JL) on 02/06/2017.
//  Copyright © 2015 Jean-Luc Deltombe (LX3JL). All rights reserved.
//
// ----------------------------------------------------------------------------
//    This file is part of ambed.
//
//    xlxd is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    xlxd is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
// ----------------------------------------------------------------------------

#include "main.h"
#include <string.h>
#include "cusb3000interface.h"
#include "cusb3080interface.h"
#include "cftdidevicedescr.h"

#include "Log.h"

////////////////////////////////////////////////////////////////////////////////////////
// constructor

CFtdiDeviceDescr::CFtdiDeviceDescr(void)
{
    m_bUsed = false;
    m_uiVid = 0;
    m_uiPid = 0;
    ::memset(m_szDescription, 0, sizeof(m_szDescription));
    ::memset(m_szSerial, 0, sizeof(m_szSerial));
}

CFtdiDeviceDescr::CFtdiDeviceDescr(uint32 uiVid, uint32 uiPid, const char *szDescription, const char *szSerial)
{
    m_bUsed = false;
    m_uiVid = uiVid;
    m_uiPid = uiPid;
    ::strcpy(m_szDescription, szDescription);
    ::strcpy(m_szSerial, szSerial);
}

CFtdiDeviceDescr::CFtdiDeviceDescr(const CFtdiDeviceDescr &descr)
{
    m_bUsed = descr.m_bUsed;
    m_uiVid = descr.m_uiVid;
    m_uiPid = descr.m_uiPid;
    ::memcpy(m_szDescription, descr.m_szDescription, sizeof(m_szDescription));
    ::memcpy(m_szSerial, descr.m_szSerial, sizeof(m_szSerial));
}

////////////////////////////////////////////////////////////////////////////////////////
// interface factory

int CFtdiDeviceDescr::CreateInterface(CFtdiDeviceDescr *descr, std::vector<CVocodecChannel *>*channels)
{
    int iNbChs = 0;
    //TODO create signle channel interface
    // done
    return iNbChs;
}

int CFtdiDeviceDescr::CreateInterfacePair(CFtdiDeviceDescr *descr1, CFtdiDeviceDescr *descr2, std::vector<CVocodecChannel *>*channels)
{
    int iNbChs = 0;

    // create interface objects
    if ( (descr1->GetNbChannels() == 1) && (descr2->GetNbChannels() == 1) )
    {
        // create 3000-3000 pair
        CUsb3000Interface *Usb3000A = InstantiateUsb3000(descr1);
        CUsb3000Interface *Usb3000B = InstantiateUsb3000(descr2);
        iNbChs = CreatePair(Usb3000A, Usb3000B, channels);
        LogDebug("%s:%s <--> %s:%s are linked",descr1->GetDescription(),descr1->GetSerialNumber(),descr2->GetDescription(),descr2->GetSerialNumber());
    }
    // done
    return iNbChs;
}

////////////////////////////////////////////////////////////////////////////////////////
// get

const char * CFtdiDeviceDescr::GetChannelDescription(int ch) const
{
    static char descr[FTDI_MAX_STRINGLENGTH];
    char tag[3] = "_X";
    
    ::strcpy(descr, GetDescription());
    if ( ::strlen(descr) >= 2 )
    {
        descr[::strlen(descr)-2] = 0x00;
        tag[1] = (char)ch + 'A';
        ::strcat(descr, tag);
    }
    return descr;
}

const char * CFtdiDeviceDescr::GetChannelSerialNumber(int ch) const
{
    static char serial[FTDI_MAX_STRINGLENGTH];
    
    ::strcpy(serial, GetSerialNumber());
    if ( ::strlen(serial) >= 1 )
    {
        serial[::strlen(serial)-1] = (char)ch + 'A';
    }
    return serial;
}

int CFtdiDeviceDescr::GetNbChannels(void) const
{
    int iNbChs = 0;
    
    // single channel devices
    if ( (::strcmp(m_szDescription, "USB-3000")   == 0) ||           // DVSI's USB-3000
         (::strcmp(m_szDescription, "DVstick-30") == 0) ||           // DVMEGA AMBE3000 device
         (::strcmp(m_szDescription, "ThumbDV")    == 0) ||           // ThumbDV
         (::strcmp(m_szDescription, "Nano-3080")    == 0) )          // BG4TGO WT3080
    {
        iNbChs = 1;
    }
    // done
    return iNbChs;
}


////////////////////////////////////////////////////////////////////////////////////////
// 1 ch + 1 ch pair creation

int CFtdiDeviceDescr::CreatePair(CUsb3000Interface *Usb3000A, CUsb3000Interface *Usb3000B, std::vector<CVocodecChannel *>*channels)
{
    int nStreams = 0;
    
    // init the interfaces
    if ( Usb3000A->Init(CODEC_AMBEPLUS) && Usb3000B->Init(CODEC_AMBE2PLUS) )
    {
        CVocodecChannel *Channel;
        // create all channels
        {
            // ch1 for DStar -> DMR
            Channel = new CVocodecChannel(Usb3000A, 0, Usb3000B, 0, CODECGAIN_AMBEPLUS);
            channels->push_back(Channel);
            Usb3000A->AddChannel(Channel);
            Usb3000B->AddChannel(Channel);
            LogDebug("Created DStar -> DMR channel");

            // ch2 for DMR -> DStar
            Channel = new CVocodecChannel(Usb3000B, 0, Usb3000A, 0, CODECGAIN_AMBE2PLUS);
            channels->push_back(Channel);
            Usb3000A->AddChannel(Channel);
            Usb3000B->AddChannel(Channel);
            LogDebug("Created DMR -> DStar channel");
            // done
            nStreams = 2;
        }
    }
    else
    {
        // cleanup
        delete Usb3000A;
        delete Usb3000B;
    }
    
    // done
    return nStreams;
   
}

CUsb3000Interface *CFtdiDeviceDescr::InstantiateUsb3000(CFtdiDeviceDescr *descr)
{
    CUsb3000Interface *Usb3000 = NULL;
    
    // intstantiate the proper version of USB-3000
    if ( (::strcmp(descr->GetDescription(), "USB-3000")  == 0) ||           // DVSI's USB-3000
         (::strcmp(descr->GetDescription(), "DVstick-30")== 0) ||           // DVMEGA AMBE3000 device
         (::strcmp(descr->GetDescription(), "ThumbDV")   == 0) )            // ThumbDV
   {
        Usb3000 = new CUsb3000Interface
            (descr->GetVid(), descr->GetPid(), descr->GetDescription(), descr->GetSerialNumber());
    }
    else if ( (::strcmp(descr->GetDescription(), "Nano-3080") == 0) )         // BG4TGO WT3080
    {
        Usb3000 = new CUsb3080Interface
            (descr->GetVid(), descr->GetPid(), descr->GetDescription(), descr->GetSerialNumber());
    }    
    // done
    return Usb3000;
}
