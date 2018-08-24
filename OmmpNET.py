#!/usr/bin/env python3.4 -i

##MIT License
##
##Copyright (c) 2018 Douglas E. Moore
##
##Permission is hereby granted, free of charge, to any person obtaining a copy
##of this software and associated documentation files (the "Software"), to deal
##in the Software without restriction, including without limitation the rights
##to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
##copies of the Software, and to permit persons to whom the Software is
##furnished to do so, subject to the following conditions:
##
##The above copyright notice and this permission notice shall be included in all
##copies or substantial portions of the Software.
##
##THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
##IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
##FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
##AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
##LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
##OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
##SOFTWARE.

from collections import namedtuple
from collections import OrderedDict
from struct import *
import socket
import binascii

def prng(ofs,siz=0,cnt=0):
    """
    create a range of memory map addresses separated by siz bytes
    """
    if siz !=0 and cnt!=0:
        # an array of cnt elements separated by siz bytes 
        return range(ofs,cnt*siz+ofs,siz)
    else:
        # a range with a single address
        return range(ofs,ofs+1)

"""
OmmpNET.commands OrderedDict uses this
"""
CmdFmt = namedtuple('CmdFmt',['ofs','len','typ','pak'])

"""
read/write block/quadlet functions use this
"""
RspFmt = namedtuple('RspFmt',['rcode','data'])

"""
change this for testing on other systems
"""
aa = ('192.168.10.225',2001)

class OmmpNET:

    def __init__(self):
        self.transaction_label = 0
        
    """
    tcode field in ieee-1394 Flags word
    Flags |= (tcode << 4)
    """
    tcodes = (\
        'WriteQuadletRequest',
        'WriteBlockRequest',
        'WriteQuadletBlockResponse',
        'ReservedTcode3',
        'ReadQuadletRequest',
        'ReadQuadletResponse',
        'ReadBlockRequest',
        'ReadBlockResponse'
        )
    
    """
    namedtuples for packing commands and unpacking responses
    """    
    WriteQuadletRequest = namedtuple('WriteQuadletRequest',\
        ['tl','tcode','dofs_hi','dofs_lo','value'])

    WriteQuadletResponse = namedtuple('WriteResponse',\
        ['tl','tcode','rcode'])

    WriteBlockRequest = namedtuple('WriteBlockRequest',\
        ['tl','tcode','dofs_hi','dofs_lo','length','value'])

    WriteBlockResponse = namedtuple('WriteResponse',\
        ['tl','tcode','rcode'])
    
    ReadQuadletRequest = namedtuple('ReadQuadletRequest',\
        ['tl','tcode','dofs_hi','dofs_lo'])

    ReadQuadletResponseHeader = namedtuple('ReadQuadletResponseHeader',\
        ['tl','tcode','rcode'])
    
    ReadQuadletResponse = namedtuple('ReadQuadletResponse',\
        ['tl','tcode','rcode','value'])

    ReadBlockRequest = namedtuple('ReadBlockRequest',\
        ['tl','tcode','dofs_hi','dofs_lo','length'])

    ReadBlockResponseHeader = namedtuple('ReadBlockResponseHeader',\
        ['tl','tcode','rcode','length'])
    
    ReadBlockResponse = namedtuple('ReadBlockResponse',\
        ['tl','tcode','rcode','length','value'])

    """
    format strings for packing commands and unpacking responses
    """    
    WriteQuadletRequestFormatString = '!2x2B2xH2I'
    WriteBlockRequestHeaderFormatString = '!2x2B2xHIH2x'
    WriteResponseHeaderFormatString = '!2x2B2xB5x'    
    ReadQuadletRequestFormatString = '!2x2B2xHI'
    ReadQuadletResponseHeaderFormatString = '!2x2B2xB5x'
    ReadBlockRequestFormatString = '!2x2B2xHIH2x'
    ReadBlockResponseHeaderFormatString = '!2x2B2xB5xH2x'

    def get_next_transaction_label(self):
        """
        tl is incremented with each use for potential response verification
        """
        tl = self.transaction_label
        self.transaction_label += 1
        return tl

    def get_transaction_label(self):
        """
        get the last tl sent
        """
        return self.transaction_label

    def write_mmap_value(self,aa,cmd,value,index=0):
        """
        aa -  is the device's UDP address as a tuple, ie:(ip,port)
        cmd - is a key string from the commands OrderedDict
        value is the value to be written
        index - is used to index the prng

        Note:
        calcsize(CmdFmt.pak) is used to determine if a quadlet or
        block write is required.
        """
        mmap = self.commands[cmd]
        if calcsize(mmap.pak) > 4:
            return self.write_block_request(aa,mmap,index,value)
        else:
            return self.write_quadlet_request(aa,mmap,index,value)
            
    def read_mmap_value(self,aa,cmd,index=0):
        """
        aa -  is the device's UDP address as a tuple, ie:(ip,port)
        cmd - is a key string from the commands OrderedDict
        index - is used to index the prng

        Note:
        calcsize(CmdFmt.pak) is used to determine if a quadlet or
        block read is required.
        """
        mmap = self.commands[cmd]
        if calcsize(mmap.pak) > 4:
            return self.read_block_request(aa,mmap,index)
        else:
            return self.read_quadlet_request(aa,mmap,index)

    def write_quadlet_request(self,aa,mmap,index,value):
        """
        aa    -  is the device's UDP address as a tuple, ie:(ip,port)
        mmap  - commands dictionary value
        index - index for addressing array of say tomers, events, points, etc
        value - value to be written
        """
        tl = self.get_next_transaction_label()
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        txargs = self.WriteQuadletRequest(tl<<2,0<<4,0xffff,mmap.ofs[index],\
            value)
        print(txargs)
        pkt = pack(self.WriteQuadletRequestFormatString,*txargs)
        print('WriteQuadletRequest',binascii.hexlify(pkt))
        s.sendto(pkt,aa)
        d = s.recvfrom(1024)
        print('WriteResponse',binascii.hexlify(d[0]))
        rsp = unpack(self.WriteResponseHeaderFormatString,d[0])
        rsp = self.WriteQuadletResponse(*(rsp[0]>>2,rsp[1]>>4,rsp[2]>>4))
        return (rsp.rcode,self.ecodes[rsp.rcode])

    def write_block_request(self,aa,mmap,index,value):
        """
        aa    -  is the device's UDP address as a tuple, ie:(ip,port)
        mmap  - commands dictionary value
        index - index for addressing array of say tomers, events, points, etc
        value - value to be written

        Note: the write block transfer length is obtained from
            calcsize(mmap.pak)
        """
        tl = self.get_next_transaction_label()
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        txargs = self.WriteBlockRequest(tl<<2,1<<4,0xffff,mmap.ofs[index],\
            calcsize(mmap.pak),value)
        print(txargs)
        fmt = self.WriteBlockRequestHeaderFormatString + mmap.pak
        pkt = pack(fmt,*txargs)
        print('WriteBlockRequest',binascii.hexlify(pkt))
        s.sendto(pkt,aa)
        d = s.recvfrom(1024)
        print('WriteResponse',binascii.hexlify(d[0]))
        rsp = unpack(self.WriteResponseHeaderFormatString,d[0])
        rsp = self.WriteBlockResponse(*(rsp[0]>>2,rsp[1]>>4,rsp[2]>>4))
        return (rsp.rcode,self.ecodes[rsp.rcode])

    def read_quadlet_request(self,aa,mmap,index):
        """
        aa    -  is the device's UDP address as a tuple, ie:(ip,port)
        mmap  - commands dictionary value
        index - index for addressing array of say tomers, events, points, etc
        """
        tl = self.get_next_transaction_label()
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        txargs = self.ReadQuadletRequest(*(tl<<2,4<<4,0xffff,mmap.ofs[index]))
        print(txargs)
        pkt = pack(self.ReadQuadletRequestFormatString,*txargs)
        print('ReadQuadletRequest',binascii.hexlify(pkt))
        s.sendto(pkt,aa)
        d = s.recvfrom(1024)
        print('ReadQuadletResponse',binascii.hexlify(d[0]))
        # is rcode == 0?
        rsp = unpack(self.ReadQuadletResponseHeaderFormatString,d[0]\
            [:calcsize(self.ReadQuadletResponseHeaderFormatString)])
        rsp = self.ReadQuadletResponseHeader(*(rsp[0]>>2,rsp[1]>>4,rsp[2]>>4))
        if rsp.rcode == 0:
            return (rsp.rcode,unpack('!'+mmap.pak,d[0][calcsize(\
                self.ReadQuadletResponseHeaderFormatString):]))
        else:
            return (rsp.rcode,self.ecodes[rsp.rcode])
            
    def read_block_request(self,aa,mmap,index):
        """
        aa    -  is the device's UDP address as a tuple, ie:(ip,port)
        mmap  - commands dictionary value
        index - index for addressing array of say tomers, events, points, etc

        Note: the read block transfer length is obtained from
            calcsize(mmap.pak)
        """
        blen = calcsize(mmap.pak)
        tl = self.get_next_transaction_label()
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        txargs = self.ReadBlockRequest(*(tl<<2,5<<4,0xffff,\
            mmap.ofs[index],calcsize(mmap.pak)))
        print(txargs)
        pkt = pack(self.ReadBlockRequestFormatString,*txargs)
        print('ReadBlockRequest',binascii.hexlify(pkt))
        s.sendto(pkt,aa)
        d = s.recvfrom(1024)
        print('ReadBlockResponse',binascii.hexlify(d[0]))
        # get the ReadBlockResponseHeader
        rsp = unpack(self.ReadBlockResponseHeaderFormatString,\
            d[0][:calcsize(self.ReadBlockResponseHeaderFormatString)])
        rsp = self.ReadBlockResponseHeader(*(rsp[0]>>2,rsp[1]>>4,\
            rsp[2]>>4,rsp[3]))
        # is rcode == 0?
        if rsp.rcode == 0:
            return (rsp.rcode,unpack('!'+mmap.pak,d[0][calcsize(\
                self.ReadBlockResponseHeaderFormatString):]))
        else:
            return (rsp.rcode,self.ecodes[rsp.rcode])

    """
    Description for Errors returned in rcode
    can use index of if desired
    """
    ecodes = {
        0:'No Error',
        1:'Undefined command',
        2:'Invalid Point type',
        3:'Invalid float',
        4:'Powerup Clear expected',
        5:'Invalid memory Address or invalid Data for the memory Address',
        6:'Invalid command Length',
        7:'Reserved',
        8:'Busy',
        9:'Cannot erase flash',
        10:'Cannot program flash',
        11:'Downloaded image too small',
        12:'Image CRC mismatch',
        13:'Image Length mismatch',
        14:'Feature is not yet implemented',
        15:'Communications watchdog Timeout'
        }
    
    """
    Controller types
    """
    brain_types = {
        0x004E:'SNAP-PAC-R1-B',
        0x0052:'OPTOEMU-SNR-DR2',
        0x0056:'OPTOEMU-SNR-DR1',
        0x0058:'G4EB2',
        0x005A:'OPTOEMU-SNR-3V',
        0x005C:'SNAP-PAC-SRA',
        0x0062:'SNAP-PAC-SB2',
        0x0064:'SNAP-PAC-SB1',
        0x0066:'SNAP-PAC-R2-W',
        0x0068:'SNAP-PAC-R1-W',
        0x006A:'SNAP-PAC-S1-W',
        0x006C:'SNAP-PAC-S2-W',
        0x0070:'SNAP-PAC-EB2-W',
        0x0072:'SNAP-PAC-EB1-W',
        0x0074:'SNAP-PAC-EB2',
        0x0076:'SNAP-PAC-EB1',
        0x0078:'SNAP-PAC-R2',
        0x007A:'SNAP-PAC-R1',
        0x007C:'SNAP-PAC-S1',
        0x0083:'SNAP-ENET-S64',
        0x008A:'SNAP-UPN-ADS',
        0x008C:'SNAP-UP1-M64',
        0x0092:'SNAP-UP1-D64',
        0x0093:'SNAP-UP1-ADS',
        0x0094:'SNAP-WLAN-FH-ADS',
        0x0097:'SNAP-ENET-D64',
        0x0098:'SNAP-B3000-ENET,SNAP-ENET-RTC',
        0x00E1:'E1',
        0x00E2:'E2',
        0x0193:'SNAP-LCE'
        }

    """
    Module types
    """
    Module_types = {
        0x00:'4-ch Digital or empty*',
        0x41:'SNAP-AIV-4',
        0xA9:'SNAP-AOD29',
        0x04:'SNAP-AICTD',
        0x42:'SNAP-AICTD-4',
        0xAB:'SNAP-AOA-23-iH',
        0x09:'SNAP-AITM-2',
        0x43:'SNAP-AIR40K-4',
        0xB3:'SNAP-AOA23-iSRC,SNAP-AOA23-iSRC-FM',
        0x0A:'SNAP-AIPM',
        0x44:'SNAP-AIMV-4',
        0xB9:'SNAP-OAD-29-HFi',
        0x0B:'SNAP-AILC',
        0x45:'SNAP-AIMV2-4',
        0xD0:'SNAP-PID-V',
        0x0C:'SNAP-AILC-2',
        0x48:'SNAP-AIPM-3V',
        0xE0:'SNAP-IDC-32,SNAP-IDC-32-FM',
        0x0E:'SNAP-AIRTD-10',
        0x49:'SNAP-AIPM-3',
        0xE1:'SNAP-ODC-32-SRC,SNAP-ODC-32-SRC-FM',
        0x0F:'SNAP-AIRTD-1K',
        0x4A:'SNAP-AIMA-8',
        0xE2:'SNAP-ODC-32-SNK,SNAP-ODC-32-SNK-FM',
        0x10:'SNAP-AIRTD',
        0x4B:'SNAP-AIV-8',
        0xE3:'SNAP-IAC-A-16',
        0x55:'SNAP-AIRTD-8U',
        0x4C:'SNAP-AICTD-8',
        0xE4:'SNAP-IAC-16',
        0x12:'SNAP-AIV',
        0x4D:'SNAP-AIMA-32,SNAP-AIMA-32-FM',
        0xE5:'SNAP-IDC-16',
        0x20:'SNAP-AITM-i',
        0x4E:'SNAP-AIV-32,SNAP-AIV-32-FM',
        0xE6:'SNAP-IDC-32N',
        0x21:'SNAP-AITM2-i',
        0x4F:'SNAP-AITM-8,SNAP-AITM-8-FM',
        0xE7:'SNAP-IAC-K-16',
        0x22:'SNAP-AIMA-i',
        0x64:'SNAP-AIMA',
        0xE8:'SNAP-IDC-HT-16',
        0x23:'SNAP-AIV-i',
        0x66:'SNAP-AITM',
        0xEA:'SNAP-IDC-32DN',
        0x24:'SNAP-AIV2-i',
        0x69:'SNAP-AIRATE',
        0xEB:'SNAP-IDC-32D',
        0x25:'SNAP-pH/ORP',
        0x70:'SNAP-AIVRMS',
        0xF0:'SNAP-SCM-232,SNAP-SCM-232 (Rev A)',
        0x26:'SNAP-AIMA-iSRC,SNAP-AIMA-iSRC-FM',
        0x71:'SNAP-AIARMS',
        0xF1:'SNAP-SCM-485,SNAP-SCM-485-422',
        0x27:'SNAP-AIMA2-i',
        0x83:'SNAP-AOA-3',
        0xF6:'SNAP-SCM-PROFI',
        0x28:'SNAP-AIVRMS-i,SNAP-AIVRMS-i-FM',
        0x85:'SNAP-AOV-5',
        0xF8:'SNAP-SCM-MCH16',
        0x29:'SNAP-AIARMS-i,SNAP-AIARMS-i-FM',
        0xA3:'SNAP-AOA23',
        0xF9:'SNAP-SCM-W2',
        0x2A:'SNAP-AIMA-iH',
        0xA5:'SNAP-AOV25',
        0xFA:'SNAP-SCM-SSI',
        0x2B:'SNAP-AIRATE-HFi',
        0xA7:'SNAP-AOV27',
        0xFB:'SNAP-SCM-ST2',
        0x32:'SNAP-AITM-4i',
        0xA8:'SNAP-AOA28',
        0xFC:'SNAP-SCM-CAN2B',
        0x40:'SNAP-AIMA-4',
        0xCF:'SNAP-AOVA-8'
    }

    Digital_Point_features = {
        0x0000:'disable Point features',
        0x0001:'Counter Inpit (cOnfigures and starts the Counter)',
        0x0002:'On-Time totalizer Inpit',
        0x0003:'Period measurement (cOntinuous)',
        0x0004:'simple quadrature Counter Inpit',
        0x0005:'frequency measurement (cOntinuous) (Firmware Lower than 8.1)',
        0x0008:'frequency measurement (cOntinuous) (Firmware R8.1a and higher)',
        0x0009:'On-pulse duration measurement (One-Time)',
        0x000A:'Off-pulse duration measurement (One-Time)',
        0x000B:'Period measurement (One-Time)',
        0x000C:'frequency measurement (One-Time)',
        0x0012:'Off-Time totalizer Inpit',
        0x0041:'quadrature Counter Inpit with index'
        }

    serial_types = {
        0xF0:'SNAP-SCM-232,SNAP-SCM-232 (Rev A)',
        0xF1:'SNAP-SCM-485-422 or SNAP-SCM-485',
        0xF6:'SNAP-SCM-PROFI',
        0xF8:'SNAP-SCM-MCH16',
        0xF9:'SNAP-SCM-W2',
        0xFA:'SNAP-SCM-SSI',
        0xFB:'SNAP-SCM-ST2',
        0xFC:'SNAP-SCM-CAN2B',
        0x2A:'SNAP-AIMA-iH',
        0xAB:'SNAP-AOA-23-iH',
        }

    pid_algorithm = {
        0:'Disabled',
        1:'Velocity Type B',
        2:'ISA',
        3:'Parallel',
        4:'Interacting',
        5:'Velocity Type C',
        6:'ISA (V >= R9.4c)',
        7:'Parallel (V >= R9.4c)',
        8:'Interaction (V >= R9.4c)',
        }
    
    """
    opcodes for mmap Address 0xf0380000
    """
    operation_codes = {    
        'Send Powerup Clear':1,
        'ReSet to defaults':2,
        'Save Cfg in flash':3,
        'Erase Cfg from flash':4,
        'ReSet Hardware':5,
        'Clear Digital Events Old Cfg':6,
        'Clear Alarms Cfg':7,
        'Clear PPP Cfg':8,
        'Clear EMail Cfg':9,
        'Clear Digital Events Expanded Cfg':10,
        'Clear PID Loops Cfg':11,
        'Clear Data Log':12,
        'Save Cfg and IP Address to MicroSD Card':13,
        'Erase Cfg and IP Address from MicroSD Card':14,
        'Erase Firmware from MicroSD Card':15,
        'Erase strategy from MicroSD Card':16,
        'Boot to loader':0x87654321
        }
    
    def power_up_clear(self,aa):
        key = 'Status R/W Operation Code'
        opc = self.operation_codes['Send Powerup Clear']
        return RspFmt(*self.write_mmap_value(aa,key,opc))

    def get_unit_type(self,aa):
        key = 'Status Read Device Unit Type'
        return RspFmt(*self.read_mmap_value(aa,key))

    def get_module_type(self,aa,point):
        key = 'Point Module Type'
        return RspFmt(*self.read_mmap_value(aa,key,point))

    def get_mmap_version(self,aa):
        key = 'Status Read Memory Map Version'
        return RspFmt(*self.read_mmap_value(aa,key))
    
    def get_firmware_version(self,aa):
        key = 'Status Read Firmware Version'
        return RspFmt(*self.read_mmap_value(aa,key))
    
    def get_firmware_version_string(self,aa):
        key = 'Status Read Firmware Version'
        rsp = RspFmt(*self.read_mmap_value(aa,key))
        if rsp.rcode == 0:
            VersionString = '{:s}{:d}.{:d}{:s}'.format(\
                ('A','RB','R')[rsp.data[2]],\
                rsp.data[0],\
                rsp.data[1],\
                chr(rsp.data[3]+ord('a')))
            return RspFmt(*(rsp.rcode,VersionString))
        return RspFmt(*rsp)
        
    def get_power_up_clear_required_flag(self,aa):
        key = 'Status Read Powerup Clear Flag'
        return RspFmt(*self.read_mmap_value(aa,key))

    def get_busy_flag(self,aa):
        key = 'Status Read Busy Flag'
        return RspFmt(*self.read_mmap_value(aa,key))
    
    def get_last_error(self,aa):
        key = 'Status Read Last Error code'
        return RspFmt(*self.read_mmap_value(aa,key))

    def get_mac_address(self,aa):
        # 00-A0-3D-00-0D-D8
        # (0, 160, 61, 0, 13, 216)
        key = 'Status Read MAC Address'
        return RspFmt(*self.read_mmap_value(aa,key))

    def set_point_type(self,aa,point,typ):
        key = 'Point Type'
        return RspFmt(*self.write_mmap_value(aa,key,typ,point))

    def get_point_type(self,aa,point):
        key = 'Point Type'
        return RspFmt(*self.read_mmap_value(aa,key,point))

    def activate_digital_point(self,aa,point):
        key = 'Digital Point Activate'
        return RspFmt(*self.write_mmap_value(aa,key,1,point))

    def deactivate_digital_point(self,aa,point):
        key = 'Digital Point Deactivate'                        
        return RspFmt(*self.write_mmap_value(aa,key,1,point))
        
    def get_digital_points_state(self,aa):
        key = 'Digital Point State'
        return RspFmt(*self.read_mmap_value(aa,key))
        
    def activate_digital_points(self,aa,mask):
        key = 'Digital Bank Point Activate Mask'
        return RspFmt(*self.write_mmap_value(aa,key,mask))

    def deactivate_digital_points(self,aa,mask):
        key = 'Digital Bank Point Deactivate Mask'
        return RspFmt(*self.write_mmap_value(aa,key,mask))

    def set_timer_delay(self,aa,timer,period):
        key = 'Timer Delay'           
        return RspFmt(*self.write_mmap_value(aa,key,period,timer))
        
    def get_timer_delay(self,aa,timer):
        key = 'Timer Delay'           
        return RspFmt(*self.read_mmap_value(aa,key,timer))

    def set_timer_trigger_on_mask(self,aa,timer,mask):
        key = 'Timer Start Input On Mask'
        return RspFmt(*self.write_mmap_value(aa,key,mask,timer))

    def get_timer_trigger_on_mask(self,aa,timer):
        key = 'Timer Start Input On Mask'
        return RspFmt(*self.read_mmap_value(aa,key,timer))

    def set_timer_trigger_off_mask(self,aa,timer,mask):
        key = 'Timer Start Input Off Mask'
        return RspFmt(*self.write_mmap_value(aa,key,mask,timer))

    def get_timer_trigger_off_mask(self,aa,timer):
        key = 'Timer Start Input Off Mask'
        return RspFmt(*self.read_mmap_value(aa,key,timer))

    def set_timer_reaction_on_mask(self,aa,timer,mask):
        key = 'Timer Expired Output On Mask'
        return RspFmt(*self.write_mmap_value(aa,key,mask,timer))

    def get_timer_reaction_on_mask(self,aa,timer):
        key = 'Timer Expired Output On Mask'
        return RspFmt(*self.read_mmap_value(aa,key,timer))

    def set_timer_reaction_off_mask(self,aa,timer,mask):
        key = 'Timer Expired Output Off Mask'
        return RspFmt(*self.write_mmap_value(aa,key,mask,timer))

    def get_timer_reaction_off_mask(self,aa,timer):
        key = 'Timer Expired Output Off Mask'
        return RspFmt(*self.read_mmap_value(aa,key,timer))

    def test_digital_bank_write(self,aa):
        """
        sample routine showing how to set and read back memory map values
        """
        for i in range(16):
            if i & 4 == 0:
                self.set_point_type(aa,i,0x180)
                self.activate_digital_point(aa,i)
        print('get_digital_state_mask')
        mask = self.om.test_digital_bank_write(aa)(aa).data
        print('set_digital_off_mask')
        self.set_digital_off_mask(aa,mask)
        print('get_digital_state_mask')
        mask = self.om.test_digital_bank_write(aa)(aa).data
        print('set_digital_on_mask')
        self.set_digital_on_mask(aa,mask)
        print('get_digital_state_mask')
        mask = self.get_digital_points_state(aa).data        
                
    def generate_10_hz_sqw_on_point_0(self,aa):
        """
        cfg p0.0 as a digital output
        deactivate p0.0
        set timer 0 delay to 50ms
        set timer 0 start event on activation of P0.0
        set timer 0 reaction event to turn off P0.0
        set timer 1 delay to 50ms
        set timer 1 start event to P0.0 clear
        set timer 1 reaction event to set P0.0
        activate p0.0 to start squarewave
        """
        # send a power up clear
        print('send a power up clear',self.power_up_clear(aa))
        # Set first point to an Output
        print('make digital point 0 an output',self.set_point_type(aa,0,0x180))
        # turn Off point P0.0
        print('turn off digital point 0',self.deactivate_digital_point(aa,0))
        # Set up a 50ms timer
        print('set t0 delay',self.set_timer_delay(aa,0,50))
        print('get t0 delay',self.get_timer_delay(aa,0))
        # start it when P0.0 Sets
        print('set t0 trigger on mask',self.set_timer_trigger_on_mask(aa,0,1))
        print('get t0 trigger on mask',self.get_timer_trigger_on_mask(aa,0))
        # use Event to clear P0.0
        print('set t0 reaction off mask',self.set_timer_reaction_off_mask(aa,0,1))
        print('get t0 reaction off mask',self.get_timer_reaction_off_mask(aa,0))
        # Set up a 50ms timer
        print('set t1 delay',self.set_timer_delay(aa,1,50))
        print('get t1 delay',self.get_timer_delay(aa,1))
        # start it when P0.0 clears
        print('set t1 trigger off mask',self.set_timer_trigger_off_mask(aa,1,1))
        print('get t1 trigger off mask',self.get_timer_trigger_off_mask(aa,1))
        # use Event to Set P0.0
        print('set t1 reaction on mask',self.set_timer_reaction_on_mask(aa,1,1))
        print('get t1 reaction on mask',self.get_timer_reaction_on_mask(aa,1))
        # turn On P0.0 to start timer 0
        print('turn on digital point 0 to start sqw',self.activate_digital_point(aa,0))

                
    """
    Momory Map information for building and parsing Data
    CmdFmt = namedtuple('CmdFmt',['ofs','len','typ','pak'])
        ofs - a range of 1 to cnt memory map addresses offset by siz bytes
        len - the number of bytes transferred
        typ - the OptoMMP type exchanged
        pak - Python pack/unpack type for the OptoMMP type and len
            
    """
    commands = OrderedDict([        
        # (EXPANDED) Analog & Digital POINT Cfg—READ/WRITE
        # 64 Points per Module for 64 Modules for 4096 points
        ('PointEX Module Type',CmdFmt(prng(0xf0100000,192,4096),4,'UI','I')),
        ('PointEX Type',CmdFmt(prng(0xf0100004,192,4096),4,'UI','I')),
        ('PointEX Feature',CmdFmt(prng(0xf0100008,192,4096),4,'UI','I')),
        ('PointEX Analog Offset',CmdFmt(prng(0xf010000c,192,4096),4,'F','f')),
        ('PointEX Analog Gain',CmdFmt(prng(0xf0100010,192,4096),4,'F','f')),
        ('PointEX Analog Hi Scaling Factor',CmdFmt(prng(0xf0100014,192,4096),4,'F','f')),
        ('PointEX Analog Lo Scaling factor',CmdFmt(prng(0xf0100018,192,4096),4,'F','f')),
        ('0xf010001c',CmdFmt(prng(0xf010001c,192,4096),4,'-','4x')),
        ('PointEX Average Filter Weight',CmdFmt(prng(0xf0100020,192,4096),4,'F','f')),
        ('PointEX Watchdog Value',CmdFmt(prng(0xf0100024,192,4096),4,'F','f')),
        ('PointEX Enable Watchdog',CmdFmt(prng(0xf0100028,192,4096),4,'B','I')),
        ('0xf010002c',CmdFmt(prng(0xf010002c,192,4096),4,'-','4x')),
        ('PointEX Name',CmdFmt(prng(0xf0100030,192,4096),51,'S-ZT','51s')),
        ('0xf0100063',CmdFmt(prng(0xf0100063,192,4096),1,'–','x')),        
        ('PointEX Steinhart-Hart Coefficient A',CmdFmt(prng(0xf0100064,192,4096),4,'F','f')),
        ('PointEX Steinhart-Hart Coefficient B',CmdFmt(prng(0xf0100068,192,4096),4,'F','f')),
        ('PointEX Steinhart-Hart Coefficient 2nd Order',CmdFmt(prng(0xf010006C,192,4096),4,'F','f')),
        ('PointEX Steinhart-Hart Coefficient C',CmdFmt(prng(0xf0100070,192,4096),4,'F','f')),
        ('0xf0100074',CmdFmt(prng(0xf0100074,192,4096),68,'–','68x')),
        ('PointEX Analog Lower Clamp',CmdFmt(prng(0xf01000B8,192,4096),4,'F','f')),
        ('PointEX Analog Upper Clamp',CmdFmt(prng(0xf01000BC,192,4096),4,'F','f')),
        # Points continue On 0xc0 boundary
        
        # (EXPANDED) Analog POINT CALC & Set—READ/WRITE
        # up to 64 Points per Module On up to 64 Modules
        ('PointEX Analog Offset',CmdFmt(prng(0xF01C0000,8,4096),4,'F','f')),
        ('PointEX Analog Gain',CmdFmt(prng(0xF01C0004,8,4096),4,'F','f')),

        # (EXPANDED) Analog POINT READ & CLEAR—READ/WRITE
        ('PointEX Minimum Value',CmdFmt(prng(0xF01D4000,12,4096),4,'F','f')),
        ('PointEX Maximum Value',CmdFmt(prng(0xF01D4004,12,4096),4,'F','f')),
        ('0xF01D4008',CmdFmt(prng(0xF01D4008,12,4096),4,'-','4x')),

        # (EXPANDED) Analog POINT READ—READ
        ('PointEX Analog Value in EU',CmdFmt(prng(0xF0260000,20,4096),4,'F','f')),
        ('PointEX Analog Value in Counts',CmdFmt(prng(0xF0260004,20,4096),4,'F','f')),
        ('PointEX Minimum Point Value in EU',CmdFmt(prng(0xF0260008,20,4096),4,'F','f')),
        ('PointEX Maximum Point Value in EU',CmdFmt(prng(0xF026000C,20,4096),4,'F','f')),
        ('Reserved',CmdFmt(prng(0xF0260010,20,4096),4,'-','4x')),

        # (EXPANDED) Analog POINT WRITE—READ/WRITE
        ('PointEX Analog Value in EU',CmdFmt(prng(0xF02A0000,64,4096),4,'F','f')),
        ('PointEX Analog Value in Counts',CmdFmt(prng(0xF02A0004,64,4096),4,'F','f')),
        ('0xF02A0008',CmdFmt(prng(0xF02A0008,64,4096),4,'-','4x')),
        ('PointEX TPO Period Seconds',CmdFmt(prng(0xF02A000C,64,4096),4,'F','f')),
        ('0xF02A0010',CmdFmt(prng(0xF02A0010,64,4096),4,'–','4x')),
        ('PointEX SNAP-AILC Fast Settle level',CmdFmt(prng(0xF02A0014,64,4096),4,'UI','I')),
        ('PointEX SNAP-AILC Filter Weight',CmdFmt(prng(0xF02A0018,64,4096),4,'UI','I')),
        ('0xF02A001C',CmdFmt(prng(0xF02A001C,64,4096),36,'–','36x')),

        # (EXPANDED) Digital POINT READ & CLEAR—READ OnLY
        ('PointEX Digital Point Feature Data',CmdFmt(prng(0xF02E0000,24,4096),4,'UI','I')),
        ('PointEX On Latch',CmdFmt(prng(0xF02E0004,24,4096),4,'UI','I')),
        ('PointEX Off Latch',CmdFmt(prng(0xF02E0008,24,4096),4,'UI','I')),
        ('0xF02E000C',CmdFmt(prng(0xF02E000C,24,4096),12,'–','12x')),

        # STATUS AREA READ—READ OnLY
        ('Status Read Memory Map Version',CmdFmt(prng(0xf0300000),4,'UI','I')),
        ('Status Read Powerup Clear Flag',CmdFmt(prng(0xf0300004),4,'UI','I')),
        ('Status Read Busy Flag',CmdFmt(prng(0xf0300008),4,'UI','I')),
        ('Status Read Last Error code',CmdFmt(prng(0xf030000C),4,'I','i')),
        ('Status Read Last Transaction label',CmdFmt(prng(0xf0300010),2,'UI','H')),
        ('Status Read Last Transaction Source Address',CmdFmt(prng(0xf0300012),2,'UI','H')),
        ('Status Read Last Transaction Error Address',CmdFmt(prng(0xf0300014),4,'UI','I')),
        ('Status Read Loader Version',CmdFmt(prng(0xf0300018),4,'UI','I')),
        ('Status Read Firmware Version',CmdFmt(prng(0xf030001C),4,'IP','4B')),
        ('Status Read Device Unit Type',CmdFmt(prng(0xf0300020),4,'UI','I')),
        ('Status Read Hardware Revision (Month)',CmdFmt(prng(0xf0300024),1,'UI','B')),
        ('Status Read Hardware Revision (Day)',CmdFmt(prng(0xf0300025),1,'UI','B')),
        ('Status Read Hardware Revision (Year)',CmdFmt(prng(0xf0300026),2,'UI','H')),
        ('Status Read Number of bytes of installed RAM',CmdFmt(prng(0xf0300028),4,'UI','I')),
        ('Status Read 0xf030002c',CmdFmt(prng(0xf030002c),2,'–','2x')),
        ('Status Read MAC Address',CmdFmt(prng(0xf030002e),6,'UI','6B')),
        ('Status Read TCP/IP Address',CmdFmt(prng(0xf0300034),4,'IP','4B')),
        ('Status Read TCP/IP subnet Mask',CmdFmt(prng(0xf0300038),4,'IP','4B')),
        ('Status Read TCP/IP default gateway',CmdFmt(prng(0xf030003c),4,'IP','4B')),
        ('Status Read DNS Server Address',CmdFmt(prng(0xf0300040),4,'IP','4B')),
        ('Status Read 0xf0300044',CmdFmt(prng(0xf0300044),4,'–','4x')),
        ('Status Read Send BootP or DHCP On Powerup',CmdFmt(prng(0xf0300048),4,'UI','I')),
        ('Status Read Temperature Units',CmdFmt(prng(0xf030004c),4,'B','I')),
        ('Status Read 0xf0300050',CmdFmt(prng(0xf0300050),4,'–','4x')),
        ('Status Read Watchdog Time in milliSeconds',CmdFmt(prng(0xf0300054),4,'UI','I')),
        ('Status Read TCP/IP RTO in milliSeconds',CmdFmt(prng(0xf0300058),4,'UI','I')),
        ('Status Read Digital Module Scan Counter',CmdFmt(prng(0xf030005C),4,'UI','I')),
        ('Status Read HD Digital and Analog Scan Counter',CmdFmt(prng(0xf0300060),4,'UI','I')),
        ('Status Read Initial RTO',CmdFmt(prng(0xf0300064),4,'UI','I')),
        ('Status Read TCP Number of retries',CmdFmt(prng(0xf0300068),4,'UI','I')),
        ('Status Read TCP idle session Timeout',CmdFmt(prng(0xf030006C),4,'UI','I')),
        ('Status Read Ethernet Errors: late collisions',CmdFmt(prng(0xf0300070),4,'UI','I')),
        ('Status Read Ethernet Errors: excessive collisions',CmdFmt(prng(0xf0300074),4,'UI','I')),
        ('Status Read Ethernet Errors: other',CmdFmt(prng(0xf0300078),4,'UI','I')),
        ('Status Read Smart Modules present',CmdFmt(prng(0xf030007C),4,'M','I')),
        ('Status Read Device’s part Number (string)',CmdFmt(prng(0xf0300080),32,'S-ZT','32s')),
        ('Status Read Firmware Version Date',CmdFmt(prng(0xf03000A0),16,'S-ZT','16s')),
        ('Status Read Firmware Version Time',CmdFmt(prng(0xf03000B0),16,'S-ZT','16s')),
        ('Status Read ARCNET Reconfigs detected',CmdFmt(prng(0xf0300100),4,'UI','I')),
        ('Status Read ARCNET Reconfigs initiated by I/O Unit',CmdFmt(prng(0xf0300104),4,'UI','I')),
        ('Status Read Idle Time Session Closed Count',CmdFmt(prng(0xf0300108),4,'UI','I')),
        ('Status Read Milliseconds since Powerup',CmdFmt(prng(0xf030010C),4,'UI','I')),
        ('Status Read Ethernet MAC reSets since Powerup',CmdFmt(prng(0xf0300110),4,'UI','I')),
        ('Status Read Digital Output Point reSets since Powerup',CmdFmt(prng(0xf0300114),4,'UI','I')),
        ('Status Read Digital interrupt failures since Powerup',CmdFmt(prng(0xf0300118),4,'UI','I')),
        ('Status Read PID loops available',CmdFmt(prng(0xf030011C),4,'UI','I')),
        ('Status Read ARCNET Transmit Attempts Since Powerup',CmdFmt(prng(0xf0300120),4,'UI','I')),
        ('Status Read ARCNET other',CmdFmt(prng(0xf0300124),4,'UI','I')),
        ('Status Read ARCNET ACKs',CmdFmt(prng(0xf0300128),4,'UI','I')),
        ('Status Read ARCNET ACK delay',CmdFmt(prng(0xf030012C),4,'UI','I')),
        ('Status Read ARCNET Timeout Value',CmdFmt(prng(0xf0300130),4,'UI','I')),
        ('Status Read ARCNET Timeouts',CmdFmt(prng(0xf0300134),4,'UI','I')),
        ('Status Read ARCNET Receive interrupts',CmdFmt(prng(0xf0300138),4,'UI','I')),
        ('0xf030013C',CmdFmt(prng(0xf030013C),4,'–','4x')),
        ('Status Read Milliseconds per Analog/HDD Scan',CmdFmt(prng(0xf0300140),4,'F','f')),
        ('Status Read Milliseconds per Digital (4-ch) Scan',CmdFmt(prng(0xf0300144),4,'F','f')),
        ('Status Read Number of LD Digital Modules supPorted',CmdFmt(prng(0xf0300148),4,'UI','I')),
        ('Status Read Serial Brain Serial Number',CmdFmt(prng(0xf030014C),4,'UI','I')),
        ('Status Read Serial Brain Multidrop Address',CmdFmt(prng(0xf0300150),4,'UI','I')),
        ('Status Read Serial Brain Baudrate',CmdFmt(prng(0xf0300154),4,'UI','I')),
        ('Status Read Serial Brain Number of Framing Errors',CmdFmt(prng(0xf0300158),4,'UI','I')),
        ('Status Read Serial Brain Number of FIFO Overrun Errors',CmdFmt(prng(0xf030015C),4,'UI','I')),
        ('Status Read Elapsed Time since Powerup',CmdFmt(prng(0xf0300160),4,'UI','I')),
        ('0xf0300164',CmdFmt(prng(0xf0300164),0x00C4,'–','196x')),
        ('Status Read Milliseconds Since Powerup',CmdFmt(prng(0xf0300228),8,'UI','Q')),
        ('Status Read Current boot Device',CmdFmt(prng(0xf0300230),4,'UI','I')),
        ('0xf0300234',CmdFmt(prng(0xf0300234),0x0014,'–','20x')),
        ('Status Read Status Area Write result',CmdFmt(prng(0xf0300248),4,'I','i')),
        ('Status Read Firmware Revision',CmdFmt(prng(0xf030024C),4,'UI','IP')),
        
        # COMMUNICATIONS PORT Cfg—READ/WRITE
        # (PAC-S & PAC-R only)
        # meaning is muddled without user's guide for controller
        ('Port 0 CTS state',CmdFmt(prng(0xf0310810),4,'UI','I')),
        ('Port 0 RI state',CmdFmt(prng(0xf0310814),4,'UI','I')),
        ('0xf0310818',CmdFmt(prng(0xf0310818),0x0014,'-','20x')),
        ('Port 1 RTS state',CmdFmt(prng(0xf031082C),4,'UI','I')),
        ('Port 1 CTS state',CmdFmt(prng(0xf0310830),4,'UI','I')),
        ('0xf0310834',CmdFmt(prng(0xf0310834),24,'–','24x')),
        ('Port 2 RTS',CmdFmt(prng(0xf031084C),4,'UI','I')),
        ('0xf0310850',CmdFmt(prng(0xf0310850),944,'–','944x')),
        ('Control for programmable PPP LED',CmdFmt(prng(0xf0310C00),4,'UI','I')),
        ('0xf0310C04',CmdFmt(prng(0xf0310C04),508,'-','508x')),
        ('State for programmable PPP LED',CmdFmt(prng(0xf0310E00),4,'UI','I')),
        ('PAC-S2 Port 0 Mode',CmdFmt(prng(0xf0311100),4,'UI','I')),
        ('PAC-S2 Port 1 Mode',CmdFmt(prng(0xf0311104),4,'UI','I')),
        ('PAC-S2 Port 2 Mode',CmdFmt(prng(0xf0311108),4,'UI','I')),
        ('PAC-S2 Port 3 Mode',CmdFmt(prng(0xf031110C),4,'UI','I')),

        # SERIAL PASS-THROUGH—READ/WRITE
        # 6 blocks on 0x1000 boundary
        ('Serial Pass Through Enable',CmdFmt(prng(0xf0329000,4096,6),4,'UI','I')),
        ('Serial Data Rate',CmdFmt(prng(0xf0329004,4096,6),4,'UI','I')),
        ('Serial Data Bits',CmdFmt(prng(0xf0329008,4096,6),4,'UI','I')),
        ('Serial Parity',CmdFmt(prng(0xf032900C,4096,6),4,'UI','I')),
        ('Serial Stop Bits',CmdFmt(prng(0xf0329010,4096,6),4,'UI','I')),
        ('Serial Duplicity',CmdFmt(prng(0xf0329014,4096,6),4,'UI','I')),
        ('Serial Half/Full Duplex',CmdFmt(prng(0xf0329018,4096,6),4,'UI','I')),
        ('0xf032901c',CmdFmt(prng(0xf032901c,4096,6),228,'-','228x')),
        ('Serial Write Target Address, Read Response Length',CmdFmt(prng(0xf0329100,4096,6),4,'UI','I')),
        ('Serial Write Packet Type, Read Response Data',CmdFmt(prng(0xf0329104,4096,6),4,'UI','I')),
        ('Serial Write Packet Length',CmdFmt(prng(0xf0329108,4096,6),4,'UI','I')),
        ('Serial Write Memory Map Command Packet',CmdFmt(prng(0xf032910C,4096,6),4,'UI','I')),
        ('0xf0329110',CmdFmt(prng(0xf0329110),0x0ef0,'-','3824x')),

        # DATE AND TIME Cfg—READ/WRITE
        ('Set Date and Time',CmdFmt(prng(0xf0350000),22,'S-ZT','22s')),
       
        #STATUS AREA WRITE—READ/WRITE
        ('Status R/W Operation Code',CmdFmt(prng(0xf0380000),4,'UI','I')),
        ('Status R/W BootP or DHCP Mode',CmdFmt(prng(0xf0380004),4,'UI','I')),
        ('Status R/W Temperature Units',CmdFmt(prng(0xf0380008),4,'B','I')),
        ('Status R/W Reserved',CmdFmt(prng(0xf038000C),4,'–','4x')),
        ('Status R/W Watchdog Timeout',CmdFmt(prng(0xf0380010),4,'UI','I')),
        ('Status R/W TCP/IP Minimum RTO',CmdFmt(prng(0xf0380014),4,'UI','I')),
        ('Status R/W TCP/IP initial RTO',CmdFmt(prng(0xf0380018),4,'UI','I')),
        ('Status R/W TCP/IP Number of retries',CmdFmt(prng(0xf038001C),4,'UI','I')),
        ('Status R/W idle session Timeout',CmdFmt(prng(0xf0380020),4,'UI','I')),
        ('Status R/W Wireless Mode',CmdFmt(prng(0xf0380024),4,'UI','I')),
        ('Status R/W Wireless ESS',CmdFmt(prng(0xf0380028),4,'UI','I')),
        ('Status R/W 0xf038002C',CmdFmt(prng(0xf038002C),32,'–','32x')),
        ('Status R/W Digital LD Scan Rate',CmdFmt(prng(0xf038004C),4,'UI','I')),
        ('Status R/W Digital HD and Analog Scan Rate',CmdFmt(prng(0xf0380050),4,'UI','I')),
        ('Status R/W Scanner Tuning Flags',CmdFmt(prng(0xf0380054),4,'M','I')),
        ('Status R/W On Mask for Scanner Flags',CmdFmt(prng(0xf0380058),4,'M','I')),
        ('Status R/W Off Mask for Scanner Flags',CmdFmt(prng(0xf038005C),4,'M','I')),
        ('0xf0380060',CmdFmt(prng(0xf0380060),244,'-','244x')),
        ('Status R/W DNS Host Name',CmdFmt(prng(0xf0380154),64,'S-ZT','64s')),
        ('Status R/W Domain Name',CmdFmt(prng(0xf0380194),256,'S-ZT','256s')),
        ('Status R/W Digital Scan Interval',CmdFmt(prng(0xf0380294),4,'UI','I')),
        ('Status R/W 16 Bit Overrange Value',CmdFmt(prng(0xf0380298),4,'F','f')),
        ('Status R/W 32 Bit Overrange Value',CmdFmt(prng(0xf03802B0),4,'F','f')),

        # MODBUS Cfg—READ/WRITE
        ('Modbus byte/word order',CmdFmt(prng(0x00390000),4,'UI','I')),

        # NETWORK SECURITY Cfg—READ/WRITE
        ('Network OptoMMP Port',CmdFmt(prng(0xf03a0004),4,'UI','I')),
        ('Network Modbus/TCP Port',CmdFmt(prng(0xf03a0008),4,'UI','I')),
        ('Network SNMP Port',CmdFmt(prng(0xf03a000C),4,'UI','I')),
        ('Network FTP Port',CmdFmt(prng(0xf03a0010),4,'UI','I')),
        ('0xf03a0014',CmdFmt(prng(0xf03a0014),12,'-','12x')),
        # 10 Filter Addresses and Masks
        ('Network IP Filter Address',CmdFmt(prng(0xf03a0020,8,10),4,'IP','4B')),
        ('Network IP Filter Mask',CmdFmt(prng(0xf03a0024,8,10),4,'IP','4B')),         
        ('Network Stop Incoming Broadcasts',CmdFmt(prng(0xf03a0070),4,'UI','I')),
        ('Network PAC Control Host Task Listen Port',CmdFmt(prng(0xf03a0074),4,'UI','I')),
        ('Network Enable IP Protocol',CmdFmt(prng(0xf03a0078),4,'UI','I')),
        ('Network Enable HTTPS Web Server',CmdFmt(prng(0xf03a007C),4,'UI','I')),

        #SSI MODULE Cfg—READ/WRITE
        # 16 SSI modules with up to 4 ports
        # but only first two ports on each module are implimented
        ('SSI Number of bits',CmdFmt(prng(0xf03a1000,64,64),4,'UI','I')),
        ('SSI Clock divider',CmdFmt(prng(0xf03a1004,64,64),4,'UI','I')),
        ('SSI Data delay',CmdFmt(prng(0xf03a1008,64,64),4,'UI','I')),
        ('SSI MSB Offset',CmdFmt(prng(0xf03a100C,64,64),4,'UI','I')),
        ('SSI MSG Length',CmdFmt(prng(0xf03a1010,64,64),4,'UI','I')),
        ('SSI Error bit Offset',CmdFmt(prng(0xf03a1014,64,64),4,'UI','I')),
        ('SSI Error bit meaning',CmdFmt(prng(0xf03a1018,64,64),4,'UI','I')),
        ('SSI Data coding',CmdFmt(prng(0xf03a101C,64,64),4,'UI','I')),
        ('SSI Enable scanning',CmdFmt(prng(0xf03a1020,64,64),4,'UI','I')),
        ('SSI Pad for alignment',CmdFmt(prng(0xf03a1024,64,64),28,'–','28x')),

        # SERIAL MODULE IDENTIFICATION—READ ONLY
        # up to 16 ports on 0x10 boundary
        ('Serial Module Hardware type',CmdFmt(prng(0xf03a7F00,16,16),1,'UI','B')),
        ('Serial Module Hardware subtype',CmdFmt(prng(0xf03a7F01,16,16),1,'UI','B')),
        ('Serial Module Hardware rev month',CmdFmt(prng(0xf03a7F02,16,16),1,'UI','B')),
        ('Serial Module Hardware rev day',CmdFmt(prng(0xf03a7F03,16,16),1,'UI','B')),
        ('Serial Module Hardware rev year',CmdFmt(prng(0xf03a7F04,16,16),2,'UI','H')),
        ('Serial Module Hardware loader revision',CmdFmt(prng(0xf03a7F06,16,16),4,'UI','I')),
        ('Serial Module Hardware firmware revision',CmdFmt(prng(0xf03a7F0A,16,16),4,'UI','I')),
        ('0xf03a7F0E',CmdFmt(prng(0xf03a7F0E,16,16),2,'–','2x')),

        # SERIAL MODULE Cfg—READ/WRITE
        # up to 16 ports on 0x10 boundary
        ('Serial Port TCP port number',CmdFmt(prng(0xF03A8000,16,16),4,'UI','I')),
        ('Serial Port Baud rate',CmdFmt(prng(0xF03A8004,16,16),4,'UI','I')),
        ('Serial Port Parity.',CmdFmt(prng(0xF03A8008,16,16),1,'UI','B')),
        ('Serial Port Data Bits',CmdFmt(prng(0xF03A8009,16,16),1,'UI','B')),
        ('Serial Port Stop Bits',CmdFmt(prng(0xF03A800A,16,16),1,'UI','B')),
        ('Serial Port Hardware flow control',CmdFmt(prng(0xF03A800B,16,16),1,'UI','B')),
        ('Serial Port Send test message on powerup',CmdFmt(prng(0xF03A800C,16,16),1,'UI','B')),
        ('0xF03A800D',CmdFmt(prng(0xF03A800D,16,16),3,'–','3x')),

        # WIEGAND SERIAL MODULE CONFIGURATION—READ/WRITE
        # up to 16 modules
        ('Wiegand Module Loader version number',CmdFmt(prng(0xF03A8500,8,16),4,'UI','I')),
        ('Wiegand Module Firmware kernel version number',CmdFmt(prng(0xF03A8504,8,16),4,'UI','I')),
        ('0xF03A880',CmdFmt(prng(0xF03A8580),128,'UI','128x')),
        # up to 32 ports, 2 per module
        ('Wiegand port TCP Port',CmdFmt(prng(0xF03A8600,64,32),4,'UI','I')),
        ('Wiegand Port Data Format',CmdFmt(prng(0xF03A8604,64,32),4,'UI','I')),
        ('Wiegand Port Data Length',CmdFmt(prng(0xF03A8608,64,32),4,'UI','I')),
        ('Wiegand Port First bit of the site code',CmdFmt(prng(0xF03A860C,64,32),4,'UI','I')),
        ('Wiegand Port Length of the site code, in bits',CmdFmt(prng(0xF03A8610,64,32),4,'UI','I')),
        ('Wiegand Port First bit of the badge code',CmdFmt(prng(0xF03A8614,64,32),4,'UI','I')),
        ('Wiegand Port Length of the badge code, in bits',CmdFmt(prng(0xF03A8618,64,32),4,'UI','I')),
        ('0xF03A861C',CmdFmt(prng(0xF03A861C,64,32),36,'-','36x')),

        # SNAP-SCM-CAN2B SERIAL MODULE CONFIGURATION—READ/WRITE
        # 16 single channel CAN moduleson 0x2c boundary
        ('CAN Module TCP/UDP Port Number',CmdFmt(prng(0xF03A9000,44,16),4,'UI','I')),
        ('CAN Module BAUD Rate',CmdFmt(prng(0xF03A9004,44,16),4,'UI','I')),
        ('CAN Module Data Mask 0',CmdFmt(prng(0xF03A9008,44,16),4,'UI','I')),
        ('CAN Module Filter 0',CmdFmt(prng(0xF03A900C,44,16),4,'UI','I')),
        ('CAN Module Filter 1',CmdFmt(prng(0xF03A9010,44,16),4,'UI','I')),
        ('CAN Module Data Mask 1',CmdFmt(prng(0xF03A9014,44,16),4,'UI','I')),
        ('CAN Module Filter 2',CmdFmt(prng(0xF03A9018,44,16),4,'UI','I')),
        ('CAN Module Filter 3',CmdFmt(prng(0xF03A901C,44,16),4,'UI','I')),
        ('CAN Module Filter 4',CmdFmt(prng(0xF03A9020,44,16),4,'UI','I')),
        ('CAN Module Filter 5',CmdFmt(prng(0xF03A9024,44,16),4,'UI','I')),
        ('CAN Module Error Code',CmdFmt(prng(0xF03A9028,44,16),4,'UI','I')),

        # HART MODULE CONFIGURATION—READ/WRITE
        # 16 Hart dual channel modules, channel on 0x30, module on 0x60 boundaries
        ('Hart TCP Port',CmdFmt(prng(0xF03A9400,48,16),4,'UI','I')),
        ('Hart Master Address',CmdFmt(prng(0xF03A9404,48,16),4,'UI','I')),
        ('Hart Retry Limit',CmdFmt(prng(0xF03A9408,48,16),4,'UI','I')),
        ('Hart Burst Mode',CmdFmt(prng(0xF03A940C,48,16),4,'UI','I')),
        ('Hart Promiscuous Mode',CmdFmt(prng(0xF03A9410,48,16),4,'UI','I')),
        ('Hart Preamble Count',CmdFmt(prng(0xF03A9414,48,16),4,'UI','I')),
        ('0xF03A9418',CmdFmt(prng(0xF03A9418,48,16),24,'-','24x')),

        # SNMP CONFIGURATION—READ/WRITE
        ('sysName—device name',CmdFmt(prng(0xF03C0000),32,'S-ZT','32s')),
        ('sysLocation—device location',CmdFmt(prng(0xF03C0020),32,'S-ZT','32s')),
        ('sysContact—owner ID',CmdFmt(prng(0xF03C0040),32,'S-ZT','32s')),
        ('Enable authentication trap',CmdFmt(prng(0xF03C0060),4,'UI','I')),
        ('Enable cold start trap',CmdFmt(prng(0xF03C0064),4,'UI','I')),
        
        # 8 communities
        ('Community Name',CmdFmt(prng(0xF03C0068,32,8),20,'S-ZT','20s')),
        ('Community read access privileges.',CmdFmt(prng(0xF03C007C,32,8),4,'UI','I')),
        ('Community Set write access privileges.',CmdFmt(prng(0xF03C0080,32,8),4,'UI','I')),
        ('Community Set trap access privileges.',CmdFmt(prng(0xF03C0084,32,8),4,'UI','I')),
        
        # 16 Hosts
        ('Host Community',CmdFmt(prng(0xF03C0168,24,16),20,'S-ZT','20s')),
        ('Host IP address',CmdFmt(prng(0xF03C017C,24,16),4,'IP','4B')),
        # 1 trap
        ('SNMP trap destination port',CmdFmt(prng(0xF03C0308),4,'UI','I')),
        ('SNMP trap version',CmdFmt(prng(0xF03C030C),4,'UI','I')),

        # FTP USER NAME/PASSWORD CONFIGURATION—READ/WRITE
        ('FTP User name',CmdFmt(prng(0xF03D0000),64,'S-ZT','64s')),
        ('FTP Password',CmdFmt(prng(0xF03D0040),64,'S-ZT','64s')),

        #PPP CONFIGURATION—READ/WRITE
        ('PPP Local IP address',CmdFmt(prng(0xF03E0000),4,'IP','4B')),
        ('PPP Remote IP address',CmdFmt(prng(0xF03E0004),4,'IP','4B')),
        ('PPP Serial port speed',CmdFmt(prng(0xF03E0008),4,'UI','I')),
        ('PPP Serial port parity',CmdFmt(prng(0xF03E000C),4,'UI','I')),
        ('PPP Serial port stop bits',CmdFmt(prng(0xF03E0010),4,'UI','I')),
        ('PPP Serial port data bits',CmdFmt(prng(0xF03E0014),4,'UI','I')),
        ('PPP Modem initialization string',CmdFmt(prng(0xF03E0018),64,'S-ZT','64s')),
        ('PPP Subnet mask',CmdFmt(prng(0xF03E0058),4,'IP','4B')),
        ('PPP Maximum Authentication Retries',CmdFmt(prng(0xF03E005C),4,'UI','4B')),
        ('PPP Serial port flow control.',CmdFmt(prng(0xF03E0060),4,'UI','I')),
        ('PPP Modem hangup string',CmdFmt(prng(0xF03E0064),64,'S-ZT','64s')),

        #PPP INCOMING CONNECTION CONFIGURATION—READ/WRITE
        ('PPP Send commands',CmdFmt(prng(0xF03E00A4),4,'UI','I')),
        ('PPP Echo request period',CmdFmt(prng(0xF03E00A8),4,'UI','I')),
        ('PPP Echo request retries',CmdFmt(prng(0xF03E00AC),4,'UI','I')),
        ('PPP Connection establishment timeout',CmdFmt(prng(0xF03E00B0),4,'UI','I')),
        ('PPP Enable incoming calls',CmdFmt(prng(0xF03E6000),4,'UI','I')),
        ('PPP default gateway',CmdFmt(prng(0xF03E6004),4,'UI','I')),
        ('PPP Modem listen string',CmdFmt(prng(0xF03E600C),64,'S-ZT','I')),
        ('PPP Inactivity timeout',CmdFmt(prng(0xF03E604C),4,'UI','I')),
        ('PPP incoming Login name, Firmware <= R5.0',CmdFmt(prng(0xF03E6050),16,'S-ZT','16s')),
        ('PPP incoming Password, Firmware <= R5.0',CmdFmt(prng(0xF03E6060),16,'S-ZT','16s')),
        ('PPP incoming Login name, Firmware >= R5.1',CmdFmt(prng(0xF03E6070),64,'S-ZT','64s')),
        ('PPP incoming Password, Firmware >= R5.1',CmdFmt(prng(0xF03E60B0),64,'S-ZT','64s')),

        #PPP Outgoing CONNECTION CONFIGURATION—READ/WRITE
        ('PPP Enable Outgoing calls',CmdFmt(prng(0xF03EB000),4,'UI','I')),
        ('PPP Specify local IP address',CmdFmt(prng(0xF03EB004),4,'UI','I')),
        ('PPP Set default gateway',CmdFmt(prng(0xF03EB008),4,'UI','I')),
        ('0xF03EB00C',CmdFmt(prng(0xF03EB00C),4,'-','4x')),
        ('PPP Outgoing Login name, Firmware <= R5.0',CmdFmt(prng(0xF03EB010),16,'S-ZT','16s')),
        ('PPP Outgoing Password, Firmware <= R5.0',CmdFmt(prng(0xF03EB020),16,'S-ZT','16s')),
        ('PPP inactivity timeout',CmdFmt(prng(0xF03EB030),4,'UI','I')),
        ('PPP Outgoing Phone number',CmdFmt(prng(0xF03EB034),64,'S-ZT','64s')),
        ('PPP Maximum Outgoing connection time',CmdFmt(prng(0xF03EB074),4,'UI','I')),
        ('PPP Maximum Outgoing dialing retries',CmdFmt(prng(0xF03EB078),4,'UI','I')),
        ('PPP Outgoing Retry interval',CmdFmt(prng(0xF03EB07C),4,'UI','I')),
        ('PPP Disable time before reconnect',CmdFmt(prng(0xF03EB080),4,'UI','I')),
        ('PPP Outgoing Link always connected',CmdFmt(prng(0xF03EB084),4,'UI','I')),
        ('PPP Outgoing Login, Firmware >= R5.1',CmdFmt(prng(0xF03EB088),64,'S-ZT','64s')),
        ('PPP Outgoing Password, Firmware >= R5.1',CmdFmt(prng(0xF03EB0C8),64,'S-ZT','64s')),

        # PPP STATUS—READ ONLY
        ('PPP Connection status',CmdFmt(prng(0xF03EB800),4,'UI','I')),
        ('PPP Connection type',CmdFmt(prng(0xF03EB804),4,'UI','I')),
        ('PPP Number of retries ',CmdFmt(prng(0xF03EB808),4,'UI','I')),
        ('PPP Idle time remaining',CmdFmt(prng(0xF03EB80C),4,'UI','I')),
        ('PPP Retry time remaining',CmdFmt(prng(0xF03EB810),4,'UI','I')),
        ('PPP Disable time remaining',CmdFmt(prng(0xF03EB814),4,'UI','I')),
        ('PPP Outgoing connection time remaining',CmdFmt(prng(0xF03EB818),4,'UI','I')),
        ('PPP Buffered Outgoing frames waiting',CmdFmt(prng(0xF03EB81C),4,'UI','I')),

        ('Stream Enable I/O mirroring',CmdFmt(prng(0xF03FFFC4),4,'B','I')),
        ('Stream Beginning address',CmdFmt(prng(0xF03FFFC8),4,'UI','I')),
        ('Stream Size of data',CmdFmt(prng(0xF03FFFCC),4,'UI','I')),
        ('Stream Enable',CmdFmt(prng(0xF03FFFD0),4,'B','I')),
        ('Stream Interval',CmdFmt(prng(0xF03FFFD4),4,'UI','I')),
        ('Stream UDP port number',CmdFmt(prng(0xF03FFFD8),4,'UI','I')),
        ('0xF03FFFDC',CmdFmt(prng(0xF03FFFDC),4,'UI','I')),
        # 8 stream target IP addresses
        ('Stream Destination IP address',CmdFmt(prng(0xF03FFFE0),4,'IP','4B')),

        # Digital BANK READ—READ ONLY
        ('Bank Read Digital State',CmdFmt(prng(0xF0400000),8,'M','Q')),
        ('Bank Read Digital On Latches',CmdFmt(prng(0xF0400008),8,'M','Q')),
        ('Bank Read Digital Off Latches',CmdFmt(prng(0xF0400010),8,'M','Q')),
        ('Bank Read Digital Counters State',CmdFmt(prng(0xF0400018),8,'M','Q')),
        ('Bank Read Digital Pulse Measurement Complete',CmdFmt(prng(0xF0400020),8,'M','Q')),
        ('Bank Read Counter Data',CmdFmt(prng(0xF0400100),256,'UI','64I')),
        
        # Digital BANK WRITE—READ/WRITE
        ('Digital Bank Point Activate Mask',CmdFmt(prng(0xF0500000),8,'M','Q')),
        ('Digital Bank Point Deactivate Mask',CmdFmt(prng(0xF0500008),8,'M','Q')),
        ('Digital Bank Counters Activate Mask',CmdFmt(prng(0xF0500010),8,'M','Q')),
        ('Digital Bank Counters Deactivate Mask',CmdFmt(prng(0xF0500018),8,'M','Q')),

        # Analog BANK READ—READ OnLY
        ('Analog Data in EU',CmdFmt(prng(0xF0600000),256,'F','64f')),
        ('Analog Data in Counts',CmdFmt(prng(0xF0600100),256,'F','64f')),
        ('Analog Min Value in EU',CmdFmt(prng(0xF0600200),256,'F','64f')),
        ('Analog Max Value in EU',CmdFmt(prng(0xF0600300),256,'F','64f')),

        # Analog BANK WRITE—READ/WRITE
        ('Analog Output in EU',CmdFmt(prng(0xF0700000),256,'F','64f')),
        ('Analog Outputin Counts',CmdFmt(prng(0xF0700100),256,'F','64f')),

        # Digital POINT READ—READ OnLY
        # 64 Points On 0x40 boundary
        ('Digital Point State',CmdFmt(prng(0xF0800000,64,64),4,'B','I')),
        ('Digital Point On-Latch State',CmdFmt(prng(0xF0800004,64,64),4,'B','I')),
        ('Digital Point Off-Latch State',CmdFmt(prng(0xF0800008,64,64),4,'B','I')),
        ('Digital Point Counter State',CmdFmt(prng(0xF080000C,64,64),4,'B','I')),
        ('Digital Point Feature Value',CmdFmt(prng(0xF0800010,64,64),4,'UI','I')),

        # Digital POINT WRITE—READ/WRITE
        # 64 Points On 0x40 boundary
        ('Digital Point Activate',CmdFmt(prng(0xF0900000,64,64),4,'B','I')),
        ('Digital Point Deactivate',CmdFmt(prng(0xF0900004,64,64),4,'B','I')),
        ('Digital Point Counter Activate',CmdFmt(prng(0xF0900008,64,64),4,'B','I')),
        ('Digital Point Counter Deactivate',CmdFmt(prng(0xF090000C,64,64),4,'B','I')),

        # (OLD) Analog POINT READ
        # 64 blocks of 64 bytes,
        ('Analog Read Value in EU',CmdFmt(prng(0xf0a00000,64,64),4,'F','f')),
        ('Analog Read Value in Counts',CmdFmt(prng(0xf0a00004,64,64),4,'F','f')),
        ('Analog Read Min Value in EU',CmdFmt(prng(0xf0a00008,64,64),4,'F','f')),
        ('Analog Read Max Value in EU',CmdFmt(prng(0xf0a0000c,64,64),4,'F','f')),
        ('0xf0a00010',CmdFmt(prng(0xf0a00010,64,64),48,'-','48x')),
        # Points continue On 0x40 boundaries
        
        # (OLD) Analog POINT WRITE—READ/WRITE
        # 64 blocks of 64 bytes,
        ('Analog Read/Write Output in EU',CmdFmt(prng(0xf0b00000,64,64),4,'F','f')),
        ('Analog Read/Write Output in Counts',CmdFmt(prng(0xf0b00004,64,64),4,'F','f')),
        ('Analog Read/Write TPO Resolution',CmdFmt(prng(0xf0b00008,64,64),4,'F','f')),
        ('Analog Read/Write TPO Period in Seconds',CmdFmt(prng(0xf0b0000c,64,64),4,'F','f')),
        ('0xf0b00010',CmdFmt(prng(0xf0b00010,64,64),4,'-','4x')),
        ('Analog Read/Write Load Cell Fast Settle Level',CmdFmt(prng(0xf0b00014,64,64),4,'UI','I')),
        ('Analog Read/Write Load Cell Filter Weight',CmdFmt(prng(0xf0b00018,64,64),4,'UI','I')),
        ('0xf0b0001c',CmdFmt(prng(0xf0b0001c,64,64),36,'-','36x')),
        # Points continue On 0x40 boundaries

        # (OLD) Analog AND Digital POINT Cfg INFORMATIOn—READ/WRITE
        # first of 64 blocks of 64 bytes,
        ('Point Module Type',CmdFmt(prng(0xf0c00000,64,64),4,'UI','I')),
        ('Point Type',CmdFmt(prng(0xf0c00004,64,64),4,'UI','I')),
        ('Point Feature',CmdFmt(prng(0xf0c00008,64,64),4,'UI','I')),
        ('Point Analog Offset',CmdFmt(prng(0xf0c0000c,64,64),4,'F','f')),
        ('Point Analog Gain',CmdFmt(prng(0xf0c00010,64,64),4,'F','f')),
        ('Point Analog Point Hi Scaling Factor',CmdFmt(prng(0xf0c00014,64,64),4,'F','f')),
        ('Point Analog Point Lo Scaling Factor',CmdFmt(prng(0xf0c00018,64,64),4,'F','f')),
        ('0xf0b0001c',CmdFmt(prng(0xf0b0001c,64,64),4,'-','4x')),
        ('Point Analog Average Filter Weight',CmdFmt(prng(0xf0b00020,64,64),4,'F','f')),
        ('Point Analog Watchdog Value in EU',CmdFmt(prng(0xf0b00024,64,64),4,'F','f')),
        ('Point Watchdog Enable',CmdFmt(prng(0xf0b00028,64,64),4,'B','I')),
        ('0xf0b0002c',CmdFmt(prng(0xf0b0002c,64,64),4,'-','4x')),
        ('Point Name',CmdFmt(prng(0xf0c00030,64,64),16,'S-ZT','16s')),
        # Points continue On 0x40 boundaries
        
        # first of 64 blocks of 8 bytes
        ('Point Analog Output Lower Clamp',CmdFmt(prng(0xf0c01000,8,64),4,'F','f')),
        ('Point Analog Output Upper Clamp',CmdFmt(prng(0xf0c01004,8,64),4,'F','f')),
        # Points continue On 0x08 boundaries

        # (OLD) Digital EventS AND ReactionS—READ/WRITE
        # first of 128 blocks of 64 bytes
        ('Digital Event Points On Mask',CmdFmt(prng(0xf0d00000,64,128),8,'M','Q')),
        ('Digital Event Points Off Mask',CmdFmt(prng(0xf0d00008,64,128),8,'M','Q')),
        ('Digital Reaction Points On Mask',CmdFmt(prng(0xf0d00010,64,128),8,'M','Q')),
        ('Digital Reaction Points Off Mask',CmdFmt(prng(0xf0d00018,64,128),8,'M','Q')),
        ('Digital Event Scratch Pad Bits On Mask',CmdFmt(prng(0xf0d00020,64,128),8,'M','Q')),
        ('Digital Event Scratch Pad Bits Off Mask',CmdFmt(prng(0xf0d00028,64,128),8,'M','Q')),
        ('Digital Reaction Scratch Pad Bits On Mask',CmdFmt(prng(0xf0d00030,64,128),8,'M','Q')),
        ('Digital Reaction Scratch Pad Bits Off Mask',CmdFmt(prng(0xf0d00038,64,128),8,'M','Q')),
        # Events continue On 0x40 boundary    

        # Digital EventS - EXPANDED (FORMERLY TIMERS)—READ/WRITE
        # first of 512 blocks of 128 Bits
        ('Timer Start Input On Mask',CmdFmt(prng(0xf0d40000,128,512),8,'M','Q')),
        ('Timer Start Input Off Mask',CmdFmt(prng(0xf0d40008,128,512),8,'M','Q')),
        ('Timer Start Scratch Pad On Mask',CmdFmt(prng(0xf0d40010,128,512),8,'M','Q')),
        ('Timer Start Scratch Pad Off Mask',CmdFmt(prng(0xf0d40018,128,512),8,'M','Q')),
        ('Timer Expired Output On Mask',CmdFmt(prng(0xf0d40020,128,512),8,'M','Q')),
        ('Timer Expired Output Off Mask',CmdFmt(prng(0xf0d40028,128,512),8,'M','Q')),
        ('Timer Expired Scratch Pad On Mask',CmdFmt(prng(0xf0d40030,128,512),8,'M','Q')),
        ('Timer Expired Scratch Pad Off Mask',CmdFmt(prng(0xf0d40038,128,512),8,'M','Q')),
        ('Timer Delay',CmdFmt(prng(0xf0d40040,128,512),4,'UI','I')),
        ('0xf0d40044',CmdFmt(prng(0xf0d40044,128,512),4,'–','4x')),
        ('Timer Time Remaining',CmdFmt(prng(0xf0d40048,128,512),4,'UI','I')),
        ('Timer State',CmdFmt(prng(0xf0d4004C,128,512),4,'UI','I')),
        ('0xf0d40050',CmdFmt(prng(0xf0d40050,128,512),48,'–','48x')),
        # Events continue On 0x80 boundary

        # CUSTOM Cfg AREA—WRITE
        # first element of 1024 elements
        # creates a scatter write gather read like custom memory area
        ('Configure Custom Data',CmdFmt(prng(0xf0d50000,4,1024),4,'UI','I')),
        
        # Custom Data ACCESS AREA—READ/WRITE
        # first element of 1024 elements
        # accesses a scatter write gather read like custom memory area
        ('Access Custom Data',CmdFmt(prng(0xf0d60000,4,1024),4,'*','TBD')),

        # SCRATCH PAD—READ/WRITE
        # 64 bits 
        ('Scratch Pad bit state',CmdFmt(prng(0xF0D80000),8,'M','Q')),
        ('Scratch Pad bit On mask',CmdFmt(prng(0xF0D80400),8,'M','Q')),
        ('Scratch Pad bit Off mask',CmdFmt(prng(0xF0D80408),8,'M','Q')),
        # 1024 32-bit integer values
        ('Scratch Pad 32-bit Integer value',CmdFmt(prng(0xF0D81000,4,1024),4,'I','i')),
        # 1024 floating point values
        ('Scratch Pad float value',CmdFmt(prng(0xF0D82000,4,1024),4,'F','f')),
        # 64 pascal like string values
        ('Scratch Pad string value Length',CmdFmt(prng(0xF0D83000,132,64),2,'UI','H')),
        ('Scratch Pad string value Data',CmdFmt(prng(0xF0D83002,132,64),128,'S-PL','128p')),
        # 9216 32-bit integer values
        ('Scratch Pad 32-bit Integer value',CmdFmt(prng(0xF0DA0000,4,9216),4,'I','i')),
        # 9216 floating point values
        ('Scratch Pad float value',CmdFmt(prng(0xF0DC0000,4,9216),4,'F','f')),
        # 1024 64-bit values
        ('Scratch Pad 64-bit Integer value',CmdFmt(prng(0xF0DE0000,64,1024),8,'M','Q')),

        # (OLD) Analog POINT CALCULATIOn AND Set—READ OnLY
        # 64 floating Point Offset Values
        ('Offset in EU',CmdFmt(prng(0xf0e00000,4,64),4,'F','f')),
        # 64 floating Point gain Values
        ('Offset in EU',CmdFmt(prng(0xf0e00100,4,64),4,'F','f')),

        #(OLD) Digital READ AND CLEAR—READ OnLY
        # 64 UI Count Values
        ('Counts',CmdFmt(prng(0xf0f00000,4,64),4,'UI','I')),      
        # 64 4 Boolean Values
        ('On Latch',CmdFmt(prng(0xf0f00100,4,64),4,'B','I')),
        # 64 4 Boolean Values
        ('Off Latch',CmdFmt(prng(0xf0f00200,4,64),4,'B','I')),

        # (OLD) Analog READ AND CLEAR/RESTART—READ OnLY
        # 64 Minimum Values
        ('Analog Minimum Value',CmdFmt(prng(0xf0f80000,4,64),4,'F','f')),
        # 64 Maximum Values
        ('Analog Maximum Value',CmdFmt(prng(0xf0f80100,4,64),4,'F','f')),

        # Streaming—READ OnLY
        ('Stream Analog Values in EU',CmdFmt(prng(0xf1000000),256,'F','64f')),
        ('Stream Digital Point feature Data',CmdFmt(prng(0xf1000100),256,'UI','64I')),
        ('Stream State of 4-channel Digital Points',CmdFmt(prng(0xf1000200),8,'M','Q')),
        ('Stream State of Digital On-Latches as Mask',CmdFmt(prng(0xf1000208),8,'M','Q')),
        ('Stream State of Digital Off-Latches as Mask',CmdFmt(prng(0xf1000210),8,'M','Q')),
        ('Stream Active Counters Mask',CmdFmt(prng(0xf1000218),8,'M','Q')),
        
        # Digital PACKED DATA—READ
        # 32 Points per Module
        # 0x80 Module boundary
        ('HD Digital Packed Counters',CmdFmt(prng(0xf1001000,128,128),4,'UI','I')),
        ('HD Digital Packed Flags and State',CmdFmt(prng(0xf1001800,128,128),4,'UI','I')),

        # Analog PACKED DATA—READ
        # 32 Points per Module
        # 0x80 Module boundary
        ('HD Analog Packed Value in EU',CmdFmt(prng(0xf1001000,128,128),4,'F','f')),
        ('HD Digital Packed State',CmdFmt(prng(0xf1001800,128,128),4,'UI','I')),

        # Alarm Event SetTINGS—READ/WRITE
        # 64 Alarms On 0x80 boundary
        ('Deviation Alarm State',CmdFmt(prng(0xf1100000,128,64),4,'B','I')),
        ('Enable Deviation Alarm',CmdFmt(prng(0xf1100004,128,64),4,'B','I')),
        ('Previous Deviation Value',CmdFmt(prng(0xf1100008,128,64),4,'F','f')),
        ('Deviation Amount',CmdFmt(prng(0xf110000C,128,64),4,'F','f')),
        ('On Deviation Alarm Set Scratch Pad Bits',CmdFmt(prng(0xf1100010,128,64),8,'M','Q')),
        ('On Deviation Alarm Clear Scratch Pad Bits',CmdFmt(prng(0xf1100018,128,64),8,'M','Q')),
        ('High Alarm State',CmdFmt(prng(0xf1100020,128,64),4,'B','I')),
        ('Enable High Alarm',CmdFmt(prng(0xf1100024,128,64),4,'B','I')),
        ('High Alarm Setpoint',CmdFmt(prng(0xf1100028,128,64),4,'F','f')),
        ('High Alarm Deadband',CmdFmt(prng(0xf110002C,128,64),4,'F','f')),
        ('On High Alarm Set Scratch Pad Bits',CmdFmt(prng(0xf1100030,128,64),8,'M','Q')),
        ('On High Alarm Clear Scratch Pad Bits',CmdFmt(prng(0xf1100038,128,64),8,'M','Q')),
        ('Low Alarm State',CmdFmt(prng(0xf1100040,128,64),4,'B','I')),
        ('Enable Low Alarm',CmdFmt(prng(0xf1100044,128,64),4,'B','I')),
        ('Low Alarm Setpoint',CmdFmt(prng(0xf1100048,128,64),4,'F','f')),
        ('Low Alarm Deadband',CmdFmt(prng(0xf110004C,128,64),4,'F','f')),
        ('On Low Alarm Set Scratch Pad Bits',CmdFmt(prng(0xf1100050,128,64),8,'M','Q')),
        ('On Low Alarm Clear Scratch Pad Bits',CmdFmt(prng(0xf1100058,128,64),8,'M','Q')),
        ('Value Address',CmdFmt(prng(0xf1100060,128,64),4,'UI','I')),
        ('Value Type',CmdFmt(prng(0xf1100064,128,64),4,'UI','I')),
        ('Pad for alignment',CmdFmt(prng(0xf1100068,128,64),24,'–','24x')),
        # 0x80 Module boundary

        # Event Message Cfg—READ/WRITE
        # 128 Messages 0xc0 boundary
        ('Current Message State',CmdFmt(prng(0xf1200000,192,128),4,'UI','I')),
        ('Scratch Pad Bits Set',CmdFmt(prng(0xf1200004,192,128),4,'M','I')),
        ('Scratch Pad Bits Clear',CmdFmt(prng(0xf120000C,192,128),4,'M','I')),
        ('Enable Streaming',CmdFmt(prng(0xf1200014,192,128),4,'B','I')),
        ('Stream Period in Seconds',CmdFmt(prng(0xf1200018,192,128),4,'UI','I')),
        ('Enable EMail',CmdFmt(prng(0xf120001C,192,128),4,'B','I')),
        ('EMail Period in Seconds',CmdFmt(prng(0xf1200020,192,128),4,'UI','I')),
        ('Enable SNMP Trap',CmdFmt(prng(0xf1200024,192,128),4,'B','I')),
        ('SNMP trap Period in Seconds',CmdFmt(prng(0xf1200028,192,128),4,'UI','I')),
        ('SNMP trap type',CmdFmt(prng(0xf120002C,192,128),4,'UI','I')),
        ('Priority',CmdFmt(prng(0xf1200030,192,128),4,'UI','I')),
        ('0xf1200034',CmdFmt(prng(0xf1200034,192,128),4,'–','4x')),
        ('Enable serial Module Message',CmdFmt(prng(0xf1200038,192,128),4,'B','I')),
        ('Serial Ports to Receive Message',CmdFmt(prng(0xf120003C,192,128),4,'M','I')),
        ('Message Text',CmdFmt(prng(0xf1200040,192,128),128,'S-ZT','128s')),

        # 128 destinations 0x10 boundary
        ('Destination Memory map Address',CmdFmt(prng(0xf1208000,16,128),4,'UI','I')),
        ('Destination I/0 Unit IP Address',CmdFmt(prng(0xf1208004,16,128),4,'UI','I')),
        ('Destination I/0 Unit UDP Port',CmdFmt(prng(0xf1208008,16,128),4,'UI','I')),
        ('Copy Interval',CmdFmt(prng(0xf120800C,16,128),4,'UI','I')),

        # 128 most recent Messages 0x0104 boundary
        ('Most recent Message Text',CmdFmt(prng(0xf1209000,260,128),256,'S-ZT','256s')),
        ('Most recent Message Length',CmdFmt(prng(0xf1209100,260,128),4,'UI','I')),

        # EMail Cfg—READ/WRITE
        ('EMAIL SMTP Mail Server IP Address',CmdFmt(prng(0xf1300000),4,'IP','4B')),
        ('EMAIL SMTP Mail Server Port Number',CmdFmt(prng(0xf1300004),4,'UI','I')),
        ('EMAIL TO: Address',CmdFmt(prng(0xf1300008),50,'S-ZT','50s')),        
        ('EMAIL FROM: I/O Unit',CmdFmt(prng(0xf130003A),50,'S-ZT','50s')),
        ('EMAIL RE: Subject',CmdFmt(prng(0xf130006C),50,'S-ZT','50s')),
        ('EMAIL EMail Response Timeout',CmdFmt(prng(0xf13000A0),4,'UI','I')),
        
        # SERIAL EVENT CONFIGURATION—READ/WRITE
        # 32 serial events on 0x74 boundary
        ('Serial Event port mask',CmdFmt(prng(0xF1540000,116,32),4,'M','I')),
        ('0xF1540004',CmdFmt(prng(0xF1540004,116,32),4,'-','4x')),
        ('Serial Event SNMP trap type',CmdFmt(prng(0xF1540008,116,32),4,'UI','I')),
        ('Serial Event SNMP trap period',CmdFmt(prng(0xF154000C,116,32),4,'UI','I')),
        ('Serial Event Pattern string',CmdFmt(prng(0xF1540010,116,32),40,'S-ZT','40s')),
        ('Serial Event Reaction string',CmdFmt(prng(0xF1540038,116,32),40,'S-ZT','40s')),
        ('Serial Event Scratch Pad bits to turn on',CmdFmt(prng(0xF1540060,116,32),8,'M','Q')),
        ('Serial Event Scratch Pad bits to turn off',CmdFmt(prng(0xF1540068),8,'M','Q')),
        ('Serial Event Enable email message',CmdFmt(prng(0xF1540070,116,32),4,'UI','I')),
        # 32 serial event trap priorities
        ('Serial Event SNMP trap priority',CmdFmt(prng(0xF1540E80,4,32),4,'UI','I')),
        # 32 serial event disables
        ('Serial Event SNMP trap Disable',CmdFmt(prng(0xF1540F00,4,32),4,'UI','I')),

        # WIEGAND SERIAL EVENT CONFIGURATION—READ/WRITE
        # 32 Wiegand events on 0x74 boundary
        ('Wiegand port event mask',CmdFmt(prng(0xF1560000,116,32),4,'M','I')),
        ('0xF1560004',CmdFmt(prng(0xF1560004,116,32),4,'UI','I')),
        ('Wiegand Event reaction trap',CmdFmt(prng(0xF1560008,116,32),4,'UI','I')),
        ('Wiegand Event reaction period',CmdFmt(prng(0xF156000C,116,32),4,'UI','I')),
        ('Wiegand Event Pattern string',CmdFmt(prng(0xF1560010,116,32),40,'S-ZT','40s')),
        ('Wiegand Event Reaction string',CmdFmt(prng(0xF1560038,116,32),40,'S-ZT','40s')),
        ('Wiegand Event Scratch Pad bits to turn on',CmdFmt(prng(0xF1560060,116,32),8,'M','Q')),
        ('Wiegand Event Scratch Pad bits to turn off',CmdFmt(prng(0xF1560068,116,32),8,'M','Q')),
        ('Wiegand Event Enable email message',CmdFmt(prng(0xF1560070,116,32),4,'UI','I')),
        # 32 Wiegand event trap priorities
        ('Wiegand Event SNMP trap priority',CmdFmt(prng(0xF1560E80,4,32),4,'UI','I')),
        # 32 Wiegand event trap disables
        ('Wiegand Event SNMP trap Disable',CmdFmt(prng(0xF1560F00,4,32),4,'UI','I')),

        # SNAP HIGH-DENSITY DIGITAL—READ ONLY
        # 16 HD modules 64 bit point state masks
        ('HD Digital Point state',CmdFmt(prng(0xF1808000,64,16),8,'M','Q')),
        ('HD Digital On-latch state',CmdFmt(prng(0xF1808008,64,16),8,'M','Q')),
        ('HD Digital Off-latch state',CmdFmt(prng(0xF1808010,64,16),8,'M','Q')),
        ('0xF1808018',CmdFmt(prng(0xF1808018,64,16),40,'-','40x')),
        # 4096 = up to 16 modules with up to 64 32 bit counters each
        ('HD Digital Counter value',CmdFmt(prng(0xF1809000,4,16),4,'UI','I')),

        # SNAP HIGH-DENSITY DIGITAL READ AND CLEAR—READ/WRITE
        # 16 HD modules 64 bit point state masks
        ('HD Digital On-latch clear mask',CmdFmt(prng(0xF180A000,32,16),8,'M','Q')),
        ('HD Digital Off-latch clear mask',CmdFmt(prng(0xF180A008,32,16),8,'M','Q')),
        ('Reserved',CmdFmt(prng(0xF180A010,32,16),16,'-','16x')),
        # 4096 = up to 16 modules with up to 64 32 bit counters each
        ('HD Digital Read and Clear Counter',CmdFmt(prng(0xF180B000,4,16),4,'UI','I')),

        # SNAP HIGH-DENSITY DIGITAL WRITE—READ/WRITE
        # 16 modules with up to 64 points
        ('HD Digital On mask',CmdFmt(prng(0xF180C000,64,16),8,'M','Q')),
        ('HD Digital Off mask',CmdFmt(prng(0xF180C008,64,16),8,'M','Q')),
        ('0xF180C010',CmdFmt(prng(0xF180C010,64,16),48,'-','48x')),

        # PID CONFIGURATION AND STATUS—READ/WRITE
        # realtime values
        # up to 150 PIDs on 0x50 boundaries
        ('PID Current Error value',CmdFmt(prng(0xF2000000,80,150),4,'F','f')),
        ('PID Current P value',CmdFmt(prng(0xF2000004,80,150),4,'F','f')),
        ('PID Current I value',CmdFmt(prng(0xF2000008,80,150),4,'F','f')),
        ('PID Current D value',CmdFmt(prng(0xF200000C,80,150),4,'F','f')),
        ('PID Current Value',CmdFmt(prng(0xF2000010,80,150),4,'F','f')),
        ('0xF2000014',CmdFmt(prng(0xF2000014,80,150),20,'-','20x')),
        ('PID Scan counter',CmdFmt(prng(0xF2000028,80,150),4,'UI','I')),
        ('PID Status flags',CmdFmt(prng(0xF200002C,80,150),4,'UI','I')),
        ('PID Status bits ON mask',CmdFmt(prng(0xF2000030,80,150),4,'UI','I')),
        ('PID Status bits OFF mask',CmdFmt(prng(0xF2000034,80,150),4,'UI','I')),
        ('PID Current input value',CmdFmt(prng(0xF2000038,80,150),4,'F','f')),
        ('0xF200003C',CmdFmt(prng(0xF200003C,80,150),20,'-','20x')),

        # PID CONFIGURATION AND STATUS—READ/WRITE
        # configuration values
        # up to 150 PID loops on 0x80 boundaries
        ('PID Process Value',CmdFmt(prng(0xF2100000,128,150),4,'F','f')),
        ('PID Setpoint value',CmdFmt(prng(0xF2100004,128,150),4,'F','f')),
        ('PID Current value of feed forward',CmdFmt(prng(0xF2100008,128,150),4,'F','f')),
        ('PID Current value of Output',CmdFmt(prng(0xF210000C,128,150),4,'F','f')),
        ('PID Gain value',CmdFmt(prng(0xF2100010,128,150),4,'F','f')),
        ('PID Integral value',CmdFmt(prng(0xF2100014,128,150),4,'F','f')),
        ('PID Derivative value',CmdFmt(prng(0xF2100018,128,150),4,'F','f')),
        ('PID Feed forward gain',CmdFmt(prng(0xF210001C,128,150),4,'F','f')),
        ('PID Maximum output change allowed',CmdFmt(prng(0xF2100020,128,150),4,'F','f')),
        ('PID Minimum output change allowed',CmdFmt(prng(0xF2100024,128,150),4,'F','f')),
        ('PID Input low range',CmdFmt(prng(0xF2100028,128,150),4,'F','f')),
        ('PID Input high range',CmdFmt(prng(0xF210002C,128,150),4,'F','f')),
        ('PID Output lower clamp',CmdFmt(prng(0xF2100030,128,150),4,'F','f')),
        ('PID Output upper clamp',CmdFmt(prng(0xF2100034,128,150),4,'F','f')),
        ('PID scan time',CmdFmt(prng(0xF2100038,128,150),4,'F','f')),
        ('PID underrange output',CmdFmt(prng(0xF210003C,128,150),4,'F','f')),
        ('PID overrange output',CmdFmt(prng(0xF2100040,128,150),4,'F','f')),
        ('PID Input address',CmdFmt(prng(0xF2100044,128,150),4,'UI','I')),
        ('PID setpoint address',CmdFmt(prng(0xF2100048,128,150),4,'UI','I')),
        ('PID output address',CmdFmt(prng(0xF210004C,128,150),4,'UI','I')),
        ('PID Algorithm choice.',CmdFmt(prng(0xF2100050,128,150),4,'UI','I')),
        ('PID Manual mode',CmdFmt(prng(0xF2100054,128,150),4,'UI','I')),
        ('PID mode flags',CmdFmt(prng(0xF2100058,128,150),4,'UI','I')),
        ('PID mode bits ON',CmdFmt(prng(0xF210005C,128,150),4,'UI','I')),
        ('PID mode bits OFF',CmdFmt(prng(0xF2100060,128,150),4,'UI','I')),
        ('PID Current process value in memory',CmdFmt(prng(0xF2100064,128,150),4,'F','f')),
        ('PID Current setpoint in memory',CmdFmt(prng(0xF2100068,128,150),4,'F','f')),
        ('0xF210006C',CmdFmt(prng(0xF210006C,128,150),0x14,'-','20x')),

        # DATA LOGGING CONFIGURATION—READ/WRITE
        # 64 logging records on 0x1c boundary
        ('LOG Scratch Pad bits on',CmdFmt(prng(0xF3000000,28,64),8,'M','Q')),
        ('LOG Scratch Pad bits off',CmdFmt(prng(0xF3000008,28,64),8,'M','Q')),
        ('LOG Memory map address',CmdFmt(prng(0xF3000010,28,64),4,'UI','I')),
        ('LOG Data format',CmdFmt(prng(0xF3000014,28,64),4,'UI','I')),
        ('LOG interval',CmdFmt(prng(0xF3000018,28,64),4,'UI','I')),
        ('LOG Enable email',CmdFmt(prng(0xF3000700),4,'B','I')),
        ('LOG Number of entries per message',CmdFmt(prng(0xF3000704),4,'UI','I')),

        # DATA LOG—READ/WRITE
        #  circular buffer of 300 entries of 20 byte records
        # the record format is explained on page 54 of the users guide
        ('LOG Data entry',CmdFmt(prng(0xF3020000,20,300),20,'UI','20p')),

        # PID MODULE CONFIGURATION—READ/WRITE
        # 16 pid modules 256 bytes per
        ('PID Control word',CmdFmt(prng(0xF4000000,256,16),4,'I','i')),
        ('PID Status flags',CmdFmt(prng(0xF4000004,256,16),4,'I','i')),
        ('PID Scantime resolution',CmdFmt(prng(0xF4000008,256,16),4,'I','i')),
        ('PID Scantime multiplier',CmdFmt(prng(0xF400000C,256,16),4,'I','i')),
        ('PID TPO period multiplier',CmdFmt(prng(0xF4000010,256,16),4,'I','i')),
        ('PID Output',CmdFmt(prng(0xF4000014,256,16),4,'I','i')),
        ('PID proportional gain',CmdFmt(prng(0xF4000018,256,16),4,'I','i')),
        ('PID integral ratio',CmdFmt(prng(0xF400001C,256,16),4,'I','i')),
        ('PID derivative ratio',CmdFmt(prng(0xF4000020,256,16),4,'I','i')),
        ('PID Setpoint',CmdFmt(prng(0xF4000024,256,16),4,'I','i')),
        ('PID Process variable',CmdFmt(prng(0xF4000028,256,16),4,'I','i')),
        ('PID Filter exponential',CmdFmt(prng(0xF400002C,256,16),4,'I','i')),
        ('PID Setpoint low limit',CmdFmt(prng(0xF4000030,256,16),4,'I','i')),
        ('PID Setpoint high limit',CmdFmt(prng(0xF4000034,256,16),4,'I','i')),
        ('PID Process low limit',CmdFmt(prng(0xF4000038,256,16),4,'I','i')),
        ('PID Process high limit',CmdFmt(prng(0xF400003C,256,16),4,'I','i')),
        ('PID Output low limit',CmdFmt(prng(0xF4000040,256,16),4,'I','i')),
        ('PID Output high limit',CmdFmt(prng(0xF4000044,256,16),4,'I','i')),
        ('PID Output slew rate',CmdFmt(prng(0xF4000048,256,16),4,'I','i')),
        ('PID Output limit deadband',CmdFmt(prng(0xF400004C,256,16),4,'I','i')),
        ('PID Current PID',CmdFmt(prng(0xF4000050,256,16),4,'I','i')),
        ('PID Last PID',CmdFmt(prng(0xF4000054,256,16),4,'I','i')),
        ('PID Oldest PID',CmdFmt(prng(0xF4000058,256,16),4,'I','i')),
        ('PID Current PID error',CmdFmt(prng(0xF400005C,256,16),4,'I','i')),
        ('PID Last PID error',CmdFmt(prng(0xF4000060,256,16),4,'I','i')),
        ('PID Output change',CmdFmt(prng(0xF4000064,256,16),4,'I','i')),
        ('PID Output value',CmdFmt(prng(0xF4000068,256,16),4,'I','i')),
        ('PID Scantime countdown',CmdFmt(prng(0xF400006C,256,16),4,'I','i')),

        # CONTROL ENGINE—READ/WRITE
        ('Control engine enable',CmdFmt(prng(0xF4080000),4,'UI','I')),
        ('Control engine feature',CmdFmt(prng(0xF4080004),4,'UI','I')),

        # SERIAL BRAIN COMMUNICATION—READ/WRITE
        ('SNAP PAC SB Turnaround Delay',CmdFmt(prng(0xF7002000),4,'UI','I')),
        ('SNAP PAC SB Debug Flag',CmdFmt(prng(0xF7002100),4,'B','I')),

        # MICROSD CARD—READ/WRITE
        ('MicroSD Card Update Enable',CmdFmt(prng(0xF7002200),4,'B','I')),
        ('MicroSD Card Available Space < 2GB',CmdFmt(prng(0xF7002204),4,'UI','I')),
        ('MicroSD Card Available Space > 2GB',CmdFmt(prng(0xF7002208),8,'UI','Q')),

        # WLAN STATUS—READ ONLY
        ('0xF7003000',CmdFmt(prng(0xF7003000),2,'-','2x')),
        ('WLAN MAC address',CmdFmt(prng(0xF7003002),6,'UI','6B')),
        ('WLAN state',CmdFmt(prng(0xF7003008),4,'UI','I')),
        ('0xF700300C',CmdFmt(prng(0xF700300C),2,'UI','2x')),
        ('WLAN ID for current BSS',CmdFmt(prng(0xF700300E),6,'UI','6B')),
        ('WLAN SSID of current BSS',CmdFmt(prng(0xF7003014),32,'S-ZT','32s')),
        ('WLAN Channel frequency (MHz)',CmdFmt(prng(0xF7003034),4,'UI','I')),
        ('WLAN Rate at which data is received (kbps)',CmdFmt(prng(0xF7003038),4,'UI','I')),
        ('WLAN Rate at which data is transmitted (kbps)',CmdFmt(prng(0xF700303C),4,'UI','I')),
        ('WLAN Signal to noise ratio (dB)',CmdFmt(prng(0xF7003040),4,'UI','I')),
        ('WLAN Signal level (dBm)',CmdFmt(prng(0xF7003044),4,'I','i')),
        ('WLAN Noise level (dBm)',CmdFmt(prng(0xF7003048),4,'I','i')),
        ('WLAN Number of missed beacon frames',CmdFmt(prng(0xF700304C),4,'UI','I')),
        ('WLAN Number of connection events',CmdFmt(prng(0xF7003050),4,'UI','I')),
        ('WLAN Number of disconnection events',CmdFmt(prng(0xF7003054),4,'UI','I')),
        ('WLAN Number of low signal strength events',CmdFmt(prng(0xF7003058),4,'UI','I')),
        ('WLAN Total packets received',CmdFmt(prng(0xF700305C),4,'UI','I')),
        ('WLAN Total packets transmitted',CmdFmt(prng(0xF7003060),4,'UI','I')),
        ('WLAN Total bytes received',CmdFmt(prng(0xF7003064),4,'UI','I')),
        ('WLAN Total bytes transmitted',CmdFmt(prng(0xF7003068),4,'UI','I')),
        ('WLAN Bad packets received',CmdFmt(prng(0xF700306c),4,'UI','I')),
        ('WLAN Packet transmit problems',CmdFmt(prng(0xF7003070),4,'UI','I')),
        ('WLAN Incoming packets dropped due to resource constraints',CmdFmt(prng(0xF7003074),4,'UI','I')),
        ('WLAN Outgoing packets dropped due to resource constraints',CmdFmt(prng(0xF7003078),4,'UI','I')),
        ('WLAN Received packets that were too long or too short',CmdFmt(prng(0xF700307c),4,'UI','I')),
        ('WLAN Received packets with wrong network ID/SSID',CmdFmt(prng(0xF7003080),4,'UI','I')),
        ('WLAN Unable to code/decode (WEP)',CmdFmt(prng(0xF7003084),4,'UI','I')),
        ('WLAN Can''t perform MAC reassembly',CmdFmt(prng(0xF7003088),4,'UI','I')),
        ('WLAN Maximum MAC retries reached',CmdFmt(prng(0xF700308C),4,'UI','I')),

        # WLAN CONFIGURATION—READ/WRITE
        ('WLAN Enable wireless network configuration',CmdFmt(prng(0xF7004000),4,'UI','I')),
        ('WLAN IP address',CmdFmt(prng(0xF7004004),4,'IP','4B')),
        ('WLAN Subnet mask',CmdFmt(prng(0xF7004008),4,'IP','4B')),
        ('WLAN Default Gateway IP address',CmdFmt(prng(0xF700400C),4,'IP','4B')),
        ('WLAN Secondary Gateway IP address',CmdFmt(prng(0xF7004010),4,'IP','4B')),
        ('WLAN Primary DNS Server IP address',CmdFmt(prng(0xF7004014),4,'IP','4B')),
        ('WLAN Secondary DNS Server IP address',CmdFmt(prng(0xF7004018),4,'IP','4B')),
        ('WLAN Service Set Identifier',CmdFmt(prng(0xF700401C),36,'S-ZT','36s')),
        ('0xF7004040',CmdFmt(prng(0xF7004040),4,'UI','I')),
        ('WLAN IEEE 802.11 operation mode',CmdFmt(prng(0xF7004044),4,'UI','I')),
        ('WLAN Security protocol',CmdFmt(prng(0xF7004048),4,'UI','I')),
        ('WLAN Authenticated key management protocol',CmdFmt(prng(0xF700404C),4,'UI','I')),
        ('WLAN IEEE 802.11 authentication algorithm',CmdFmt(prng(0xF7004050),4,'UI','I')),
        ('WLAN Unicast cipher for WPA',CmdFmt(prng(0xF7004054),4,'UI','I')),
        ('WLAN Broadcast/multicast cipher for WPA',CmdFmt(prng(0xF7004058),4,'UI','I')),
        ('WLAN WPA 256-bit pre-shared key',CmdFmt(prng(0xF700405C),68,'S-ZT','68s')),
        ('WLAN Static WEP key0:',CmdFmt(prng(0xF70040A0),36,'S-ZT','36s')),
        ('WLAN Static WEP key1:',CmdFmt(prng(0xF70040C4),36,'S-ZT','36s')),
        ('WLAN Static WEP key2:',CmdFmt(prng(0xF70040E8),36,'S-ZT','36s')),
        ('WLAN Static WEP key3:',CmdFmt(prng(0xF700410C),36,'S-ZT','36s')),
        ('WLAN Default WEP key:',CmdFmt(prng(0xF7004130),4,'UI','I')),
        ('WLAN Channel for ad-hoc mode.',CmdFmt(prng(0xF7004134),4,'UI','I')),
##        ('0xF700413C',CmdFmt(prng(0xF700413C),1708,'-','708x')),

        # WLAN ENABLE—READ/WRITE
        ('WLAN Enable.',CmdFmt(prng(0xF8000000),4,'B','I')),
        ('WLAN logging mode',CmdFmt(prng(0xF8000004),4,'UI','I')),
        ('WLAN Inactivity timeout',CmdFmt(prng(0xF8000008),4,'UI','I')),

        # IP SETTINGS—READ/WRITE
        ('ENET Primary address',CmdFmt(prng(0xFFFFF008),8,'IP','4H')),
        ('ENET Primary Subnet mask',CmdFmt(prng(0xFFFFF010),8,'IP','4H')),
        ('ENET Primary default gateway address',CmdFmt(prng(0xFFFFF018),8,'IP','4H')),
        ('ENET Primary DNS server address',CmdFmt(prng(0xFFFFF020),8,'IP','4H')),
        ('ENET 0xFFFFF028',CmdFmt(prng(0xFFFFF028),64,'-','64x')),
        ('ENET Secondary address',CmdFmt(prng(0xFFFFF050),8,'IP','4H')),
        ('ENET Secondary subnet mask',CmdFmt(prng(0xFFFFF058),8,'IP','4H')),
        ('ENET Secondary MAC address',CmdFmt(prng(0xFFFFF060),8,'IP','4H')),
        ('ENET Secondary default gateway address',CmdFmt(prng(0xFFFFF068),8,'IP','4H')),
        ('ENET Secondary DNS server address',CmdFmt(prng(0xFFFFF070),8,'IP','4H')),
        ])

om = OmmpNET()
