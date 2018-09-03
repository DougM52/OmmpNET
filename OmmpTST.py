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

import OmmpNET
import OmmpUTL as utl

def test_digital_bank_write(om,aa):
    """
    sample routine showing how to set and read back memory map values
    """
    # send a power up clear
    print('sent a power up clear',om.power_up_clear(aa))
    for i in range(12):
        if i & 4 == 0:
            print('P{:d}.{:d} set to output'.format(i>>2,i&3),om.set_point_type(aa,i,0x180))
            print('P{:d}.{:d} activated'.format(i>>2,i&3),om.activate_digital_point(aa,i))
    rsp = om.get_digital_points_state(aa)
    mask = rsp.data[0]
    print('get_digital_points_state',rsp)
    rsp = om.deactivate_digital_points(aa,mask)
    print('deactivate_digital_points',rsp)
    rsp = om.get_digital_points_state(aa)
    print('get_digital_points_state',rsp)
    rsp = om.activate_digital_points(aa,mask)
    print('activate_digital_points',rsp)
    rsp = om.get_digital_points_state(aa)
    print('get_digital_points_state',rsp)
            
def generate_10_hz_sqw_on_point_0(om,aa):
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
    print('send a power up clear',om.power_up_clear(aa))
    # Set first point to an Output
    print('make digital point 0 an output',om.set_point_type(aa,0,0x180))
    # turn Off point P0.0
    print('turn off digital point 0',om.deactivate_digital_point(aa,0))
    # Set up a 50ms timer
    print('set t0 delay',om.set_timer_delay(aa,0,50))
    print('get t0 delay',om.get_timer_delay(aa,0))
    # start it when P0.0 Sets
    print('set t0 trigger on mask',om.set_timer_trigger_on_mask(aa,0,1))
    print('get t0 trigger on mask',om.get_timer_trigger_on_mask(aa,0))
    # use Event to clear P0.0
    print('set t0 reaction off mask',om.set_timer_reaction_off_mask(aa,0,1))
    print('get t0 reaction off mask',om.get_timer_reaction_off_mask(aa,0))
    # Set up a 50ms timer
    print('set t1 delay',om.set_timer_delay(aa,1,50))
    print('get t1 delay',om.get_timer_delay(aa,1))
    # start it when P0.0 clears
    print('set t1 trigger off mask',om.set_timer_trigger_off_mask(aa,1,1))
    print('get t1 trigger off mask',om.get_timer_trigger_off_mask(aa,1))
    # use Event to Set P0.0
    print('set t1 reaction on mask',om.set_timer_reaction_on_mask(aa,1,1))
    print('get t1 reaction on mask',om.get_timer_reaction_on_mask(aa,1))
    # turn On P0.0 to start timer 0
    print('turn on digital point 0 to start sqw',om.activate_digital_point(aa,0))

        
def get_analog_point_types(om,aa):
    for i in range(32):
        print(i,om.get_point_type(aa,i))

if __name__ == "__main__":                 
    # createthe OmmpNET object
    om = OmmpNET.OmmpNET()
    aa = ('192.168.10.225',2001)


