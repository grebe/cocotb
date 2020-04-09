# Copyright (c) 2014 Potential Ventures Ltd
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Potential Ventures Ltd,
#       SolarFlare Communications Inc nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL POTENTIAL VENTURES LTD BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Drivers for Advanced Microcontroller Bus Architecture."""

import cocotb
from cocotb.decorators import coroutine
from cocotb.amba import AXI4Bus, AXI4LiteBus, AXI4StreamBus
from cocotb.triggers import RisingEdge, ReadOnly, Lock
from cocotb.drivers import BusDriver, ValidatedBusDriver
from cocotb.result import ReturnValue
from cocotb.binary import BinaryValue

import array


class AXIProtocolError(Exception):
    pass

class AXI4StreamMaster(ValidatedBusDriver):
    """AXI4-Stream Master Driver"""

    _bus_type = AXI4StreamBus
    _default_config = {}

    def __init__(self, entity, name, clock, big_endian=False, **kwargs):
        self.big_endian = big_endian

        config = kwargs.pop('config', {})
        ValidatedBusDriver.__init__(self, entity, name, clock, **kwargs)

        self.config = AXI4StreamMaster._default_config.copy()

        for configoption, value in config.items():
            self.config[configoption] = value

            self.log.debug("Setting config option %s to %s", configoption, str(value))

        self.bus.TVALID <= 0
        if hasattr(self.bus, "TDATA"):
            self.bus.TDATA  <= BinaryValue(n_bits=len(self.bus.TDATA),
                                           value="x" * len(self.bus.TDATA),
                                           bigEndian=self.big_endian)
        if hasattr(self.bus, "TKEEP"):
            self.bus.TKEEP  <= BinaryValue(n_bits=len(self.bus.TKEEP),
                                           value="1"*len(self.bus.TKEEP))
        if hasattr(self.bus, "TSTRB"):
            self.bus.TSTRB  <= BinaryValue(n_bits=len(self.bus.TSTRB),
                                           value="1"*len(self.bus.TSTRB))

        if hasattr(self.bus, "TLAST"):
            self.bus.TLAST.setimmediatevalue(0)
        if hasattr(self.bus, "TID"):
            self.bus.TID.setimmediatevalue(0)
        if hasattr(self.bus, "TDEST"):
            self.bus.TDEST.setimmediatevalue(0)
        if hasattr(self.bus, "TUSER"):
            self.bus.TUSER.setimmediatevalue(0)

    @coroutine
    def _wait_ready(self):
        yield ReadOnly()
        while not self.bus.TREADY.value:
            yield RisingEdge(self.clock)
            yield ReadOnly()

    @coroutine
    def _driver_send(self, data=None, strb=None, keep=None, last=None, id=None, dest=None, user=None, sync=True):
        self.log.debug(f"Sending AXI4-Stream transmission: {data}")
        clkedge = RisingEdge(self.clock)

        if hasattr(self.bus, "TDATA"):
            dataword = BinaryValue(n_bits=len(self.bus.TDATA), bigEndian=self.big_endian)
        if hasattr(self.bus, "TSTRB"):
            strbword = BinaryValue(n_bits=len(self.bus.TSTRB))
        if hasattr(self.bus, "TKEEP"):
            keepword = BinaryValue(n_bits=len(self.bus.TKEEP))
        if hasattr(self.bus, "TLAST"):
            lastword = BinaryValue(n_bits=1)
        if hasattr(self.bus, "TID"):
            idword = BinaryValue(n_bits=len(self.bus.TID))
        if hasattr(self.bus, "TDEST"):
            destword = BinaryValue(n_bits=len(self.bus.TDEST))
        if hasattr(self.bus, "TUSER"):
            userword = BinaryValue(n_bits=len(self.bus.TUSER))

        self.bus.TVALID <= 0
        if sync:
            yield clkedge
        # if not self.on:
        #     self.bus.TVALID <= 0
        #     for _ in range(self.off):
        #         yield clkedge
        #     self._next_valids()

        # if self.on is not True and self.on:
        #     self.on -= 1

        self.bus.TVALID <= 1
        if hasattr(self.bus, "TDATA"):
            # print(f"Assigning {data}")
            dataword.assign(data)
            self.bus.TDATA <= dataword
        if hasattr(self.bus, "TSTRB"):
            strbword.assign(strb)
            self.bus.TSTRB <= strbword
        if hasattr(self.bus, "TKEEP"):
            keepword.assign(keep)
            self.bus.TKEEP <= keepword
        if hasattr(self.bus, "TLAST"):
            lastword.assign(last)
            self.bus.TLAST <= lastword
        if hasattr(self.bus, "TID"):
            idword.assign(id)
            self.bus.TID <= idword
        if hasattr(self.bus, "TDEST"):
            destword.assign(dest)
            self.bus.TDEST <= destword
        if hasattr(self.bus, "TUSER"):
            userword.assign(user)
            self.bus.TUSER <= userword

        if hasattr(self.bus, "TREADY"):
            yield self._wait_ready()
        yield clkedge
        self.bus.TVALID <= 0
        # if hasattr(self.bus, "TDATA"):
        #     dataword.binstr = "x" * len(self.bus.TDATA)

    def _send_string(self, string, sync=True, id=None, dest=None):
        if not hasattr(self.bus, "TDATA"):
            raise AttributeError("Cannot send string without TDATA")
        clkedge = RisingEdge(self.clock)
        n = int((len(self.bus.TDATA) + 7)/ 8)
        strb = None
        if hasattr(self.bus, "TSTRB"):
            strb = "1" * len(self.bus.TSTRB)
        keep = None
        if hasattr(self.bus, "TKEEP"):
            keep = "1" * len(self.bus.TKEEP)
        last = None
        if hasattr(self.bus, "TLAST"):
            last = False
        id = None
        if hasattr(self.bus, "TID"):
            id = 0
        dest = None
        if hasattr(self.bus, "TDEST"):
            dest = 0
        user = None
        if hasattr(self.bus, "TUSER"):
            user = 0

        while string:
            nbytes = min(len(string), n)
            data = string[:nbytes]
            if len(string) <= n:
                if hasattr(self.bus, "TLAST"):
                    last = True
                if hasattr(self.bus, "TSTRB"):
                    strb = "0" * (n - len(string)) + "1" * len(string)
                string = ""
            else:
                string = string[n:]
            self._driver_send(data=data, strb=strb, keep=keep, last=last, id=id, dest=dest, user=user, sync=sync)

class AXI4LiteMaster(BusDriver):
    """AXI4-Lite Master.

    TODO: Kill all pending transactions if reset is asserted.
    """
    _bus_type = AXI4LiteBus


    def __init__(self, entity, name, clock, **kwargs):
        BusDriver.__init__(self, entity, name, clock, **kwargs)

        # Drive some sensible defaults (setimmediatevalue to avoid x asserts)
        self.bus.AWVALID.setimmediatevalue(0)
        self.bus.WVALID.setimmediatevalue(0)
        self.bus.ARVALID.setimmediatevalue(0)
        self.bus.BREADY.setimmediatevalue(1)
        self.bus.RREADY.setimmediatevalue(1)

        # Mutex for each channel that we master to prevent contention
        self.write_address_busy = Lock("%s_wabusy" % name)
        self.read_address_busy = Lock("%s_rabusy" % name)
        self.write_data_busy = Lock("%s_wbusy" % name)

    @cocotb.coroutine
    def _send_write_address(self, address, delay=0):
        """
        Send the write address, with optional delay (in clocks)
        """
        yield self.write_address_busy.acquire()
        for cycle in range(delay):
            yield RisingEdge(self.clock)

        self.bus.AWADDR <= address
        self.bus.AWVALID <= 1

        while True:
            yield ReadOnly()
            if self.bus.AWREADY.value:
                break
            yield RisingEdge(self.clock)
        yield RisingEdge(self.clock)
        self.bus.AWVALID <= 0
        self.write_address_busy.release()

    @cocotb.coroutine
    def _send_write_data(self, data, delay=0, byte_enable=0xF):
        """Send the write address, with optional delay (in clocks)."""
        yield self.write_data_busy.acquire()
        for cycle in range(delay):
            yield RisingEdge(self.clock)

        self.bus.WDATA <= data
        self.bus.WVALID <= 1
        self.bus.WSTRB <= byte_enable

        while True:
            yield ReadOnly()
            if self.bus.WREADY.value:
                break
            yield RisingEdge(self.clock)
        yield RisingEdge(self.clock)
        self.bus.WVALID <= 0
        self.write_data_busy.release()

    @cocotb.coroutine
    def write(self, address, value, byte_enable=0xf, address_latency=0,
              data_latency=0, sync=True):
        """Write a value to an address.

        Args:
            address (int): The address to write to.
            value (int): The data value to write.
            byte_enable (int, optional): Which bytes in value to actually write.
                Default is to write all bytes.
            address_latency (int, optional): Delay before setting the address (in clock cycles).
                Default is no delay.
            data_latency (int, optional): Delay before setting the data value (in clock cycles).
                Default is no delay.
            sync (bool, optional): Wait for rising edge on clock initially.
                Defaults to True.

        Returns:
            BinaryValue: The write response value.

        Raises:
            AXIProtocolError: If write response from AXI is not ``OKAY``.
        """
        if sync:
            yield RisingEdge(self.clock)

        c_addr = cocotb.fork(self._send_write_address(address,
                                                      delay=address_latency))
        c_data = cocotb.fork(self._send_write_data(value,
                                                   byte_enable=byte_enable,
                                                   delay=data_latency))

        if c_addr:
            yield c_addr.join()
        if c_data:
            yield c_data.join()

        # Wait for the response
        while True:
            yield ReadOnly()
            if self.bus.BVALID.value and self.bus.BREADY.value:
                result = self.bus.BRESP.value
                break
            yield RisingEdge(self.clock)

        yield RisingEdge(self.clock)


        if int(result):
            raise AXIProtocolError("Write to address 0x%08x failed with BRESP: %d"
                               % (address, int(result)))

        raise ReturnValue(result)

    @cocotb.coroutine
    def read(self, address, sync=True):
        """Read from an address.

        Args:
            address (int): The address to read from.
            sync (bool, optional): Wait for rising edge on clock initially.
                Defaults to True.

        Returns:
            BinaryValue: The read data value.

        Raises:
            AXIProtocolError: If read response from AXI is not ``OKAY``.
        """
        if sync:
            yield RisingEdge(self.clock)

        self.bus.ARADDR <= address
        self.bus.ARVALID <= 1

        while True:
            yield ReadOnly()
            if self.bus.ARREADY.value:
                break
            yield RisingEdge(self.clock)

        yield RisingEdge(self.clock)
        self.bus.ARVALID <= 0

        while True:
            yield ReadOnly()
            if self.bus.RVALID.value and self.bus.RREADY.value:
                data = self.bus.RDATA.value
                result = self.bus.RRESP.value
                break
            yield RisingEdge(self.clock)

        if int(result):
            raise AXIProtocolError("Read address 0x%08x failed with RRESP: %d" %
                               (address, int(result)))

        raise ReturnValue(data)

    def __len__(self):
        return 2**len(self.bus.ARADDR)

class AXI4Slave(BusDriver):
    '''
    AXI4 Slave

    Monitors an internal memory and handles read and write requests.
    '''
    _bus_type = AXI4Bus

    def __init__(self, entity, name, clock, memory, callback=None, event=None,
                 big_endian=False, **kwargs):

        BusDriver.__init__(self, entity, name, clock, **kwargs)
        self.clock = clock

        self.big_endian = big_endian
        self.bus.ARREADY.setimmediatevalue(1)
        self.bus.RVALID.setimmediatevalue(0)
        self.bus.RLAST.setimmediatevalue(0)
        self.bus.AWREADY.setimmediatevalue(1)
        self.bus.WREADY.setimmediatevalue(1)
        self.bus.BVALID.setimmediatevalue(0)
        for i in ("BID", "RID", "BRESP", "RRESP"):
            if hasattr(self.bus, i):
                getattr(self.bus, i).setimmediatevalue(0)

        self._memory = memory

        self.write_address_busy = Lock("%s_wabusy" % name)
        self.read_address_busy = Lock("%s_rabusy" % name)
        self.write_data_busy = Lock("%s_wbusy" % name)

        cocotb.fork(self._read_data())
        cocotb.fork(self._write_data())

    def _size_to_bytes_in_beat(self, AxSIZE):
        if AxSIZE < 7:
            return 2 ** AxSIZE
        return None

    @cocotb.coroutine
    def _write_data(self):
        clock_re = RisingEdge(self.clock)

        while True:
            self.bus.AWREADY <= 1
            self.bus.WREADY <= 0
            while True:
                yield ReadOnly()
                # self.bus.WREADY <= 0
                if self.bus.AWVALID.value:
                    # self.bus.WREADY <= 1
                    break
                yield clock_re

            yield ReadOnly()
            _awaddr = int(self.bus.AWADDR)
            _awlen = int(self.bus.AWLEN)
            _awsize = int(self.bus.AWSIZE)
            _awburst = int(self.bus.AWBURST)
            _awprot = int(self.bus.AWPROT)

            burst_length = _awlen + 1
            bytes_in_beat = self._size_to_bytes_in_beat(_awsize)

            if __debug__:
                self.log.debug(
                    "AWADDR  %d\n" % _awaddr +
                    "AWLEN   %d\n" % _awlen +
                    "AWSIZE  %d\n" % _awsize +
                    "AWBURST %d\n" % _awburst +
                    "BURST_LENGTH %d\n" % burst_length +
                    "Bytes in beat %d\n" % bytes_in_beat)

            burst_count = burst_length

            yield clock_re

            self.bus.AWREADY <= 0
            self.bus.WREADY <= 1

            while True:
                if self.bus.WVALID.value:
                    word = self.bus.WDATA.value
                    word.big_endian = self.big_endian
                    _burst_diff = burst_length - burst_count
                    _st = _awaddr + (_burst_diff * bytes_in_beat)  # start
                    _end = _awaddr + ((_burst_diff + 1) * bytes_in_beat)  # end
                    # _memrange = bytearray(word.get_buff(), 'ascii')
                    _memrange = word.get_binstr()
                    _memrange = int(_memrange, 2).to_bytes(len(_memrange) // 8, byteorder = 'little')
                    _memrange = list(_memrange)
                    # print(_memrange)
                    # print(f"bytes_in_beat = {bytes_in_beat}")
                    # print(self._memory[_st:_end])
                    # array.array('b')
                    # _memrange.frombytes(word.get_buff())
                    self._memory[_st:_end] = _memrange
                    # array.array('B', word.get_buff())
                    burst_count -= 1
                    if burst_count == 0:
                        break
                yield clock_re

    @cocotb.coroutine
    def _read_data(self):
        clock_re = RisingEdge(self.clock)

        while True:
            self.bus.ARREADY <= 1
            self.bus.RVALID <= 0
            while True:
                yield ReadOnly()
                if self.bus.ARVALID.value:
                    break
                yield clock_re

            # yield ReadOnly()
            _araddr = int(self.bus.ARADDR)
            _arlen = int(self.bus.ARLEN)
            _arsize = int(self.bus.ARSIZE)
            _arburst = int(self.bus.ARBURST)
            _arprot = int(self.bus.ARPROT)

            burst_length = _arlen + 1
            bytes_in_beat = self._size_to_bytes_in_beat(_arsize)

            word = BinaryValue(n_bits=bytes_in_beat*8, bigEndian=self.big_endian)

            if __debug__:
                self.log.debug(
                    "ARADDR  %d\n" % _araddr +
                    "ARLEN   %d\n" % _arlen +
                    "ARSIZE  %d\n" % _arsize +
                    "ARBURST %d\n" % _arburst +
                    "BURST_LENGTH %d\n" % burst_length +
                    "Bytes in beat %d\n" % bytes_in_beat)

            burst_count = burst_length

            yield clock_re

            self.bus.ARREADY <= 0
            self.bus.RVALID <= 1
            self.bus.RLAST <= 0
            while burst_count > 0:
                if self.bus.RREADY.value:
                    _burst_diff = burst_length - burst_count
                    _st = _araddr + (_burst_diff * bytes_in_beat)
                    _end = _araddr + ((_burst_diff + 1) * bytes_in_beat)
                    byteorder = 'little'
                    if self.big_endian:
                        byteorder = 'big'
                    word.integer = int.from_bytes(self._memory[_st:_end].tostring(), byteorder=byteorder)
                    self.bus.RDATA <= word
                    if burst_count == 1:
                        self.bus.RLAST <= 1
                    burst_count -= 1
                yield clock_re
            self.bus.RVALID <= 0
