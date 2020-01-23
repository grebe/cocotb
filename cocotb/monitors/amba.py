import warnings

from cocotb.amba import AXI4StreamBus
from cocotb.utils import hexdump
from cocotb.decorators import coroutine
from cocotb.monitors import BusMonitor
from cocotb.triggers import RisingEdge, ReadOnly
from cocotb.binary import BinaryValue

class AmbaProtocolError(Exception):
    pass

class AXI4StreamMonitor(BusMonitor):
    _bus_type = AXI4StreamBus

    _default_config = {}

    def __init__(self, entity, name, clock, **kwargs):
        config = kwargs.pop('config', {})
        report_channel = kwargs.pop('report_channel', False)
        BusMonitor.__init__(self, entity, name, clock, **kwargs)

        self.config = self._default_config.copy()
        self.report_channel = report_channel

        for configoption, value in config.items():
            self.config[configoption] = value

            self.log.debug("Setting config option %s to %s", configoption, str(value))

    @coroutine
    def _monitor_recv(self):
        clkedge = RisingEdge(self.clock)
        rdonly = ReadOnly()

        def valid():
            if hasattr(self.bus, "TREADY"):
                return self.bus.TVALID.value and self.bus.TREADY.value
            else:
                return self.bus.TVALID.value

        while True:
            yield clkedge
            yield rdonly
            if valid():
                vec = self.bus.TDATA.value
                self._recv(vec.buff)
