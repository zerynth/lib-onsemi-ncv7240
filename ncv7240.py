"""
.. module:: ncv7240

**************
NCV7240 Module
**************

.. _datasheet: https://www.onsemi.com/pub/Collateral/NCV7240-D.PDF

This module contains the Zerynth driver for the OnSemi NCV7240 automotive 
eight channel low-side/relay driver, that can be controlled with a standard SPI
interface or parallel input pins.

Each output driver is protected for over load current and includes an
output clamp for inductive loads. Open load or shorts to ground can be detected
when the driver is in the off state.

    """

import spi

class NCV7240(spi.Spi):
    """
.. class:: NCV7240

    The NCV7240 class implements several methods to access the I/O expander over
    an SPI bus and easily control its features.
    
    Each one of the 8 channels can be configured in one of 4 modes:
    
        * **Standby** (default) - the channel is disabled
        * **On** - the output driver is active (low), with over-load fault detection
        * **Off** - the output driver is inactive, with open-load fault detection
        * **Input** - the output driver on/off state is controlled by input pins (PWM capable)
    
    :note: Faults are latched and must be cleared by switching the channel to Standby mode to attempt recovery.
    :note: Off mode and open-load detection can be used as way to read channels as digital inputs.
    """

    CH_STANDBY = 0b00
    CH_INPUT = 0b01
    CH_ON = 0b10
    CH_OFF = 0b11

    CH_OPEN = 0b10
    CH_FAULT = 0b01

    _CH_MASK = 0b11
    _ALL_MASK = 0b01010101

    def __init__(self, spidrv, cs, clk=500000):
        """
.. method:: __init__(spidrv, cs, clk=500000)

        Creates an instance of NCV7240 class, using the specified SPI settings.

        :param spidrv: the *SPI* driver to use (SPI0, ...)
        :param cs: Chip select pin to access the NCV7240 chip
        :param clk: Clock speed, default 500 kHz
        
        :note: SPI mode is fixed by the slave chip protocol (CPOL=0,CPHA=1)
        """
        self.reg = bytearray(2)
        self.cs = cs
        self.spidrv = spidrv
        self.clock = clk
        spi.Spi.__init__(self, self.cs, self.spidrv, clock=self.clock, mode=spi.SPI_MODE_LOW_SECOND)

    def _write(self):
        # print("reg: %x%x"%(self.reg[0],self.reg[1]))
        ex = None
        try:
            self.lock()
            self.select()
            self.write(self.reg)
        except Exception as e:
            ex = e
        self.unselect()
        self.done()
        self.unlock()
        if ex is not None:
            raise ex

    def _exchange(self):
        ex = None
        try:
            self.lock()
            self.select()
            stat = self.exchange(self.reg)
        except Exception as e:
            ex = e
        self.unselect()
        self.done()
        self.unlock()
        if ex is not None:
            raise ex
        # print("stat: %x%x"%(stat[0],stat[1]))
        return stat

    def _update(self, channel, mode):
        channel *= 2
        if channel < 8:
            self.reg[1] = (self.reg[1] & ~(NCV7240._CH_MASK << channel)) | (mode << channel)
        else:
            channel -= 8
            self.reg[0] = (self.reg[0] & ~(NCV7240._CH_MASK << channel)) | (mode << channel)

    def setChannel(self, channel, mode):
        """
.. method:: setChannel(channel, mode)

        Set one channel to the specified mode.

        :param channel: Index of the channel (0-7) 
        :param mode: One of the four possible modes: ``CH_STANDBY``, ``CH_ON``, ``CH_OFF``, ``CH_INPUT``
        """
        self._update(channel, mode)
        self._write()

    def setAllChannels(self, mode):
        """
.. method:: setAllChannels(mode)

        Set all channels to the specified mode.

        :param mode: One of the four possible modes: ``CH_STANDBY``, ``CH_ON``, ``CH_OFF``, ``CH_INPUT``
        
        :note: If *mode* is a list or a tuple, each element specifies the corresponding channel mode
        """
        if type(mode) in (PLIST,PTUPLE):
            i = 0
            while i < len(mode):
                if mode[i] is not None:
                    self._update(i,mode[i])
                i += 1
            self._write()
        else:
            self.reg[0] = mode * NCV7240._ALL_MASK
            self.reg[1] = mode * NCV7240._ALL_MASK
            self._write()

    def getChannel(self, channel):
        """
.. method:: getChannel(channel)

        Get the status of the specified channel.

        :param channel: Index of the channel (0-7) 
        
        :returns: One of the two possible states: ``CH_OPEN``, ``CH_FAULT``

        :note: Put the channel in Standby mode to clear status.
        """
        stat = self._exchange()
        channel *= 2
        if channel < 8:
            return (stat[1] >> channel) & NCV7240._CH_MASK
        else:
            channel -= 8
            return (stat[0] >> channel) & NCV7240._CH_MASK

    def getAllChannels(self):
        """
.. method:: getAllChannels()

        Get the status of all channels.

        :returns: A tuple where each element specifies the corresponding channel state: ``CH_OPEN`` or ``CH_FAULT``

        :note: Put the channel in Standby mode to clear status.
        """
        stat = self._exchange()
        return (
            (stat[1] >> 0) & NCV7240._CH_MASK,
            (stat[1] >> 2) & NCV7240._CH_MASK,
            (stat[1] >> 4) & NCV7240._CH_MASK,
            (stat[1] >> 6) & NCV7240._CH_MASK,
            (stat[0] >> 0) & NCV7240._CH_MASK,
            (stat[0] >> 2) & NCV7240._CH_MASK,
            (stat[0] >> 4) & NCV7240._CH_MASK,
            (stat[0] >> 6) & NCV7240._CH_MASK
        )

    def channelInputMode(self, channel):
        """
.. method:: channelInputMode(channel)

        Set the specified channel to input mode (output driver state is controlled directly by the corresponding input pin on the NCV7240 chip).

        :param channel: Index of the channel (0-7) 
        """
        self.setChannel(channel, NCV7240.CH_INPUT)

    def channelStandby(self, channel):
        """
.. method:: channelStandby(channel)

        Set the specified channel to standby mode (output driver and fault detection are disabled, fault status cleared).

        :param channel: Index of the channel (0-7) 
        """
        self.setChannel(channel, NCV7240.CH_STANDBY)

    def channelOn(self, channel):
        """
.. method:: channelOn(channel)

        Switch the specified channel output driver on (polarity active-low, over-load/over-temperature fault detection is active).

        :param channel: Index of the channel (0-7) 
        """
        self.setChannel(channel, NCV7240.CH_ON)

    def channelOff(self, channel):
        """
.. method:: channelOff(channel)

        Switch the specified channel output driver off (open-load fault detection current is active).

        :param channel: Index of the channel (0-7) 
        """
        self.setChannel(channel, NCV7240.CH_OFF)

    def isChannelOpen(self, channel):
        """
.. method:: isChannelOpen(channel)

        Returns whether the channel status indicates an open-load fault (only meaningful when the channel is off).

        :param channel: Index of the channel (0-7) 

        :note: Put the channel in Standby mode to clear fault status.
        """
        return (self.getChannel(channel) & NCV7240.CH_OPEN) != 0

    def isChannelFault(self, channel):
        """
.. method:: isChannelFault(channel)

        Returns whether the channel status indicates an over-current/over-temperature fault (only meaningful when the channel is on).

        :param channel: Index of the channel (0-7) 

        :note: Put the channel in Standby mode to clear fault status.
        """
        return (self.getChannel(channel) & NCV7240.CH_FAULT) != 0
