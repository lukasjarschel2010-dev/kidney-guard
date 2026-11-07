#!/usr/bin/env python3
"""Live plot of the 1 kHz accelerometer stream from Main/kidney_guard.

kidney_guard streams only on request: the plotter sends 's' when it connects
and 't' (back to text output) when it closes.

-r lowers the rate kidney_guard streams at (the sensor itself keeps running
at 1 kHz). Each streamed sample is the one with the largest |a| of its
group, so impact peaks survive, and each FSR is its lowest reading (hardest
press); the peak's time is only accurate to one streamed sample period, though.

Shows the last few seconds of X/Y/Z, and the FSRs on D8, D9 and D10 when the
sketch sends them (kidney_guard does), drawn as conductance, which is roughly
proportional to force, on a fixed axis that gives most of its height to strong
presses. Optionally records every sample to CSV. Close the
Arduino Serial Monitor first, only one program can open the port.

Usage:
    python3 Tools/plot_stream.py                  # auto-detect port
    python3 Tools/plot_stream.py -o impact.csv    # also record to CSV
    python3 Tools/plot_stream.py -p /dev/cu.usbmodem1101 -w 5
    python3 Tools/plot_stream.py -r 100           # stream 100 samples/sec

X/Y/Z are drawn flat at rest: samples within --noise-band of an
axis's resting value draw as that value, and anything further out draws at
its true raw value, so impact heights are kept. They are then shifted so each
axis rests at 0, cancelling gravity and the sensor's zero-g offset. Use
--noise-band 0 for the raw signal. Either way X/Y/Z draw as absolute values,
so the graph shows only positive values, on a fixed log scale that doesn't
resize. The CSV is always raw.

Since the sign is lost that way, the largest negative swing of each impact
on each axis is labelled "neg" in the axis's color.
An impact is a run of samples at least --label-min g from rest (after the
noise band) that ends once the axis stays under it for PEAK_REARM_S. Use
--label-min 0 to turn the labels off.

Keys: space = pause/resume the view (recording continues).

Requires:  python3 -m pip install -r Tools/requirements.txt
"""
import argparse
import collections
import glob
import os
import struct
import sys
import threading
import time

import numpy as np
import serial

MAX_SAMPLE_RATE_HZ = 1000         # setODR() in the sketch, sizes the buffer
SYNC0 = 0xA5
# The byte after SYNC0 names the packet layout, as int16 values per sample:
#  0x5B  x, y, z                       (older layout, no FSRs)
#  0x5C  x, y, z, fsr D8, D9, D10      (kidney_guard)
LAYOUTS = {0x5B: 3, 0x5C: 6}
NOISE_BAND_G = 6.0                 # largest resting noise measured: 5.75 g
BASELINE_TAU_S = 3.0               # how fast the resting value follows a tilt
LEVEL_STEP_G = 0.3                 # drawn level moves only for a shift this big
ACCEL_LOG_MIN_G = 1.0              # bottom of the log axis, rest draws here
ACCEL_LOG_MAX_G = 300.0            # top: 200 g full scale plus resting offsets
PEAK_LABEL_MIN_G = 20.0            # label negative peaks at least this far out
PEAK_REARM_S = 0.02                # an impact ends after this long under it,
                                   #  like IMPACT_REARM_MS in the sketch
AXIS_NAMES = ("x", "y", "z")
AXIS_COLORS = ("#e5534b", "#57ab5a", "#539bf5")
FSR_COUNT = 3
FSR_ADC_MAX = 4095                 # the reading at rest
FSR_PULLUP_KOHM = 4.7
FSR_MAX_CONDUCTANCE = 1.1          # top of the axis in 1/kohm: the hardest press
                                   #  recorded, a reading of 663
FSR_NAMES = ("D8", "D9", "D10")
# Press bands on the plot, at or below. Set from one held press at each level
#  (medians 3136, 1645, 771), midway between them in FSR resistance. Plot only:
#  the sketch's own "Heavy Squeeze" threshold is separate.
FSR_MEDIUM_ADC = 2450
FSR_HEAVY_ADC = 1160
HEADER = struct.Struct("<HIHBB")   # seq, micros, fullScale, count, overruns
MAX_COUNT = 64                   # sanity limit when resyncing
OFFSET_WINDOW_S = 5.0              # see StreamReader.offsets
VIEW_DELAY_S = 0.05                # the view trails the board by this much
                                   #  (or a packet interval if longer), so
                                   #  the newest data is already in


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        sys.exit("no /dev/cu.usbmodem* port found, is the board connected?")
    if len(ports) > 1:
        sys.exit(f"several ports found, pick one with -p: {', '.join(ports)}")
    return ports[0]


class PacketParser:
    """Turns the raw byte stream into (seq, micros, overruns, samples_g, fsr).

    micros is the board's micros() when the packet's last sample was read.
    fsr is a (count, 3) array of ADC readings, NaN for a layout without them.

    Bytes that don't form a packet with a valid checksum are skipped one at
    a time until the next sync, so startup noise or a packet the board cut
    short costs only that packet.
    """

    def __init__(self):
        self.buf = bytearray()
        self.bad_packets = 0

    def feed(self, data):
        self.buf += data
        packets = []
        while True:
            start = self.buf.find(SYNC0)
            if start < 0:
                self.buf.clear()
                return packets
            del self.buf[:start]
            if len(self.buf) < 2:
                return packets  # keep a trailing 0xA5, it may be a sync
            values = LAYOUTS.get(self.buf[1])
            if values is None:
                self._skip()
                continue
            if len(self.buf) < 2 + HEADER.size:
                return packets
            seq, micros, full_scale, count, overruns = HEADER.unpack_from(
                self.buf, 2)
            if not 0 < count <= MAX_COUNT or full_scale == 0:
                self._skip()
                continue
            size = 2 + HEADER.size + count * values * 2 + 1
            if len(self.buf) < size:
                return packets
            if sum(self.buf[2:size - 1]) & 0xFF != self.buf[size - 1]:
                self._skip()
                continue
            # bytes() copies, a view would stop buf from being resized
            counts = np.frombuffer(bytes(self.buf[2 + HEADER.size:size - 1]),
                                   "<i2").reshape(count, values)
            samples_g = counts[:, :3] * (full_scale / 2047.0)
            if values > 3:
                fsr = counts[:, 3:].astype(float)
            else:
                fsr = np.full((count, FSR_COUNT), np.nan)
            packets.append((seq, micros, overruns, samples_g, fsr))
            del self.buf[:size]

    def _skip(self):
        self.bad_packets += 1
        del self.buf[:1]


class StreamReader(threading.Thread):
    """Reads the port, timestamps samples, and records to CSV.

    Time comes from the board's micros() rather than arrival time, so USB
    jitter doesn't distort the plot, the real sensor rate is used instead
    of the nominal 1 kHz, and dropped packets leave a gap of the right
    length. t_s is the board's uptime in seconds.
    """

    def __init__(self, port, csv_path, decimation):
        super().__init__(daemon=True)
        self.port = port
        self.decimation = decimation
        self.csv = open(csv_path, "w") if csv_path else None
        if self.csv:
            self.csv.write("t_s,x_g,y_g,z_g,mag_g,"
                           + ",".join(f"fsr_{n.lower()}" for n in FSR_NAMES)
                           + "\n")
        self.parser = PacketParser()
        self.lock = threading.Lock()
        self.chunks = collections.deque()  # (t, xyz, fsr, gap, reset), GUI
        self.samples = 0
        self.dropped_packets = 0
        self.overruns = 0
        self.error = None
        self.last_seq = None
        self.last_micros = None
        self.micros_base = 0      # micros() wraps every ~71.6 minutes
        self.last_end_us = None   # unwrapped time of the previous packet
        # Until we've measured it
        self.period_us = 1e6 * decimation / MAX_SAMPLE_RATE_HZ
        self.packet_s = self.period_us / 1e6  # time between packets
        # Host monotonic time minus board time, the smallest seen over the
        #  last OFFSET_WINDOW_S: the packet that arrived with the least
        #  delay. The GUI scrolls by it instead of jumping to each packet.
        self.offsets = collections.deque()  # (arrival, offset)
        self.offset = None
        self.stopping = threading.Event()

    def run(self):
        try:
            with serial.Serial(self.port, 115200, timeout=0.1) as s:
                s.reset_input_buffer()
                # kidney_guard: set the rate (always, so an earlier -r
                #  doesn't stick), then start streaming
                s.write(f"r{self.decimation}\n".encode())
                s.write(b"s")
                while not self.stopping.is_set():
                    data = s.read(max(s.in_waiting, 1))
                    if data:
                        self._handle(self.parser.feed(data))
                s.write(b"t")  # kidney_guard: back to text output
        except serial.SerialException as e:
            self.error = str(e)
        finally:
            if self.csv:
                self.csv.close()

    def _handle(self, packets):
        arrival = time.monotonic()
        for seq, micros, overruns, xyz, fsr in packets:
            count = len(xyz)
            gap = reset = False
            if self.last_seq is not None:
                step = (seq - self.last_seq) & 0xFFFF
                if step > 0x8000:
                    # seq went backwards: the board restarted, so its time
                    #  starts over too (CSV t_s jumps back to ~0 here)
                    reset = True
                elif step != 1:
                    gap = True
                    self.dropped_packets += step - 1
            if reset:
                self.micros_base = 0
            elif self.last_micros is not None and micros < self.last_micros:
                self.micros_base += 1 << 32
            end_us = self.micros_base + micros
            if self.last_end_us is not None and not (gap or reset):
                # Spread the samples evenly since the previous packet's last
                #  sample, so consecutive packets join without a seam
                self.period_us = (end_us - self.last_end_us) / count
                self.packet_s = (end_us - self.last_end_us) / 1e6
            if reset:
                self.offsets.clear()
            self.offsets.append((arrival, arrival - end_us / 1e6))
            while self.offsets[0][0] < arrival - OFFSET_WINDOW_S:
                self.offsets.popleft()
            self.last_seq = seq
            self.last_micros = micros
            self.last_end_us = end_us
            t = (end_us - self.period_us * np.arange(count - 1, -1, -1)) / 1e6
            if self.csv:
                mag = np.linalg.norm(xyz, axis=1)
                rows = np.column_stack((t, xyz, mag, fsr))
                np.savetxt(self.csv, rows,
                           fmt=["%.3f"] + ["%.2f"] * 4 + ["%.0f"] * FSR_COUNT,
                           delimiter=",")
            with self.lock:
                self.chunks.append((t, xyz, fsr, gap, reset))
                self.offset = min(o for _, o in self.offsets)
                self.samples += count
                self.overruns += overruns

    def stop(self):
        """Stops reading and closes the CSV, so no recorded rows are lost."""
        self.stopping.set()
        self.join()

    def take_chunks(self):
        with self.lock:
            chunks = list(self.chunks)
            self.chunks.clear()
        return chunks


class NoiseGate:
    """Flattens the resting noise of X, Y and Z for the plot.

    Samples within band_g of an axis's resting value draw as that value, the
    rest pass through unchanged, so peaks aren't averaged down. The resting
    value is tracked from the in-band samples only. The drawn level moves
    only once it has drifted LEVEL_STEP_G away, otherwise the tracking
    itself would show as a slow wiggle in the flat line.
    """

    def __init__(self, band_g, sample_rate):
        self.band_g = band_g
        self.alpha = 1 / (BASELINE_TAU_S * sample_rate)
        self.reset()

    def reset(self):
        self.tracked = None
        self.level = None
        self.seen = np.zeros(3)  # in-band samples so far, per axis

    def apply(self, xyz):
        if not self.band_g:
            return xyz
        if self.tracked is None:
            # Median, so an impact in the first packet can't set the level
            self.tracked = np.median(xyz, axis=0)
            self.level = self.tracked.copy()
        quiet = np.abs(xyz - self.level) <= self.band_g
        n = quiet.sum(axis=0)
        self.seen += n
        used = n > 0
        mean = (xyz * quiet).sum(axis=0)[used] / n[used]
        # A plain average until the slow tracking has enough samples
        weight = np.maximum(1 - (1 - self.alpha) ** n[used],
                            n[used] / self.seen[used])
        self.tracked[used] += weight * (mean - self.tracked[used])
        moved = np.abs(self.tracked - self.level) > LEVEL_STEP_G
        self.level[moved] = self.tracked[moved]
        return np.where(quiet, self.level, xyz)

    def zero(self, xyz):
        """Shifts apply()'s output so each axis rests at 0."""
        return xyz if self.level is None else xyz - self.level


class NegativePeaks:
    """Finds the largest negative swing of each impact, per axis.

    feed() takes signed samples relative to rest and returns (t, axis, g)
    for each impact that ended with a swing of at least min_g below rest.
    An impact is a run of samples at least min_g from rest, either sign,
    that ends once the axis has stayed under min_g for rearm_s, so the
    ringing after a hit gets one label rather than one per oscillation.
    """

    def __init__(self, min_g, rearm_s):
        self.min_g = min_g
        self.rearm_s = rearm_s
        self.reset()

    def reset(self):
        self.last_over = [None] * 3  # time of the impact's latest sample over
        self.peak = [None] * 3       # (t, g) of its most negative one so far

    def feed(self, t, signed):
        found = []
        if not self.min_g:
            return found
        over = np.abs(signed) >= self.min_g
        for axis in range(3):
            if self.last_over[axis] is None and not over[:, axis].any():
                continue  # quiet, the common case
            for i in range(len(t)):
                if over[i, axis]:
                    self.last_over[axis] = t[i]
                    g = signed[i, axis]
                    if g < 0 and (self.peak[axis] is None
                                  or g < self.peak[axis][1]):
                        self.peak[axis] = (t[i], g)
                elif (self.last_over[axis] is not None
                      and t[i] - self.last_over[axis] >= self.rearm_s):
                    found += self._end(axis)
        return found

    def flush(self):
        """Ends every impact in progress, e.g. at a gap in the stream."""
        found = []
        for axis in range(3):
            found += self._end(axis)
        return found

    def _end(self, axis):
        peak = self.peak[axis]
        self.last_over[axis] = None
        self.peak[axis] = None
        return [] if peak is None else [(peak[0], axis, peak[1])]


COLUMNS = 4 + FSR_COUNT  # t, x, y, z, fsr D8, D9, D10


class RingBuffer:
    """Last n samples of COLUMNS, oldest first on read."""

    def __init__(self, n):
        self.data = np.full((n, COLUMNS), np.nan)
        self.pos = 0
        self.full = False
        self.total = 0  # rows ever added, so each row has a fixed index

    def clear(self):
        self.pos = 0
        self.full = False

    def extend(self, rows):
        n = len(self.data)
        rows = rows[-n:]
        end = self.pos + len(rows)
        if end <= n:
            self.data[self.pos:end] = rows
        else:
            split = n - self.pos
            self.data[self.pos:] = rows[:split]
            self.data[:end - n] = rows[split:]
        self.full = self.full or end >= n
        self.pos = end % n
        self.total += len(rows)

    def ordered(self):
        """Returns (rows, index of the first row)."""
        if not self.full:
            rows = self.data[:self.pos]
        else:
            rows = np.concatenate((self.data[self.pos:],
                                   self.data[:self.pos]))
        return rows, self.total - len(rows)


def peak_decimate(rows, first_index, ds):
    """Min and max of each group of ds rows, as two rows per group.

    Like pyqtgraph's peak downsampling, so short spikes stay visible, but
    the groups are aligned to each row's fixed index instead of to the
    first visible row. That way a sample stays in the same group from frame
    to frame and the trace doesn't shimmer while it scrolls. A group with a
    gap row is NaN, which breaks the line like the gap itself.
    """
    if ds <= 1:
        return rows
    rows = rows[(-first_index) % ds:]
    n = len(rows) // ds * ds
    groups = rows[:n].reshape(-1, ds, rows.shape[1])
    out = np.empty((2 * len(groups), rows.shape[1]))
    out[0::2] = groups.min(axis=1)
    out[1::2] = groups.max(axis=1)
    # Both at the group's first time, so each group is a vertical stroke
    out[:, 0] = np.repeat(np.fmin.reduce(groups[:, :, 0], axis=1), 2)
    return np.concatenate((out, rows[n:]))  # newest partial group as is


def smooth_span(window_s, pixels, refresh_hz):
    """Seconds to show near window_s, so the view scrolls evenly.

    Lines are drawn on whole pixels, so a speed of 2.46 pixels per display
    refresh moves the picture 2, 2, 3, 2, 3... pixels, which looks jittery.
    The span is adjusted so the speed is a whole number of pixels per
    refresh, or below one, one pixel every whole number of refreshes.
    That shows between 0.67 and 1.5 times window_s.
    """
    if pixels < 1 or refresh_hz <= 0:
        return window_s
    speed = pixels / window_s / refresh_hz  # pixels per refresh
    if speed >= 1:
        speed = round(speed)
    else:
        speed = 1 / round(1 / speed)
    return pixels / (speed * refresh_hz)


class PixelScroll:
    """Steps the view the same whole number of pixels every refresh.

    Rounding the clock-based position to a pixel each frame isn't enough:
    the time a frame runs varies by a fraction of a millisecond, and near
    a rounding boundary that turns a 2 px step into 1 px or 3 px. Instead
    the position moves by refreshes elapsed times the speed, and is only
    nudged by one pixel when it drifts more than 1.5 pixels from the clock.
    """

    def __init__(self):
        self.grid = None  # (pixels, span) the position was set up for
        self.px = None    # position of the view's right edge, in pixels
        self.last = None  # time of the previous frame

    def position(self, t_end, now, pixels, span, refresh_hz):
        """Right edge of the view: t_end moved onto the pixel grid."""
        if pixels < 1 or refresh_hz <= 0:
            self.grid = None
            return t_end
        s_per_px = span / pixels
        target = t_end / s_per_px
        speed = pixels / (span * refresh_hz)  # pixels per refresh
        if self.grid != (pixels, span):
            self.px = target  # first frame, resized or new refresh rate
        else:
            refreshes = max(1, round((now - self.last) * refresh_hz))
            self.px += refreshes * speed
            error = target - self.px
            if abs(error) > max(8, 4 * speed):
                self.px = target  # a pause, a gap or the board restarted
            elif abs(error) > 1.5:  # past timing noise: drift, catch up
                self.px += 1 if error > 0 else -1
        self.grid = (pixels, span)
        self.last = now
        # 1e-6: k * 0.5 summed up may land just under the whole pixel
        return np.floor(self.px + 1e-6) * s_per_px


def fsr_conductance(adc):
    """FSR conductance in 1/kohm from its reading, 0 at rest.

    Roughly proportional to force, unlike the reading itself, which bunches
    up harder presses because of the voltage divider.
    """
    adc = np.maximum(adc, 1)  # 0 would mean no resistance at all
    return (FSR_ADC_MAX - adc) / (FSR_PULLUP_KOHM * adc)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("-p", "--port", help="serial port (default: auto-detect)")
    parser.add_argument("-o", "--output", help="record every sample to this CSV")
    parser.add_argument("-w", "--window", type=float, default=10.0,
                        help="seconds of history shown, adjusted so the "
                             "view scrolls a whole number of pixels per "
                             "display refresh (default: 10)")
    parser.add_argument("-r", "--rate", type=float,
                        default=MAX_SAMPLE_RATE_HZ,
                        help="samples/sec to stream, kidney_guard only "
                             f"(default: {MAX_SAMPLE_RATE_HZ})")
    parser.add_argument("--noise-band", type=float, default=NOISE_BAND_G,
                        help="draw samples within this many g of the resting "
                             "value as flat, 0 shows the raw signal "
                             f"(default: {NOISE_BAND_G:g})")
    parser.add_argument("--label-min", type=float, default=PEAK_LABEL_MIN_G,
                        help="label negative peaks at least this many g from "
                             "rest, 0 turns the labels off "
                             f"(default: {PEAK_LABEL_MIN_G:g})")
    args = parser.parse_args()
    if not 1 <= args.rate <= MAX_SAMPLE_RATE_HZ:
        parser.error(f"--rate must be 1 to {MAX_SAMPLE_RATE_HZ}")
    if args.noise_band < 0:
        parser.error("--noise-band can't be negative")
    if args.label_min < 0:
        parser.error("--label-min can't be negative")
    # The board streams one of every N samples, so the rate is rounded to
    #  1000/N: -r 300 gives 333 samples/sec
    decimation = round(MAX_SAMPLE_RATE_HZ / args.rate)

    # Imported here so --help works without a Qt install
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtGui

    port = args.port or find_port()
    reader = StreamReader(port, args.output, decimation)
    reader.start()

    app = pg.mkQApp("Accelerometer stream")

    class PlotWindow(pg.GraphicsLayoutWidget):
        """Adds up the time spent painting, for FrameClock."""

        paint_s = 0.0

        def paintEvent(self, event):
            start = time.monotonic()
            super().paintEvent(event)
            self.paint_s += time.monotonic() - start

    win = PlotWindow(title=f"Accelerometer stream: {port}")
    win.resize(1100, 900)
    status = win.addLabel("waiting for data...", row=0, col=0)

    axes_plot = win.addPlot(row=1, col=0)
    axes_plot.setLabel("left", "|change from rest|" if args.noise_band
                       else "|acceleration|", units="g")
    axes_plot.addLegend(offset=(10, 10))
    fsr_plot = win.addPlot(row=2, col=0)
    fsr_plot.setLabel("left", "FSR")
    fsr_plot.setLabel("bottom", "time", units="s")
    fsr_plot.setXLink(axes_plot)
    for row in (1, 2):  # same height; stretch factors have no effect here
        win.ci.layout.setRowPreferredHeight(row, 1000)
    for plot in (axes_plot, fsr_plot):
        plot.showGrid(x=True, y=True, alpha=0.3)
        # Time grid lines on the major ticks only. The minor ones added ~90
        #  full-height lines per graph, redrawn every frame as the view
        #  scrolls: ~11 ms per frame full screen, more than a 100 Hz refresh
        plot.getAxis("bottom").setStyle(maxTickLevel=0)
        plot.getAxis("left").setWidth(80)  # keeps the time axes lined up
        # Downsampling is done by peak_decimate(), pyqtgraph's own shimmers
        #  while scrolling
    axes_plot.setLogMode(False, True)
    # The range of a log axis is given as log10 of the values
    axes_plot.setYRange(np.log10(ACCEL_LOG_MIN_G), np.log10(ACCEL_LOG_MAX_G),
                        padding=0.02)
    axes_plot.getAxis("left").setTicks([[
        (np.log10(g), str(g)) for g in (1, 2, 5, 10, 20, 50, 100, 200)]])
    fsr_plot.setYRange(0, FSR_MAX_CONDUCTANCE, padding=0.02)

    curves = [axes_plot.plot(pen=pg.mkPen(color, width=1), name=name,
                             connect="finite")
              for name, color in zip(AXIS_NAMES, AXIS_COLORS)]
    curves += [fsr_plot.plot(pen=pg.mkPen(color, width=1), connect="finite")
               for color in ("#986ee2", "#cc6b2c", "#39c5cf")]
    medium = fsr_conductance(FSR_MEDIUM_ADC)
    heavy = fsr_conductance(FSR_HEAVY_ADC)
    for level in (medium, heavy):
        fsr_plot.addItem(pg.InfiniteLine(
            pos=level, angle=0,
            pen=pg.mkPen("#888", style=QtCore.Qt.PenStyle.DashLine)))
    # The axis names the press bands instead of showing numbers
    fsr_plot.getAxis("left").setTicks([[
        (medium / 2, "light"),
        ((medium + heavy) / 2, "medium"),
        ((heavy + FSR_MAX_CONDUCTANCE) / 2, "strong")]])
    fsr_plot.showGrid(x=True, y=False)  # the band lines replace the y grid

    # smooth_span() shows up to 1.5 times --window
    ring = RingBuffer(int(2 * args.window * MAX_SAMPLE_RATE_HZ))
    gate = NoiseGate(args.noise_band, MAX_SAMPLE_RATE_HZ / decimation)
    negatives = NegativePeaks(args.label_min, PEAK_REARM_S)

    class PeakLabels(pg.GraphicsObject):
        """All negative-peak labels, drawn by one item in screen pixels.

        One pg.TextItem per label re-laid out its text and transform on
        every frame while the view scrolled, which with a few dozen labels
        pushed a full-screen frame past a 100 Hz refresh. This item sits
        on the ViewBox itself rather than on the scrolling data, so its
        bounds only change on a resize, not on every frame.
        """

        def __init__(self, vb):
            super().__init__(vb)
            self.vb = vb
            self.labels = collections.deque()  # (t, y, text, color)
            self.setZValue(10)  # above the curves
            vb.sigResized.connect(lambda _: self.prepareGeometryChange())

        def add(self, t, y, text, color):
            self.labels.append((t, y, text, QtGui.QColor(color)))
            self.update()

        def drop(self, before=None):
            """Drops labels older than before, or all of them."""
            while self.labels and (before is None
                                   or self.labels[0][0] < before):
                self.labels.popleft()
            self.update()

        def boundingRect(self):
            return self.vb.boundingRect()  # the plot area, in pixels

        def paint(self, p, *args):
            p.setClipRect(self.boundingRect())
            to_pixels = self.vb.childTransform()  # data -> plot area
            metrics = p.fontMetrics()
            for t, y, text, color in self.labels:
                pos = to_pixels.map(QtCore.QPointF(t, y))
                width = metrics.horizontalAdvance(text)
                p.setPen(color)
                # Centred just above the peak
                p.drawText(QtCore.QPointF(pos.x() - width / 2,
                                          pos.y() - metrics.descent() - 2),
                           text)

    labels = PeakLabels(axes_plot.vb)

    def add_labels(peaks):
        for t, axis, g in peaks:
            # Items on a log axis are placed by log10 of the value
            labels.add(t, np.log10(max(abs(g), ACCEL_LOG_MIN_G)),
                       "neg",
                       AXIS_COLORS[axis])

    def clear_labels(before=None):
        if labels.labels and (before is None
                              or labels.labels[0][0] < before):
            labels.drop(before)

    state = {"paused": False, "last_samples": 0, "last_time": time.monotonic(),
             "rate": 0.0, "span": args.window, "status": ""}
    scroll = PixelScroll()

    def toggle_pause():
        state["paused"] = not state["paused"]

    QtGui.QShortcut(QtGui.QKeySequence("Space"), win, toggle_pause)

    def update():
        if reader.error:
            status.setText(f"serial error: {reader.error}")
            frames.stop()
            return
        for t, xyz, fsr, gap, reset in reader.take_chunks():
            if reset:
                ring.clear()
                gate.reset()
                negatives.reset()
                clear_labels()
            if gap:  # a NaN row breaks the line across dropped packets
                ring.extend(np.full((1, COLUMNS), np.nan))
                add_labels(negatives.flush())
            signed = gate.zero(gate.apply(xyz))
            add_labels(negatives.feed(t, signed))
            # Clipped to the axis bottom, a log axis can't draw 0
            xyz = np.maximum(np.abs(signed), ACCEL_LOG_MIN_G)
            ring.extend(np.column_stack((t, xyz, fsr_conductance(fsr))))

        now = time.monotonic()
        if now - state["last_time"] >= 1.0:
            state["rate"] = ((reader.samples - state["last_samples"])
                             / (now - state["last_time"]))
            state["last_samples"] = reader.samples
            state["last_time"] = now
        recording = (f"  recording to {os.path.basename(args.output)}"
                     if args.output else "")
        paused = "  PAUSED (space)" if state["paused"] else ""
        gated = (f"  noise band: {args.noise_band:g} g"
                 if args.noise_band else "")
        text = (
            f"{state['rate']:.0f} samples/s  showing {state['span']:.1f} s  "
            f"{frames.fps():.0f} fps  dropped packets: "
            f"{reader.dropped_packets}  sensor overruns: {reader.overruns}  "
            f"bad packets: {reader.parser.bad_packets}{gated}{recording}"
            f"{paused}")
        if text != state["status"]:  # setText() lays the label out again
            status.setText(text)
            state["status"] = text

        if state["paused"]:
            return
        data, first_index = ring.ordered()
        if not len(data) or reader.offset is None:
            return
        # Scroll with the host clock, mapped to board time, so the view
        #  moves smoothly between packets instead of jumping with each one
        now = time.monotonic()
        t_end = now - reader.offset - max(VIEW_DELAY_S, reader.packet_s)
        pixels = axes_plot.vb.width() * win.devicePixelRatioF()
        screen = win.windowHandle().screen()
        refresh_hz = screen.refreshRate() if screen else 0
        span = smooth_span(args.window, pixels, refresh_hz)
        state["span"] = span
        t_end = scroll.position(t_end, now, pixels, span, refresh_hz)
        t_start = t_end - span
        clear_labels(before=t_start)  # scrolled out of view
        # Only the shown seconds (plus one sample, so the line enters from
        #  the left edge); the buffer holds more when streaming below the
        #  full rate
        visible = data[:, 0] >= t_start
        if not visible.any():
            return
        start = max(int(np.argmax(visible)) - 1, 0)
        data = data[start:]
        first_index += start
        # About one group per screen pixel; a power of two, so it only
        #  changes when the window is resized a lot
        per_pixel = span * MAX_SAMPLE_RATE_HZ / decimation / max(pixels, 1)
        ds = 2 ** int(np.log2(per_pixel)) if per_pixel >= 2 else 1
        data = peak_decimate(data, first_index, ds)
        for i, curve in enumerate(curves):
            curve.setData(data[:, 0], data[:, i + 1])
        axes_plot.setXRange(t_start, t_end, padding=0)

    class FrameClock(QtCore.QObject):
        """Calls update() once per display refresh, just before painting.

        A fixed timer beats against the display: at 16 ms on a 100 Hz
        screen each frame stayed up for one or two refreshes, so the
        scrolling stuttered. requestUpdate() is paced by the display
        (CVDisplayLink on macOS), and the UpdateRequest it sends is also
        what paints the window, so the data set here is drawn right away.

        When frames take longer than a refresh (full screen with dense
        impacts) they would again stay up for one or two refreshes, so it
        then draws on every second refresh only: an even 50 fps at 100 Hz
        looks smoother than a mix of 100 and 50. Refreshes are skipped by
        count, not by time, since under load Qt delivers them a few ms
        late. It tries every refresh again once frames look cheap enough,
        and backs off for longer each time that try fails.
        """

        MISSES_FOR_HALF = 3      # missed refreshes within a second
        HALF_HOLD_S = 3.0        # stay at half rate at least this long,
        HALF_HOLD_MAX_S = 30.0   #  doubled up to this after a failed try
        RETRY_FAILED_S = 5.0     # back at half rate this soon: a failed try
        FULL_COST = 0.85         # try full rate below this much of a
                                 #  refresh (update + paint, 90th percentile)

        def __init__(self, window, widget):
            super().__init__()
            self.window = window
            self.widget = widget
            self.running = True
            self.divider = 1           # draw on every Nth refresh
            self.skipped = 0           # refreshes skipped since the last draw
            self.last_drawn = None
            self.half_since = 0.0
            self.full_since = 0.0
            self.hold_s = self.HALF_HOLD_S
            self.misses = collections.deque()  # times of missed refreshes
            self.costs = collections.deque(maxlen=100)  # seconds per frame
            self.update_s = None       # update() time of the last frame
            window.installEventFilter(self)
            window.requestUpdate()

        def period(self):
            hz = self.window.screen().refreshRate() if self.window.screen() \
                else 0
            return 1 / hz if hz > 0 else 1 / 60

        def fps(self):
            return 1 / (self.period() * self.divider)

        def eventFilter(self, obj, event):
            if (self.running
                    and event.type() == QtCore.QEvent.Type.UpdateRequest):
                self.frame()
                self.window.requestUpdate()
            return False  # let the window paint as usual

        def frame(self):
            if self.skipped + 1 < self.divider:
                self.skipped += 1
                return  # half rate: skip this refresh
            self.skipped = 0
            now = time.monotonic()
            period = self.period()
            if self.last_drawn is not None:
                if round((now - self.last_drawn) / period) > self.divider:
                    self.misses.append(now)
            self.last_drawn = now
            if self.update_s is not None:
                # The last frame's cost: its update() plus the painting
                #  that followed it. Longer than a refresh means it missed
                #  one, even when Qt delivered this refresh early enough
                #  that the gap above still rounds to one
                cost = self.update_s + self.widget.paint_s
                self.costs.append(cost)
                if self.divider == 1 and cost > period:
                    self.misses.append(now)
            self.widget.paint_s = 0.0
            start = time.monotonic()
            update()
            self.update_s = time.monotonic() - start
            self.adapt(now, period)

        def adapt(self, now, period):
            while self.misses and self.misses[0] < now - 1.0:
                self.misses.popleft()
            if self.divider == 1 and len(self.misses) >= self.MISSES_FOR_HALF:
                if now - self.full_since < self.RETRY_FAILED_S:
                    self.hold_s = min(2 * self.hold_s, self.HALF_HOLD_MAX_S)
                else:
                    self.hold_s = self.HALF_HOLD_S
                self.divider = 2
                self.half_since = now
                self.misses.clear()
            elif (self.divider == 2
                  and now - self.half_since >= self.hold_s
                  and self.costs
                  and np.percentile(self.costs, 90)
                  < self.FULL_COST * period):
                self.divider = 1
                self.full_since = now
                self.misses.clear()

        def stop(self):
            self.running = False

    win.show()
    frames = FrameClock(win.windowHandle(), win)
    app.exec()
    reader.stop()


if __name__ == "__main__":
    main()
