#!/usr/bin/env python3
"""Live plot of the 1 kHz accelerometer stream from Main/kidney_guard.

kidney_guard streams only on request: the plotter sends 's' when it connects
and 't' (back to text output) when it closes.

-r lowers the rate kidney_guard streams at (the sensor itself keeps running
at 1 kHz). Each streamed sample is the one with the largest |a| of its
group, so impact peaks survive, and each FSR is its lowest reading (hardest
press); the peak's time is only accurate to one streamed sample period, though.

Shows the last few seconds of X/Y/Z and the vector magnitude, with the
impact threshold drawn in, and the FSRs on D8, D9 and D10 when the sketch
sends them (kidney_guard does), drawn as conductance, which is roughly
proportional to force. Optionally records every sample to CSV. Close
the Arduino Serial Monitor first, only one program can open the port.

Usage:
    python3 Tools/plot_stream.py                  # auto-detect port
    python3 Tools/plot_stream.py -o impact.csv    # also record to CSV
    python3 Tools/plot_stream.py -p /dev/cu.usbmodem1101 -w 5 -t 50
    python3 Tools/plot_stream.py -r 100           # stream 100 samples/sec

X/Y/Z and magnitude are drawn flat at rest: samples within --noise-band of an
axis's resting value draw as that value, and anything further out draws at
its true raw value, so impact heights are kept. X/Y/Z are then shifted so each
axis rests at 0, cancelling gravity and the sensor's zero-g offset, and the
magnitude is taken from those shifted values, so it rests at 0 too. Use
--noise-band 0 for the raw signal. The CSV is always raw.

Keys: space = pause/resume the view (recording continues).

Requires:  python3 -m pip install pyserial numpy pyqtgraph PyQt6
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
FSR_COUNT = 3
FSR_ADC_MAX = 4095                 # the reading at rest
FSR_PULLUP_KOHM = 4.7
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


COLUMNS = 5 + FSR_COUNT  # t, x, y, z, magnitude, fsr D8, D9, D10


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


def fsr_conductance(adc):
    """FSR conductance in 1/kohm from its reading, 0 at rest.

    Roughly proportional to force, unlike the reading itself, which bunches
    up harder presses because of the voltage divider.
    """
    adc = np.maximum(adc, 1)  # 0 would mean no resistance at all
    return (FSR_ADC_MAX - adc) / (FSR_PULLUP_KOHM * adc)


def nice_step(span):
    """A 1, 2 or 5 times a power of ten step near span/4."""
    raw = span / 4
    base = 10 ** np.floor(np.log10(raw))
    for m in (1, 2, 5):
        if m * base >= raw:
            return m * base
    return 10 * base


class StableYRange:
    """Sets a plot's y range from its visible data without wobbling.

    The range is rounded out to a nice step, grows as soon as the data
    leaves it, and shrinks only once it is well over twice what the data
    needs, so noise doesn't rescale the axis every frame.
    """

    def __init__(self, plot, min_span, floor=None):
        self.plot = plot
        self.min_span = min_span
        self.floor = floor  # lowest the range starts, e.g. 0 for |a|
        self.range = None
        plot.enableAutoRange(y=False)

    def update(self, values):
        values = values[np.isfinite(values)]
        if not len(values):
            return
        lo, hi = values.min(), values.max()
        if self.floor is not None:
            lo = min(lo, self.floor)
        span = max(hi - lo, self.min_span)
        if self.range:
            r_lo, r_hi = self.range
            if lo >= r_lo and hi <= r_hi and r_hi - r_lo <= 2.5 * span:
                return
        step = nice_step(span)
        r_lo = np.floor(lo / step) * step
        r_hi = np.ceil(hi / step) * step
        if r_hi - r_lo < span:
            r_hi = r_lo + np.ceil(span / step) * step
        self.range = (r_lo, r_hi)
        self.plot.setYRange(r_lo, r_hi, padding=0.02)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("-p", "--port", help="serial port (default: auto-detect)")
    parser.add_argument("-o", "--output", help="record every sample to this CSV")
    parser.add_argument("-w", "--window", type=float, default=10.0,
                        help="seconds of history shown (default: 10)")
    parser.add_argument("-t", "--threshold", type=float, default=50.0,
                        help="impact threshold line in g (default: 50)")
    parser.add_argument("-r", "--rate", type=float,
                        default=MAX_SAMPLE_RATE_HZ,
                        help="samples/sec to stream, kidney_guard only "
                             f"(default: {MAX_SAMPLE_RATE_HZ})")
    parser.add_argument("--noise-band", type=float, default=NOISE_BAND_G,
                        help="draw samples within this many g of the resting "
                             "value as flat, 0 shows the raw signal "
                             f"(default: {NOISE_BAND_G:g})")
    args = parser.parse_args()
    if not 1 <= args.rate <= MAX_SAMPLE_RATE_HZ:
        parser.error(f"--rate must be 1 to {MAX_SAMPLE_RATE_HZ}")
    if args.noise_band < 0:
        parser.error("--noise-band can't be negative")
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
    win = pg.GraphicsLayoutWidget(title=f"Accelerometer stream: {port}")
    win.resize(1100, 900)
    status = win.addLabel("waiting for data...", row=0, col=0)

    axes_plot = win.addPlot(row=1, col=0)
    axes_plot.setLabel("left", "change from rest" if args.noise_band
                       else "acceleration", units="g")
    axes_plot.addLegend(offset=(10, 10))
    mag_plot = win.addPlot(row=2, col=0)
    mag_plot.setLabel("left", "|a| from rest" if args.noise_band else "|a|",
                      units="g")
    mag_plot.setXLink(axes_plot)
    fsr_plot = win.addPlot(row=3, col=0)
    fsr_plot.setLabel("left", "FSR")
    fsr_plot.setLabel("bottom", "time", units="s")
    fsr_plot.setXLink(axes_plot)
    for plot in (axes_plot, mag_plot, fsr_plot):
        plot.showGrid(x=True, y=True, alpha=0.3)
        plot.getAxis("left").setWidth(80)  # keeps the time axes lined up
        # Downsampling is done by peak_decimate(), pyqtgraph's own shimmers
        #  while scrolling
    y_ranges = (StableYRange(axes_plot, min_span=1.0),
                StableYRange(mag_plot, min_span=1.0, floor=0.0),
                StableYRange(fsr_plot, min_span=1.5, floor=0.0))

    curves = [axes_plot.plot(pen=pg.mkPen(color, width=1), name=name,
                             connect="finite")
              for name, color in (("x", "#e5534b"), ("y", "#57ab5a"),
                                  ("z", "#539bf5"))]
    curves.append(mag_plot.plot(pen=pg.mkPen("#c69026", width=1),
                                connect="finite"))
    mag_plot.addItem(pg.InfiniteLine(
        pos=args.threshold, angle=0, pen=pg.mkPen("#888", style=QtCore.Qt.PenStyle.DashLine),
        label=f"{args.threshold:g} g", labelOpts={"position": 0.02}))
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
        (heavy + (heavy - medium), "strong")]])
    fsr_plot.showGrid(x=True, y=False)  # the band lines replace the y grid

    ring = RingBuffer(int(args.window * MAX_SAMPLE_RATE_HZ))
    gate = NoiseGate(args.noise_band, MAX_SAMPLE_RATE_HZ / decimation)
    state = {"paused": False, "last_samples": 0, "last_time": time.monotonic(),
             "rate": 0.0}

    def toggle_pause():
        state["paused"] = not state["paused"]

    QtGui.QShortcut(QtGui.QKeySequence("Space"), win, toggle_pause)

    def update():
        if reader.error:
            status.setText(f"serial error: {reader.error}")
            timer.stop()
            return
        for t, xyz, fsr, gap, reset in reader.take_chunks():
            if reset:
                ring.clear()
                gate.reset()
            if gap:  # a NaN row breaks the line across dropped packets
                ring.extend(np.full((1, COLUMNS), np.nan))
            xyz = gate.zero(gate.apply(xyz))
            mag = np.linalg.norm(xyz, axis=1)
            ring.extend(np.column_stack((t, xyz, mag, fsr_conductance(fsr))))

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
        status.setText(
            f"{state['rate']:.0f} samples/s  dropped packets: "
            f"{reader.dropped_packets}  sensor overruns: {reader.overruns}  "
            f"bad packets: {reader.parser.bad_packets}{gated}{recording}"
            f"{paused}")

        if state["paused"]:
            return
        data, first_index = ring.ordered()
        if not len(data) or reader.offset is None:
            return
        # Scroll with the host clock, mapped to board time, so the view
        #  moves smoothly between packets instead of jumping with each one
        t_end = (time.monotonic() - reader.offset
                 - max(VIEW_DELAY_S, reader.packet_s))
        t_start = t_end - args.window
        # Only the shown --window seconds (plus one sample, so the line
        #  enters from the left edge); the buffer holds more when streaming
        #  below the full rate
        visible = data[:, 0] >= t_start
        if not visible.any():
            return
        start = max(int(np.argmax(visible)) - 1, 0)
        data = data[start:]
        first_index += start
        # About one group per screen pixel; a power of two, so it only
        #  changes when the window is resized a lot
        pixels = axes_plot.vb.width() * win.devicePixelRatioF()
        per_pixel = args.window * MAX_SAMPLE_RATE_HZ / decimation / pixels
        ds = 2 ** int(np.log2(per_pixel)) if per_pixel >= 2 else 1
        data = peak_decimate(data, first_index, ds)
        for i, curve in enumerate(curves):
            curve.setData(data[:, 0], data[:, i + 1])
        axes_plot.setXRange(t_start, t_end, padding=0)
        in_view = data[:, 0] <= t_end
        y_ranges[0].update(data[in_view, 1:4])
        y_ranges[1].update(data[in_view, 4])
        y_ranges[2].update(data[in_view, 5:8])

    timer = QtCore.QTimer()
    timer.setTimerType(QtCore.Qt.TimerType.PreciseTimer)
    timer.timeout.connect(update)
    timer.start(16)  # ~60 fps, smooth scrolling

    win.show()
    app.exec()
    reader.stop()


if __name__ == "__main__":
    main()
