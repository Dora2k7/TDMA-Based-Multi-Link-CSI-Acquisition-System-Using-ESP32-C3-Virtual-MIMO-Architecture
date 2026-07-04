"""
csi_receiver.py (v3.0 — Processor Bridge)
------------------------------------------
Đọc CSI từ 4 cổng Serial (4 ESP32-C3) theo giao thức Binary + CRC16,
ghép theo round_id thành frame 52×12 (52 subcarrier sạch).

Cải tiến v3:
  - Tích hợp multiprocessing.Queue để gửi raw frame sang csi_processor.py
  - Hàm set_processor_queue() cho phép csi_processor điều khiển
  - Tương thích chạy độc lập (python csi_receiver.py) hoặc
    được import bởi csi_processor.py
"""

import serial
import threading
import time
import struct
import os
import csv
import queue as _queue
import numpy as np
from collections import defaultdict

# ----- CẤU HÌNH -----
PORTS = {
    0: "COM11",
    1: "COM12",
    2: "COM13",
    3: "COM14",
}
BAUDRATE  = 2000000
N_SUBCARR = 64
LINKS     = [(i, j) for i in range(4) for j in range(4) if i != j]  # 12 link

# ----- GLOBAL STATE -----
buffers             = defaultdict(dict)
lock                = threading.Lock()
n_frames_emitted    = 0
n_complete_frames   = 0
n_incomplete_frames = 0
n_crc_errors        = 0
start_time          = None
global_max_round_id = 0

# ----- PROCESSOR BRIDGE -----
# Queue được gắn bởi csi_processor.py để nhận raw frames thời gian thực
_proc_queue = None


def set_processor_queue(q) -> None:
    """Được gọi bởi csi_processor.py trước khi start threads."""
    global _proc_queue
    _proc_queue = q


# =========================================================================
# CRC16-CCITT (phải khớp với firmware ESP32)
# =========================================================================
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


# =========================================================================
# Buffered Record Reader (thay thế byte-by-byte)
# =========================================================================
def read_record(ser: serial.Serial):
    """
    Đọc bản ghi binary từ stream serial với buffered read.
    Protocol: [Magic 4B][Header 9B][CSI data NB][CRC16 2B]
    """
    MAGIC = b'\x21\x49\x53\x43'
    buf = getattr(ser, '_leftover', b'')

    while True:
        # Đọc chunk lớn thay vì từng byte
        waiting = ser.in_waiting
        if waiting > 0:
            buf += ser.read(waiting)
        else:
            buf += ser.read(1)

        if not buf:
            return None

        # Tìm magic word trong buffer
        idx = buf.find(MAGIC)
        if idx < 0:
            buf = buf[-3:]  # giữ 3 byte cuối phòng magic cắt ngang
            continue

        buf = buf[idx + 4:]  # bỏ phần trước magic + magic

        # Đọc thêm nếu chưa đủ header (9 bytes)
        while len(buf) < 9:
            chunk = ser.read(max(1, 9 - len(buf)))
            if not chunk:
                return None
            buf += chunk

        round_id, tx_id, rx_id, rssi, csi_len = struct.unpack('<IBBbH', buf[:9])
        header_bytes = buf[:9]
        buf = buf[9:]

        # Reject impossible csi_len
        if csi_len == 0 or csi_len > 512:
            continue

        # Đọc CSI data
        while len(buf) < csi_len:
            chunk = ser.read(max(1, csi_len - len(buf)))
            if not chunk:
                return None
            buf += chunk

        csi_bytes = buf[:csi_len]
        buf = buf[csi_len:]

        # Đọc CRC16 (2 bytes)
        while len(buf) < 2:
            chunk = ser.read(max(1, 2 - len(buf)))
            if not chunk:
                return None
            buf += chunk

        crc_received = struct.unpack('<H', buf[:2])[0]
        buf = buf[2:]

        # Verify CRC16
        full_record = MAGIC + header_bytes + csi_bytes
        crc_computed = crc16_ccitt(full_record)

        if crc_received != crc_computed:
            global n_crc_errors
            n_crc_errors += 1
            ser._leftover = buf
            continue  # drop corrupted record

        ser._leftover = buf
        return round_id, tx_id, rx_id, rssi, csi_bytes


def process_csi(csi_bytes: bytes) -> np.ndarray:
    """Chuyển đổi dữ liệu CSI nhị phân sang vector biên độ (amplitude)."""
    raw_array = np.frombuffer(csi_bytes, dtype=np.int8)

    if len(raw_array) % 2 != 0:
        raw_array = raw_array[:-1]

    I = raw_array[0::2].astype(np.float32)
    Q = raw_array[1::2].astype(np.float32)
    return np.sqrt(I * I + Q * Q)


# =========================================================================
# Health Monitor
# =========================================================================
class HealthMonitor:
    def __init__(self, report_interval=5.0):
        self.report_interval = report_interval
        self.window_start = time.time()
        self.window_complete = 0
        self.window_incomplete = 0
        self.window_records = 0
        self.window_crc_errors = 0
        self.link_hit_count = {}
        self.per_device_records = [0, 0, 0, 0]

    def on_record(self, dev_id, tx_id, rx_id):
        self.window_records += 1
        self.per_device_records[dev_id] += 1
        key = (tx_id, rx_id)
        self.link_hit_count[key] = self.link_hit_count.get(key, 0) + 1

    def on_frame(self, is_complete):
        if is_complete:
            self.window_complete += 1
        else:
            self.window_incomplete += 1

    def check_and_report(self):
        now = time.time()
        elapsed = now - self.window_start
        if elapsed < self.report_interval:
            return

        fps_complete = self.window_complete / elapsed
        fps_total = (self.window_complete + self.window_incomplete) / elapsed
        total_frames = self.window_complete + self.window_incomplete
        completeness = (self.window_complete / max(1, total_frames)) * 100

        # Tìm link yếu/mạnh nhất
        weakest = min(self.link_hit_count.items(), key=lambda x: x[1]) if self.link_hit_count else ((0,0), 0)
        strongest = max(self.link_hit_count.items(), key=lambda x: x[1]) if self.link_hit_count else ((0,0), 0)

        print(f"\n{'─'*60}")
        print(f"  📊 HEALTH [{elapsed:.1f}s window]")
        print(f"  FPS (clean 12/12): {fps_complete:.1f}")
        print(f"  FPS (total):       {fps_total:.1f}")
        print(f"  Completeness:      {completeness:.0f}%")
        print(f"  Records:           {self.window_records}")
        print(f"  CRC errors:        {self.window_crc_errors}")
        print(f"  Weakest link:      {weakest[0][0]}→{weakest[0][1]} ({weakest[1]} hits)")
        print(f"  Strongest link:    {strongest[0][0]}→{strongest[0][1]} ({strongest[1]} hits)")
        print(f"  Per-device:        {self.per_device_records}")
        print(f"{'─'*60}\n")

        # Reset window
        self.window_start = now
        self.window_complete = 0
        self.window_incomplete = 0
        self.window_records = 0
        self.window_crc_errors = 0
        self.link_hit_count.clear()
        self.per_device_records = [0, 0, 0, 0]


health = HealthMonitor(report_interval=5.0)


# =========================================================================
# CSV Exporter — chỉ xuất frame hoàn chỉnh 12/12 link
# =========================================================================
class CSVExporter:
    def __init__(self, output_dir="csi_data"):
        os.makedirs(output_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.filepath = os.path.join(output_dir, f"csi_{timestamp}.csv")
        self.file = open(self.filepath, "w", newline="")
        self.writer = csv.writer(self.file)

        # Header: round_id, timestamp, rồi 12 link × 52 subcarrier sạch
        header = ["round_id", "timestamp"]
        for tx, rx in LINKS:
            for sc in range(52):
                header.append(f"link{tx}{rx}_sc{sc}")
        self.writer.writerow(header)
        self.count = 0
        print(f"[CSV] Xuất file: {self.filepath}")

    def add_frame(self, round_id, frame):
        """Ghi 1 frame 52×12 vào CSV. frame shape: (52, 12)"""
        row = [round_id, f"{time.time():.6f}"]
        # Flatten: link by link, subcarrier by subcarrier
        for k in range(len(LINKS)):
            for sc in range(52):
                row.append(f"{frame[sc, k]:.4f}")
        self.writer.writerow(row)
        self.count += 1
        # Flush mỗi 10 frame để tránh mất dữ liệu khi crash
        if self.count % 10 == 0:
            self.file.flush()

    def close(self):
        self.file.flush()
        self.file.close()
        print(f"[CSV] Đã lưu {self.count} frames → {self.filepath}")


exporter = None  # khởi tạo trong main()


# =========================================================================
# Frame Assembly
# =========================================================================
def try_emit_frame(round_id: int):
    global n_frames_emitted, n_complete_frames
    data = buffers.get(round_id)
    if data is None or len(data) < len(LINKS):
        return None

    # frame có kích thước mới là 52 x 12
    frame = np.zeros((52, len(LINKS)), dtype=np.float32)
    for k, (tx, rx) in enumerate(LINKS):
        vec = data[(tx, rx)]  # 64 subcarriers từ ESP32
        if len(vec) >= 64:
            # Cắt lọc lấy 52 subcarrier hữu ích (bỏ sc0 và sc27 đến sc37)
            vec_clean = np.concatenate([vec[1:27], vec[38:64]])
        else:
            vec_clean = np.zeros(52, dtype=np.float32)
        
        n = min(len(vec_clean), 52)
        frame[:n, k] = vec_clean[:n]

    del buffers[round_id]
    n_frames_emitted += 1
    n_complete_frames += 1
    return frame


def check_and_emit_stale_rounds(age_threshold: int = 10):
    global n_frames_emitted, n_incomplete_frames, global_max_round_id
    stale_rounds = [
        r for r in list(buffers.keys())
        if global_max_round_id - r > age_threshold
    ]

    for r in sorted(stale_rounds):
        data = buffers.get(r)
        if data is None:
            continue
        present_links = set(data.keys())
        missing_links = [lnk for lnk in LINKS if lnk not in present_links]

        print(
            f"[STALE] round_id={r} "
            f"có {len(present_links)}/12 link — bỏ qua (không export)"
        )

        del buffers[r]
        n_frames_emitted += 1
        n_incomplete_frames += 1
        health.on_frame(is_complete=False)


# =========================================================================
# Reader Thread
# =========================================================================
def reader_thread(dev_id: int, port: str):
    global global_max_round_id

    try:
        ser = serial.Serial(port, BAUDRATE, timeout=0.1)
    except Exception as e:
        print(f"[ERR] Không mở được {port} (device {dev_id}): {e}")
        return

    print(f"[OK] Đang đọc device {dev_id} trên {port}")
    record_counter = 0

    while True:
        try:
            parsed = read_record(ser)
            if parsed is None:
                continue
            round_id, tx_id, rx_id, rssi, csi_bytes = parsed
            amp = process_csi(csi_bytes)

            with lock:
                health.on_record(dev_id, tx_id, rx_id)

                if round_id > global_max_round_id:
                    global_max_round_id = round_id

                buffers[round_id][(tx_id, rx_id)] = amp

                # Xuất ngay nếu đủ 12 link
                frame = try_emit_frame(round_id)
                if frame is not None:
                    on_frame_ready(round_id, frame, is_complete=True)

                # Quét stale mỗi 6 records (≈ nửa round)
                record_counter += 1
                if record_counter % 6 == 0:
                    check_and_emit_stale_rounds()

                # Health report (thread-safe vì trong lock)
                health.check_and_report()

        except Exception as e:
            print(f"[ERR] device {dev_id}: {e}")
            time.sleep(0.01)


def on_frame_ready(round_id: int, frame: np.ndarray, is_complete: bool):
    """Callback mỗi khi có 1 frame hoàn chỉnh."""
    if is_complete:
        health.on_frame(is_complete=True)
        if exporter is not None:
            exporter.add_frame(round_id, frame)
        # --- PROCESSOR BRIDGE: gửi raw frame sang csi_processor.py ---
        if _proc_queue is not None:
            try:
                _proc_queue.put_nowait((round_id, frame.copy()))
            except Exception:
                pass  # Queue đầy → bỏ qua, không block receiver
        print(
            f"[FRAME ✓] round={round_id} "
            f"mean_amp={frame.mean():.2f} "
            f"exported={exporter.count if exporter else 0}"
        )
    else:
        health.on_frame(is_complete=False)


def print_summary():
    elapsed = time.time() - start_time if start_time else 0
    print("\n" + "=" * 60)
    print("               KẾT QUẢ THU THẬP CSI")
    print("=" * 60)
    print(f"  Thời gian chạy:          {elapsed:.1f} giây")
    print(f"  Tổng frame:              {n_frames_emitted}")
    print(f"  Frame đủ 12/12 link:     {n_complete_frames}")
    print(f"  Frame thiếu link:        {n_incomplete_frames}")
    print(f"  CRC errors:              {n_crc_errors}")
    if n_frames_emitted > 0:
        pct = n_complete_frames / n_frames_emitted * 100
        print(f"  Tỉ lệ hoàn chỉnh:       {pct:.1f}%")
    if elapsed > 0:
        fps = n_frames_emitted / elapsed
        fps_complete = n_complete_frames / elapsed
        print(f"  Tốc độ trung bình:       {fps:.2f} frame/s (tổng)")
        print(f"  Tốc độ frame đủ:         {fps_complete:.2f} frame/s")
    if exporter is not None:
        print(f"  File CSV:                {exporter.filepath}")
        print(f"  Frames trong CSV:        {exporter.count}")
    print("=" * 60)


def main():
    global start_time, exporter

    exporter = CSVExporter(output_dir="csi_data")

    threads = []
    for dev_id, port in PORTS.items():
        t = threading.Thread(
            target=reader_thread, args=(dev_id, port), daemon=True
        )
        t.start()
        threads.append(t)

    start_time = time.time()
    print("Đang chạy... Ctrl+C để dừng.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        exporter.close()
        print_summary()


if __name__ == "__main__":
    main()
