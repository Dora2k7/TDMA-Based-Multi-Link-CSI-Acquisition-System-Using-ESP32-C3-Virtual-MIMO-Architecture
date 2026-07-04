"""
csi_processor.py  (v1.0)
========================
Script xử lý nhiễu CSI realtime theo pipeline bài báo WiPowerSys:
    Bước 1 – Hampel Filter  : Loại bỏ spike đột biến (MAD-based)
    Bước 2 – Gaussian Filter: Làm mịn nhiễu điện từ nền

Tính năng:
  • Nhận frame 52×12 thô từ csi_receiver.py qua queue.Queue (thread-safe)
  • Lọc online với sliding window (độ trễ ≤ window//2 frame ≈ 60ms @ 40 FPS)
  • Xuất file CSV mới 52×12 đã lọc (csi_YYYYMMDD_HHMMSS_filtered.csv)
  • Đồ thị realtime scrolling: đường xanh (raw) vs đường cam (filtered)
    → So sánh hiệu quả lọc trực quan như hình mẫu trong bài báo

Cách chạy:
  python csi_processor.py               ← chạy toàn bộ (receiver + processor)
  python csi_receiver.py                ← chạy thuần thu thập (không cần processor)

Yêu cầu:
  pip install scipy matplotlib numpy pyserial
"""

# ─── Standard / Third-party ───────────────────────────────────────────────────
import os
import sys
import csv
import time
import threading
import queue

import numpy as np
import matplotlib
matplotlib.use("TkAgg")          # Backend ổn định cho Windows real-time plot
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
from scipy.ndimage import gaussian_filter1d

# ─── Local ───────────────────────────────────────────────────────────────────
sys.path.insert(0, os.path.dirname(__file__))
import csi_receiver

# ═══════════════════════════════════════════════════════════════════════════════
# ⚙  CẤU HÌNH — chỉnh tại đây, không cần sửa code bên dưới
# ═══════════════════════════════════════════════════════════════════════════════

# --- Hampel ---
HAMPEL_WINDOW = 7        # Kích thước cửa sổ (số frame) tăng lên để lọc nhiễu tốt hơn trong khu dân cư
HAMPEL_SIGMAS = 2.5      # Giảm nhẹ hệ số sigma để nhạy bén hơn với các nhiễu WiFi khác

# --- Gaussian ---
GAUSSIAN_SIGMA = 0.8     # Tăng sigma để khử nhiễu nhỏ từ gió/điều hòa trong phòng thí nghiệm

# --- Temporal Low-pass (EMA) ---
EMA_ALPHA = 0.5          # Tăng alpha lên 0.5 để giảm độ trễ tối đa (PCA đã khử nhiễu không gian rất tốt)

# --- PCA ---
PCA_K = 5                # Số thành phần chính giữ lại (giảm từ 12 links xuống 5)
PCA_EMA_DETREND = 0.05   # [FIX] Tăng từ 0.01→0.05: τ=20 frames, hội tụ nhanh hơn (cũ: τ=100)
PCA_EMA_COV    = 0.05   # [FIX] Tăng từ 0.02→0.05: τ=20 frames, covariance ổn định nhanh hơn
PCA_WARMUP_FRAMES = 5    # [FIX] Số frame thu thập trước khi bắt đầu PCA (tránh bug centered=0)

# --- Đồ thị ---
PLOT_HISTORY  = 300      # Số frame hiển thị trên đồ thị (scrolling window)
PLOT_INTERVAL = 80       # Cập nhật đồ thị mỗi N ms (12.5 FPS plot refresh)

# Subcarrier và link nào hiển thị trên đồ thị (index 0-based trong 52×12 thô hoặc 52×5 PCA)
MONITOR_SC    = 20       # Subcarrier thứ 20 trong 52 subcarrier hữu ích
MONITOR_LINK  = 0        # Link thứ 0 (Link thô 0-11 hoặc thành phần PCA 0-4)

# --- Queue ---
QUEUE_MAXSIZE = 400      # Tối đa số frame raw đang chờ xử lý (400 frame ≈ 10s buffer)

# --- Hệ thống ---
N_SC_CLEAN = 52
N_LINKS    = 12          # 12 link ban đầu từ thiết bị

# ═══════════════════════════════════════════════════════════════════════════════
# 🔧  ONLINE HAMPEL FILTER
# ═══════════════════════════════════════════════════════════════════════════════
class OnlineHampel:
    """
    Online Hampel Filter hoạt động trên sliding window của frames CSI.

    Nguyên lý:
        Giữ một deque (vòng đệm) chứa HAMPEL_WINDOW frame gần nhất.
        Khi đủ window, lấy frame ở giữa làm "frame cần xét":
          - Tính median và MAD theo chiều thời gian (axis=0) cho tất cả 52×12 điểm cùng lúc
          - Điểm nào lệch > n_sigmas × 1.4826 × MAD → thay bằng median
        Độ trễ = window//2 frame = 2 frame ≈ 50ms @ 40FPS (chấp nhận được).

    Args:
        window_size: số frame trong cửa sổ (phải lẻ để có center rõ ràng)
        n_sigmas   : hệ số ngưỡng (mặc định 3 theo tiêu chuẩn Hampel)
    """
    def __init__(self, window_size: int = HAMPEL_WINDOW, n_sigmas: float = HAMPEL_SIGMAS):
        if window_size % 2 == 0:
            window_size += 1       # Đảm bảo lẻ để có center frame
        self.window  = window_size
        self.half    = window_size // 2
        self.sigmas  = n_sigmas
        self._buf    = deque(maxlen=window_size)  # Mỗi phần tử: ndarray (52, 12)

    def push(self, frame: np.ndarray):
        """
        Đẩy 1 frame mới.
        Returns:
            (filtered_frame, raw_center_frame) nếu đủ window,
            (None, None) nếu buffer chưa đầy.
        """
        self._buf.append(frame)
        if len(self._buf) < self.window:
            return None, None

        stacked  = np.stack(self._buf, axis=0)           # (W, 52, 12)
        center   = stacked[self.half].copy()             # Frame cần lọc
        median   = np.median(stacked, axis=0)            # (52, 12)
        mad      = np.median(np.abs(stacked - median), axis=0)  # (52, 12)

        # Ngưỡng: 3 × 1.4826 × MAD (hệ số 1.4826 để MAD tương đương σ với Gaussian)
        threshold = self.sigmas * 1.4826 * mad
        # Tránh chia cho 0 khi MAD = 0 (tín hiệu phẳng)
        mask = (mad > 1e-9) & (np.abs(center - median) > threshold)

        filtered = center.copy()
        filtered[mask] = median[mask]
        return filtered, center      # Cả hai shape (52, 12)


# ═══════════════════════════════════════════════════════════════════════════════
# 🔧  GAUSSIAN SMOOTHER
# ═══════════════════════════════════════════════════════════════════════════════
def gaussian_smooth_frame(frame: np.ndarray, sigma: float = GAUSSIAN_SIGMA) -> np.ndarray:
    """
    Làm mịn Gaussian theo trục subcarrier (axis=0) cho 1 frame (52×12).
    Áp dụng independent cho từng link.

    Tại sao axis=0 (theo subcarrier)?
        Sau Hampel (theo thời gian), nhiễu điện từ gây ra dao động tần số cao
        GIỮA các subcarrier liền kề. Gaussian theo subcarrier làm mịn các
        gợn sóng đó mà không ảnh hưởng đến temporal dynamics (chuyển động người).
    """
    return gaussian_filter1d(frame.astype(float), sigma=sigma, axis=0)


# ═══════════════════════════════════════════════════════════════════════════════
# 🔧  ONLINE PCA DIMENSIONALITY REDUCTION
# ═══════════════════════════════════════════════════════════════════════════════
class OnlinePCA:
    """
    Phân tích Thành phần Chính (PCA) không gian trực tuyến kết hợp khử xu hướng thời gian
    (Online Spatial PCA with Temporal Detrending) để giảm chiều từ 12 links xuống k.

    Các cải tiến (v1.1):
      - Warmup buffer: thu thập PCA_WARMUP_FRAMES frame đầu để tính mean thực,
        tránh bug centered=0 ở frame đầu tiên (running_mean=X → X-X=0).
      - EMA_DETREND tăng lên 0.05 (τ=20 frame): hội tụ nhanh hơn cho session ngắn.
      - EMA_COV tăng lên 0.05 (τ=20 frame): covariance ổn định nhanh hơn.
      - Sign Alignment với ngưỡng |dot|<0.1: tránh flip sai khi eigenvector gần vuông góc.
    """
    def __init__(self, k_components: int = PCA_K, ema_detrend: float = PCA_EMA_DETREND,
                 ema_cov: float = PCA_EMA_COV, warmup_frames: int = PCA_WARMUP_FRAMES):
        self.k = k_components
        self.alpha = ema_detrend
        self.beta = ema_cov
        self.running_mean = None          # (52, 12) - trung bình động thời gian thực (DC component)
        self.running_cov = None           # (12, 12) - ma trận hiệp biến động tích lũy
        self.prev_eigenvectors = None     # (12, k) - ma trận chiếu của frame trước

        # [FIX] Warmup buffer: tránh bug centered=0 ở frame PCA đầu tiên
        self._warmup_frames = warmup_frames
        self._warmup_buf: list = []       # Thu thập frame đầu để tính mean thực sự
        self._warmup_done = False
        self.n_skipped = 0                # Đếm frame bị skip trong warm-up

    def fit_transform(self, frame: np.ndarray):
        """
        Giảm chiều frame từ (52, 12) thành (52, k) bằng cách:
          1. [FIX] Warm-up: thu thập PCA_WARMUP_FRAMES frame đầu, tính mean thực.
          2. Khử thành phần tĩnh (DC detrending) theo trục thời gian.
          3. Cập nhật ma trận hiệp biến không gian (12×12) đệ quy bằng EMA.
          4. Phân rã trị riêng, sắp xếp và đồng bộ dấu (với ngưỡng).
          5. Chiếu tín hiệu động.

        Returns:
            np.ndarray (52, k) nếu đã qua warm-up, None nếu đang warm-up.
        """
        # Đảm bảo đầu vào là float
        X = frame.astype(float)

        # ── [FIX] Bước 0: Warm-up — thu thập frame để khởi tạo mean thực ──────
        if not self._warmup_done:
            self._warmup_buf.append(X)
            self.n_skipped += 1
            if len(self._warmup_buf) >= self._warmup_frames:
                # Tính mean thực từ tập warm-up, tránh running_mean=X[0] → centered=0
                self.running_mean = np.mean(self._warmup_buf, axis=0)  # (52, 12)
                self._warmup_done = True
                self._warmup_buf.clear()   # Giải phóng bộ nhớ
            return None   # Báo hiệu "đang warm-up, bỏ qua frame này"

        # 1. Cập nhật trung bình động thời gian thực (khử tĩnh - Detrending)
        self.running_mean = (1.0 - self.alpha) * self.running_mean + self.alpha * X

        # Tín hiệu động (AC component)
        centered = X - self.running_mean

        # 2. Tính ma trận hiệp biến không gian tức thời (trung bình qua 52 subcarriers)
        cov_instant = (centered.T @ centered) / max(1, centered.shape[0] - 1)  # (12, 12)

        # 3. Cập nhật ma trận hiệp biến tích lũy đệ quy (EMA)
        if self.running_cov is None:
            self.running_cov = cov_instant.copy()
        else:
            self.running_cov = (1.0 - self.beta) * self.running_cov + self.beta * cov_instant

        # 4. Phân rã eigenvalues và eigenvectors
        try:
            eigenvalues, eigenvectors = np.linalg.eigh(self.running_cov)
            # Sắp xếp eigenvalues và eigenvectors giảm dần
            idx = np.argsort(eigenvalues)[::-1]
            eigenvectors = eigenvectors[:, idx]
            P = eigenvectors[:, :self.k]  # (12, k)
        except np.linalg.LinAlgError:
            # Fallback nếu giải phương trình đặc trưng lỗi
            if self.prev_eigenvectors is not None:
                P = self.prev_eigenvectors.copy()
            else:
                P = np.zeros((centered.shape[1], self.k))
                np.fill_diagonal(P, 1.0)

        # 5. [FIX] Căn chỉnh dấu (Sign Alignment) với ngưỡng — tránh flip sai
        #    khi eigenvector mới gần vuông góc với cũ (|dot| < 0.1)
        if self.prev_eigenvectors is not None:
            for j in range(self.k):
                dot = np.dot(P[:, j], self.prev_eigenvectors[:, j])
                if dot < -0.1:
                    # Rõ ràng ngược chiều → flip
                    P[:, j] = -P[:, j]
                elif abs(dot) <= 0.1:
                    # Gần vuông góc → giữ nguyên chiều của eigenvector cũ
                    # bằng cách chiếu lên prev và theo dấu đó
                    sign = np.sign(np.dot(P[:, j], self.prev_eigenvectors[:, j]))
                    if sign == 0:
                        sign = 1.0
                    P[:, j] = sign * P[:, j]
        self.prev_eigenvectors = P.copy()

        # 6. Chiếu dữ liệu động lên không gian k-chiều
        return centered @ P


# ═══════════════════════════════════════════════════════════════════════════════
# 💾  FILTERED CSV EXPORTER (PCA 52x5)
# ═══════════════════════════════════════════════════════════════════════════════
class FilteredCSVExporter:
    """
    Ghi file CSV 52×5 sau khi đã lọc nhiễu và giảm chiều bằng PCA.
    Header: round_id, timestamp, pca_comp0_sc0_filt, ..., pca_comp4_sc51_filt
    Flush mỗi 50 frame để tránh mất dữ liệu khi crash.
    """
    FLUSH_EVERY = 50

    def __init__(self, output_dir: str = "csi_data", k_components: int = PCA_K):
        os.makedirs(output_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.filepath = os.path.join(output_dir, f"csi_{ts}_filtered.csv")
        self._file    = open(self.filepath, "w", newline="", encoding="utf-8")
        self._writer  = csv.writer(self._file)
        self.count    = 0
        self.k        = k_components

        # Build header
        header = ["round_id", "timestamp_s"]
        for pca_idx in range(self.k):
            for sc in range(N_SC_CLEAN):
                header.append(f"pca_comp{pca_idx}_sc{sc:02d}_filt")
        self._writer.writerow(header)
        print(f"[EXPORTER] Filtered CSV (PCA) → {self.filepath}")

    def add_frame(self, round_id: int, filtered: np.ndarray) -> None:
        """filtered shape: (52, k), float32."""
        row = [round_id, f"{time.time():.6f}"]
        # Flatten theo thứ tự (pca_comp, sc)
        for k in range(self.k):
            for sc in range(N_SC_CLEAN):
                row.append(f"{filtered[sc, k]:.4f}")
        self._writer.writerow(row)
        self.count += 1
        if self.count % self.FLUSH_EVERY == 0:
            self._file.flush()

    def close(self) -> None:
        self._file.flush()
        self._file.close()
        print(f"[EXPORTER] Closed — {self.count} filtered PCA frames → {self.filepath}")


# ═══════════════════════════════════════════════════════════════════════════════
# ⚙  PROCESSOR THREAD
# ═══════════════════════════════════════════════════════════════════════════════
class CSIProcessorThread:
    """
    Chạy trên background thread:
      1. Đọc raw frame từ raw_queue (đặt bởi csi_receiver)
      2. Áp Hampel → Gaussian → PCA (Dimensionality reduction 52x12 -> 52x5)
      3. Ghi CSV
      4. Đẩy (raw_center, filtered) vào plot_queue cho Plotter
    """
    def __init__(self, raw_queue: queue.Queue, plot_queue: queue.Queue):
        self.raw_queue    = raw_queue
        self.plot_queue   = plot_queue
        self.hampel       = OnlineHampel()
        self.pca          = OnlinePCA(k_components=PCA_K)
        self.exporter     = FilteredCSVExporter(k_components=PCA_K)
        self._stop_evt    = threading.Event()
        self._thread      = threading.Thread(
            target=self._run, daemon=True, name="csi-processor"
        )

    def start(self) -> None:
        self._thread.start()
        print("[PROCESSOR] Thread started.")

    def stop(self) -> None:
        self._stop_evt.set()
        self._thread.join(timeout=3.0)
        self.exporter.close()

    def _run(self) -> None:
        n_hampel_skip = 0   # [FIX] Đếm frame bị skip do Hampel warm-up
        while not self._stop_evt.is_set():
            try:
                item = self.raw_queue.get(timeout=0.5)
            except Exception:
                continue

            if item is None:   # Poison pill
                break

            round_id, raw_frame = item   # raw_frame: (52, 12) float

            # ── Bước 1: Hampel ──────────────────────────────────────────────
            filtered_h, raw_center = self.hampel.push(raw_frame)
            if filtered_h is None:
                n_hampel_skip += 1
                continue   # Buffer chưa đủ window_size frames

            # ── Bước 2: Gaussian ────────────────────────────────────────────
            filtered_g = gaussian_smooth_frame(filtered_h)

            # ── Bước 3: PCA ─────────────────────────────────────────────────
            filtered_pca = self.pca.fit_transform(filtered_g)

            # [FIX] Bỏ qua frame khi PCA đang trong giai đoạn warm-up
            if filtered_pca is None:
                continue

            # ── Ghi CSV ─────────────────────────────────────────────────────
            self.exporter.add_frame(round_id, filtered_pca)

            # ── Gửi sang Plotter (non-blocking) ─────────────────────────────
            payload = {
                "raw"      : raw_center[:, MONITOR_LINK].copy(),    # (52,)
                "filtered" : filtered_pca[:, MONITOR_LINK].copy(),  # (52,)
                "round_id" : round_id,
            }
            try:
                self.plot_queue.put_nowait(payload)
            except queue.Full:
                pass   # Plot bị lag → bỏ qua frame này, không block processor

        # [FIX] Log thống kê warm-up khi thread kết thúc
        total_skip = n_hampel_skip + self.pca.n_skipped
        print(f"[PROCESSOR] Warm-up frames skipped — "
              f"Hampel: {n_hampel_skip}, PCA: {self.pca.n_skipped}, "
              f"Total: {total_skip}")


# ═══════════════════════════════════════════════════════════════════════════════
# 📊  REALTIME PLOTTER  (chạy trên main thread — yêu cầu của matplotlib)
# ═══════════════════════════════════════════════════════════════════════════════
class RealtimePlotter:
    """
    Đồ thị dark-mode scrolling so sánh Raw vs Filtered.
    Layout giống hình mẫu trong bài báo:
        Trục X: Frame Index (300 frame gần nhất, cập nhật liên tục)
        Trục Y: Amplitude (biên độ CSI, tự động co giãn)
        Đường xanh: tín hiệu thô (Raw CSI — noisy)
        Đường cam:  tín hiệu sau lọc (Filtered CSI — Hampel + Gaussian)
    """

    # ── Màu sắc dark-theme ─────────────────────────────────────────────────
    BG_DARK   = "#0f0f1a"
    BG_AX     = "#161625"
    COL_RAW   = "#4da6ff"   # Xanh dương nhạt — Raw
    COL_FILT  = "#ff8533"   # Cam đậm — Filtered
    COL_GRID  = "#2a2a45"
    COL_TEXT  = "#c8c8e0"
    COL_SPINE = "#3a3a5c"

    def __init__(self, plot_queue: queue.Queue):
        self.pq = plot_queue

        # Sliding buffers
        self._raw_buf  = deque([np.nan] * PLOT_HISTORY, maxlen=PLOT_HISTORY)
        self._filt_buf = deque([np.nan] * PLOT_HISTORY, maxlen=PLOT_HISTORY)
        self._total    = 0          # Tổng frame đã nhận
        self._fps_buf  = deque(maxlen=40)  # Timestamps cho FPS estimate
        self._x        = np.arange(PLOT_HISTORY)
        self._last_ema = None       # Lưu giá trị lọc EMA thời gian thực trước đó
        self._raw_mean = None       # Dùng để detrend raw signal phục vụ hiển thị đồ thị
        self._raw_ema_alpha = 0.01  # Tốc độ cập nhật trung bình tĩnh của raw signal

        self._build_figure()

    # ── Xây dựng figure ────────────────────────────────────────────────────
    def _build_figure(self) -> None:
        plt.style.use("dark_background")
        self.fig, self.ax = plt.subplots(figsize=(14, 5.5))
        self.fig.patch.set_facecolor(self.BG_DARK)
        self.ax.set_facecolor(self.BG_AX)

        # Đường Raw (xanh, mảnh, trong suốt một phần)
        (self.line_raw,) = self.ax.plot(
            self._x, list(self._raw_buf),
            color=self.COL_RAW,
            linewidth=0.85,
            alpha=0.7,
            label=f"Raw CSI  (Noisy)",
            zorder=2,
        )
        # Đường Filtered (cam, dày, nổi bật)
        (self.line_filt,) = self.ax.plot(
            self._x, list(self._filt_buf),
            color=self.COL_FILT,
            linewidth=2.3,
            label="Filtered CSI  (Hampel + Gaussian)",
            zorder=3,
        )

        # Axes styling
        self.ax.set_xlim(0, PLOT_HISTORY - 1)
        self.ax.set_ylim(0, 50)
        self.ax.set_xlabel("Frame Index  (← cũ    mới →)", color=self.COL_TEXT, fontsize=10)
        self.ax.set_ylabel("Amplitude  (|H|)",              color=self.COL_TEXT, fontsize=10)
        self.ax.tick_params(colors=self.COL_TEXT, labelsize=9)
        self.ax.grid(True, color=self.COL_GRID, linewidth=0.5, linestyle="--", alpha=0.8)
        for sp in self.ax.spines.values():
            sp.set_edgecolor(self.COL_SPINE)

        # Legend
        leg = self.ax.legend(
            loc="upper right",
            facecolor="#1c1c30",
            edgecolor=self.COL_SPINE,
            labelcolor=self.COL_TEXT,
            fontsize=10,
            framealpha=0.85,
        )

        # Title (sẽ cập nhật động)
        self._title = self.ax.set_title(
            self._make_title(0, 0.0),
            color="#e0e0ff",
            fontsize=12,
            fontweight="bold",
            pad=10,
        )

        self.fig.tight_layout(pad=1.4)

    # ── Tiêu đề động ───────────────────────────────────────────────────────
    def _make_title(self, frames: int, fps: float) -> str:
        raw_link_label = f"Raw Link {MONITOR_LINK + 1:02d}"
        filt_link_label = f"Filtered Link {MONITOR_LINK + 1:02d}"
        sc_label   = f"SC {MONITOR_SC:02d}"
        return (
            f"CSI Signal Comparison  │  {raw_link_label} vs {filt_link_label}  │  {sc_label}  │  "
            f"Frames: {frames}  │  FPS: {fps:.1f}"
        )

    # ── FuncAnimation callback ─────────────────────────────────────────────
    def _update(self, _frame_idx) -> tuple:
        """Được gọi bởi FuncAnimation mỗi PLOT_INTERVAL ms."""
        updated = False

        # Drain toàn bộ queue (tránh lag tích luỹ khi queue nhiều items)
        while True:
            try:
                data = self.pq.get_nowait()
            except queue.Empty:
                break

            raw_val  = float(data["raw"][MONITOR_SC])
            filt_val = float(data["filtered"][MONITOR_SC])
            
            # Khử thành phần tĩnh (DC) của raw signal để vẽ cùng thang đo với PCA
            if self._raw_mean is None or np.isnan(self._raw_mean):
                self._raw_mean = raw_val
            else:
                self._raw_mean = (1 - self._raw_ema_alpha) * self._raw_mean + self._raw_ema_alpha * raw_val
            raw_display = raw_val - self._raw_mean

            # Bộ lọc Exponential Moving Average (EMA) theo thời gian
            if self._last_ema is None or np.isnan(self._last_ema):
                self._last_ema = filt_val
            else:
                self._last_ema = EMA_ALPHA * filt_val + (1 - EMA_ALPHA) * self._last_ema

            self._raw_buf.append(raw_display)
            self._filt_buf.append(self._last_ema)
            self._total += 1
            self._fps_buf.append(time.monotonic())
            updated = True

        if updated:
            raw_arr  = list(self._raw_buf)
            filt_arr = list(self._filt_buf)

            self.line_raw.set_ydata(raw_arr)
            self.line_filt.set_ydata(filt_arr)

            # Auto-scale Y với padding 15%
            all_vals = [v for v in raw_arr + filt_arr if not np.isnan(v)]
            if len(all_vals) > 1:
                ymin, ymax = min(all_vals), max(all_vals)
                pad = max((ymax - ymin) * 0.15, 1.5)
                self.ax.set_ylim(ymin - pad, ymax + pad)

            # Tính FPS thực tế từ timestamp buffer
            if len(self._fps_buf) >= 2:
                dt  = self._fps_buf[-1] - self._fps_buf[0]
                fps = (len(self._fps_buf) - 1) / dt if dt > 0 else 0.0
            else:
                fps = 0.0

            self._title.set_text(self._make_title(self._total, fps))

        return self.line_raw, self.line_filt, self._title

    # ── Khởi động animation ────────────────────────────────────────────────
    def run(self) -> None:
        self._ani = animation.FuncAnimation(
            self.fig,
            self._update,
            interval=PLOT_INTERVAL,
            blit=False,
            cache_frame_data=False,
            save_count=0,
        )
        plt.show()


# ═══════════════════════════════════════════════════════════════════════════════
# 🚀  ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════
def main() -> None:
    banner = """
╔══════════════════════════════════════════════════════════╗
║         CSI PROCESSOR  v1.1  —  WiPowerSys Pipeline     ║
║   Hampel + Gaussian + Online PCA  +  Realtime Plot      ║
╚══════════════════════════════════════════════════════════╝"""
    print(banner)
    print(f"  Monitor : Link {MONITOR_LINK+1:02d}  │  Subcarrier SC{MONITOR_SC:02d}")
    print(f"  Hampel  : window={HAMPEL_WINDOW}, sigmas={HAMPEL_SIGMAS}")
    print(f"  Gaussian: sigma={GAUSSIAN_SIGMA}")
    print(f"  EMA     : alpha={EMA_ALPHA}")
    print(f"  PCA     : k={PCA_K}, warmup={PCA_WARMUP_FRAMES} frames, "
          f"EMA_detrend={PCA_EMA_DETREND} (τ={1/PCA_EMA_DETREND:.0f} frames), "
          f"EMA_cov={PCA_EMA_COV} (τ={1/PCA_EMA_COV:.0f} frames)")
    total_warmup = HAMPEL_WINDOW + PCA_WARMUP_FRAMES
    print(f"  Warm-up : Hampel {HAMPEL_WINDOW} + PCA {PCA_WARMUP_FRAMES} = {total_warmup} frames skipped at start")
    print(f"  Plot    : {PLOT_HISTORY} frames, refresh {PLOT_INTERVAL}ms")
    print()

    # ── Tạo queues (thread-safe, không cần multiprocessing vì cùng process) ─
    raw_queue  = queue.Queue(maxsize=QUEUE_MAXSIZE)
    plot_queue = queue.Queue(maxsize=80)

    # ── Gắn raw_queue vào csi_receiver ──────────────────────────────────────
    csi_receiver.set_processor_queue(raw_queue)

    # ── Khởi động Processor thread ──────────────────────────────────────────
    processor = CSIProcessorThread(raw_queue, plot_queue)
    processor.start()

    # ── Khởi động CSV Exporter thô của receiver ─────────────────────────────
    csi_receiver.exporter = csi_receiver.CSVExporter(output_dir="csi_data")

    # ── Khởi động các Reader threads của receiver ────────────────────────────
    recv_threads = []
    for dev_id, port in csi_receiver.PORTS.items():
        t = threading.Thread(
            target=csi_receiver.reader_thread,
            args=(dev_id, port),
            daemon=True,
            name=f"recv-dev{dev_id}",
        )
        t.start()
        recv_threads.append(t)
        print(f"[MAIN] Reader thread started: dev{dev_id} @ {port}")

    csi_receiver.start_time = time.time()
    print("\n[MAIN] All threads running. Đang mở cửa sổ đồ thị...\n")
    print("       Đóng cửa sổ đồ thị hoặc nhấn Ctrl+C để dừng.\n")

    # ── Plotter chạy trên main thread (yêu cầu của matplotlib GUI) ──────────
    plotter = RealtimePlotter(plot_queue)
    try:
        plotter.run()
    except KeyboardInterrupt:
        print("\n[MAIN] Ctrl+C nhận được, đang dừng...")
    finally:
        processor.stop()
        if csi_receiver.exporter:
            csi_receiver.exporter.close()
        csi_receiver.print_summary()
        print("[MAIN] Shutdown hoàn tất.")


if __name__ == "__main__":
    main()
