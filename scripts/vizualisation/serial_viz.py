"""Live 12x16 thermal heatmap from the firmware's raw serial frame stream.

Frames arrive as the 4-byte magic AA 55 'T' 'T' followed by 192 little-endian
float32 values (see include/serial_frame_stream.hh); the loop below synchronises
on the magic so interleaved text logs are skipped.
"""
import argparse
import time
import numpy as np
import matplotlib.pyplot as plt
import serial

# -------- CONFIG ----------
DEFAULT_COM_PORT = "/dev/cu.usbserial-0247185B"
BAUDRATE = 115200
ROWS, COLS = 12, 16
BYTES_PER_FRAME = ROWS * COLS * 4  # 192 floats * 4 bytes
# Every frame is prefixed with this magic (see include/serial_frame_stream.hh);
# text logs share the port, so we synchronise on it rather than on content.
FRAME_MAGIC = b"\xaa\x55TT"
MIN_TEMP = -40.0   # sanity range for a decoded frame
MAX_TEMP = 150.0
VMIN, VMAX = 20, 40  # fixed color scale, degrees Celsius (matches ble.py)
# --------------------------

parser = argparse.ArgumentParser(description="Live thermal heatmap from serial frames.")
parser.add_argument("-p", "--port", default=DEFAULT_COM_PORT,
                     help=f"Serial port to read from (default: {DEFAULT_COM_PORT})")
parser.add_argument("-b", "--baudrate", type=int, default=BAUDRATE,
                     help=f"Baud rate (default: {BAUDRATE})")
args = parser.parse_args()

ser = serial.Serial(args.port, args.baudrate, timeout=0.05)
buf = bytearray()

running = True

# --- FIGURE WITH TWO SUBPLOTS ---
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))

# Full heatmap
im1 = ax1.imshow(np.zeros((ROWS, COLS)),
                 cmap="inferno",
                #  interpolation="bicubic",
                 vmin=VMIN, vmax=VMAX,
                 aspect="auto")
cbar1 = plt.colorbar(im1, ax=ax1)
ax1.set_title("Full Heatmap")

# Column-average heatmap
im2 = ax2.imshow(np.zeros((ROWS, COLS)),
                 cmap="plasma",
                #  interpolation="bicubic",
                 vmin=VMIN, vmax=VMAX,
                 aspect="auto")
cbar2 = plt.colorbar(im2, ax=ax2)
ax2.set_title("Column Average Heatmap")

# --- HANDLE FIGURE CLOSE ---
def on_close(event):
    """Figure close handler: stop the read loop and release the port."""
    global running
    print("\nFigure closed — exiting...")
    running = False
    if ser.is_open:
        ser.close()
        print("Serial port closed.")

fig.canvas.mpl_connect("close_event", on_close)

plt.show(block=False)

frame_count = 0
t0 = time.time()
fps = 0.0

try:
    while running:
        # read any available bytes (non-blocking-ish)
        n = ser.in_waiting
        if n:
            chunk = ser.read(n)
            buf.extend(chunk)
        else:
            time.sleep(0.001)

        # consume every complete magic-prefixed frame in the buffer
        while True:
            start = buf.find(FRAME_MAGIC)
            if start < 0:
                # keep a partial magic that may straddle the next chunk
                del buf[:max(0, len(buf) - (len(FRAME_MAGIC) - 1))]
                break
            if len(buf) - start < len(FRAME_MAGIC) + BYTES_PER_FRAME:
                del buf[:start]  # wait for the rest of this frame
                break
            payload = buf[start + len(FRAME_MAGIC): start + len(FRAME_MAGIC) + BYTES_PER_FRAME]
            del buf[:start + len(FRAME_MAGIC) + BYTES_PER_FRAME]

            arr = np.frombuffer(bytes(payload), dtype="<f4")
            if not (np.all(np.isfinite(arr)) and arr.min() >= MIN_TEMP and arr.max() <= MAX_TEMP):
                print("dropped a frame that failed the sanity check")
                continue

            matrix = arr.reshape((ROWS, COLS))

            # --- Full heatmap update (fixed VMIN/VMAX color scale, set once above) ---
            im1.set_data(matrix)

            # --- Column-average heatmap ---
            col_avg = np.mean(matrix, axis=0)  # 16 values
            col_matrix = np.tile(col_avg, (ROWS, 1))  # replicate for display
            im2.set_data(col_matrix)

            # Update titles
            ax1.set_title(f"Full Heatmap min:{matrix.min():.2f} max:{matrix.max():.2f}")
            ax2.set_title(f"Column Avg min:{col_avg.min():.2f} max:{col_avg.max():.2f}")

            # Update FPS
            frame_count += 1
            elapsed = time.time() - t0
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                frame_count = 0
                t0 = time.time()
                ax1.set_title(f"Full Heatmap min:{matrix.min():.2f} max:{matrix.max():.2f} fps:{fps:.1f}")

            fig.canvas.draw_idle()
            plt.pause(0.001)

        # avoid unbounded buffer growth if no magic ever shows up
        if len(buf) > 10 * BYTES_PER_FRAME:
            del buf[:-10 * BYTES_PER_FRAME]

except KeyboardInterrupt:
    print("\nInterrupted by user")
finally:
    if ser.is_open:
        ser.close()
        print("Serial port closed.")
