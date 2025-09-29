import time
import numpy as np
import matplotlib.pyplot as plt
import serial

# -------- CONFIG ----------
COM_PORT = "COM4"
BAUDRATE = 115200
ROWS, COLS = 12, 16
BYTES_PER_FRAME = ROWS * COLS * 4  # 192 floats * 4 bytes
MIN_TEMP = -40.0   # validation range
MAX_TEMP = 150.0
VMIN, VMAX = 20, 50  # initial color scale
# --------------------------

ser = serial.Serial(COM_PORT, BAUDRATE, timeout=0.05)
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

        # try to find one or more aligned valid frames in buffer
        offset = 0
        buf_len = len(buf)
        while buf_len - offset >= BYTES_PER_FRAME:
            candidate = buf[offset: offset + BYTES_PER_FRAME]
            arr = np.frombuffer(bytes(candidate), dtype='<f4')
            if arr.size != ROWS * COLS:
                offset += 1
                continue

            if np.all(np.isfinite(arr)) and arr.min() >= MIN_TEMP and arr.max() <= MAX_TEMP:
                # reshape full matrix
                matrix = arr.reshape((ROWS, COLS))

                # --- Full heatmap update ---
                im1.set_clim(vmin=np.min(matrix), vmax=np.max(matrix))
                im1.set_data(matrix)

                # --- Column-average heatmap ---
                col_avg = np.mean(matrix, axis=0)  # 16 values
                col_matrix = np.tile(col_avg, (ROWS, 1))  # replicate for display
                im2.set_clim(vmin=np.min(col_matrix), vmax=np.max(col_matrix))
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

                offset += BYTES_PER_FRAME
            else:
                offset += 1

        # drop consumed bytes from buffer
        if offset:
            buf = buf[offset:]

        # avoid unbounded buffer growth
        if len(buf) > 10 * BYTES_PER_FRAME:
            buf = buf[-10 * BYTES_PER_FRAME :]

except KeyboardInterrupt:
    print("\nInterrupted by user")
finally:
    if ser.is_open:
        ser.close()
        print("Serial port closed.")
