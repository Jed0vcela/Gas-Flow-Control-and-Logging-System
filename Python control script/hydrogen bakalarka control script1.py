import serial
import threading
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, TextBox
from collections import deque
from queue import Queue
import time

latest_commands = {
    "SP1": None,
    "SP2": None
}

tx_queue = Queue()

# ===== SERIAL CONFIG =====
PORT = "COM41"
BAUDRATE = 115200

ser = serial.Serial(PORT, BAUDRATE, timeout=1)

running = True

# ===== DATA STORAGE =====
MAX_POINTS = 1000

time_data = deque(maxlen=MAX_POINTS)

flow1_data = deque(maxlen=MAX_POINTS)
flow1_sp_data = deque(maxlen=MAX_POINTS)

flow2_data = deque(maxlen=MAX_POINTS)
flow2_sp_data = deque(maxlen=MAX_POINTS)

temp_data = deque(maxlen=MAX_POINTS)
pressure_data = deque(maxlen=MAX_POINTS)


# ===== SERIAL COMMANDS =====
def set_flow1(percent):
    tx_queue.put(("SP1", percent))


def set_flow2(percent):
    tx_queue.put(("SP2", percent))


# ===== SERIAL READER =====
def serial_reader():
    while running:
        try:
            raw = ser.readline()

            if not raw:
                continue

            try:
                line = raw.decode("utf-8", errors="ignore").strip()
            except Exception:
                continue

            if not line:
                continue

            # ===== DEBUG / INFO MESSAGES =====

            # Device responses
            if line.startswith("OK"):
                print("[DEVICE OK]", line)
                continue

            # Debug messages
            if line.startswith("DBG"):
                print("[DEVICE DBG]", line)
                continue

            # Other text messages
            if not line[0].isdigit():
                print("[TEXT]", line)
                continue

            # ===== CSV TELEMETRY =====

            parts = line.split(",")

            if len(parts) != 7:
                print("[BAD CSV]", line)
                continue

            try:
                t_ms = int(parts[0])

                flow1 = int(parts[1])
                flow1_sp = int(parts[2])

                flow2 = int(parts[3])
                flow2_sp = int(parts[4])

                ntc = int(parts[5])
                pressure = int(parts[6])

            except ValueError:
                print("[BAD NUMBER]", line)
                continue

            # ===== UNIT CONVERSIONS =====

            t_s = t_ms / 1000.0

            if flow1 != -1:
                flow1 /= 100.0
            else:
                flow1 = None

            if flow1_sp != -1:
                flow1_sp /= 100.0
            else:
                flow1_sp = None

            if flow2 != -1:
                flow2 /= 100.0
            else:
                flow2 = None

            if flow2_sp != -1:
                flow2_sp /= 100.0
            else:
                flow2_sp = None

            ntc_degC = ntc / 10.0
            pressure_bar = pressure / 100.0

            # ===== STORE DATA =====

            time_data.append(t_s)

            flow1_data.append(flow1)
            flow1_sp_data.append(flow1_sp)

            flow2_data.append(flow2)
            flow2_sp_data.append(flow2_sp)

            temp_data.append(ntc_degC)
            pressure_data.append(pressure_bar)

        except serial.SerialException as e:
            print("[SERIAL ERROR]", e)

        except Exception as e:
            print("[UNKNOWN ERROR]", e)

def serial_writer():
    print("TX THREAD STARTED")

    last_send_time = 0

    while running:

        # ===== COLLECT NEW COMMANDS =====
        try:
            while True:
                cmd_name, percent = tx_queue.get_nowait()

                latest_commands[cmd_name] = percent

        except:
            pass

        # ===== RATE LIMIT =====
        now = time.time()

        if now - last_send_time < 0.3:
            time.sleep(0.01)
            continue

        # ===== SEND LATEST VALUES =====
        for cmd_name in latest_commands:

            percent = latest_commands[cmd_name]

            if percent is None:
                continue

            try:
                value = int(percent * 100)
                value = max(0, min(10000, value))

                cmd = f"{cmd_name} {value}\n"

                print("TX RAW:", repr(cmd))

                ser.write(cmd.encode("utf-8"))
                ser.flush()

            except Exception as e:
                print("WRITE ERROR:", e)

        last_send_time = now

# ===== START SERIAL THREAD =====
thread = threading.Thread(target=serial_reader, daemon=True)
thread.start()

tx_thread = threading.Thread(
    target=serial_writer,
    daemon=True
)

tx_thread.start()

# ===== GUI =====
plt.ion()

fig, ax = plt.subplots(figsize=(12, 7))
plt.subplots_adjust(bottom=0.30)

# ===== SLIDERS =====
ax_sp1 = plt.axes([0.15, 0.18, 0.7, 0.03])
ax_sp2 = plt.axes([0.15, 0.12, 0.7, 0.03])

slider_sp1 = Slider(ax_sp1, 'SP1 %', 0, 100, valinit=0)
slider_sp2 = Slider(ax_sp2, 'SP2 %', 0, 100, valinit=0)

# ===== TEXT BOXES =====
ax_box1 = plt.axes([0.88, 0.18, 0.08, 0.04])
ax_box2 = plt.axes([0.88, 0.12, 0.08, 0.04])

text_box1 = TextBox(ax_box1, "")
text_box2 = TextBox(ax_box2, "")


# ===== CALLBACKS =====
def slider1_changed(val):
    set_flow1(val)
    text_box1.set_val(f"{val:.1f}")


def slider2_changed(val):
    set_flow2(val)
    text_box2.set_val(f"{val:.1f}")


def textbox1_submit(text):
    try:
        value = float(text)
        slider_sp1.set_val(value)
    except:
        pass


def textbox2_submit(text):
    try:
        value = float(text)
        slider_sp2.set_val(value)
    except:
        pass


last_sp1 = None
last_sp2 = None


def slider1_changed(val):
    global last_sp1

    val = round(val, 1)

    if val == last_sp1:
        return

    last_sp1 = val

    text_box1.set_val(f"{val:.1f}")

    print("QUEUE SP1:", val)

    set_flow1(val)


def slider2_changed(val):
    global last_sp2

    val = round(val, 1)

    if val == last_sp2:
        return

    last_sp2 = val

    text_box2.set_val(f"{val:.1f}")

    print("QUEUE SP2:", val)

    set_flow2(val)


slider_sp1.on_changed(slider1_changed)
slider_sp2.on_changed(slider2_changed)

text_box1.on_submit(textbox1_submit)
text_box2.on_submit(textbox2_submit)


# ===== MAIN LOOP =====
while running:
    try:
        ax.clear()

        ax.plot(time_data, flow1_data, label="Flow1")
        ax.plot(time_data, flow1_sp_data, label="SP1")

        ax.plot(time_data, flow2_data, label="Flow2")
        ax.plot(time_data, flow2_sp_data, label="SP2")

        ax.plot(time_data, temp_data, label="Temp [°C]")
        ax.plot(time_data, pressure_data, label="Pressure [bar]")

        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Value")

        ax.grid(True)
        ax.legend()
        ax.autoscale_view()

        plt.pause(0.05)

    except KeyboardInterrupt:
        running = False

ser.close()
print("Stopped")