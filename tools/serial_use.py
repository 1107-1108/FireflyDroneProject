import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import re

def open_serial():
    ports = list(serial.tools.list_ports.comports())
    if ports:
        print("Available Port Lists")
        for port in ports:
            print(port.device)
    else:
        print("No avaliable port found")
        return None

    user_port = input("input your port > ")
    user_baudrate = input("input your baud rate(ENTER for default 115200) > ")
    user_baudrate = int(user_baudrate) if user_baudrate else 115200

    serial1 = serial.Serial(user_port, user_baudrate)
    if serial1.is_open:
        print("Serial is opened")
    return serial1

def parse_line(line):
    line = line.strip()
    match = re.findall(r"roll:\s*(-?\d+\.?\d*)[, ]+pitch:\s*(-?\d+\.?\d*)[, ]+yaw:\s*(-?\d+\.?\d*)", line)
    if match:
        r, p, y = match[0]
        return float(r), float(p), float(y)

    return None

roll_data = []
pitch_data = []
yaw_data = []
time_data = []
t = 0

def animate(i, ser):
    global t

    if ser.in_waiting:
        raw = ser.readline().decode(errors="ignore")
        print(raw)
        parsed = parse_line(raw)

        if parsed:
            roll, pitch, yaw = parsed
            roll_data.append(roll)
            pitch_data.append(pitch)
            yaw_data.append(yaw)
            time_data.append(t)
            t += 1

            if len(time_data) > 300:
                time_data.pop(0)
                roll_data.pop(0)
                pitch_data.pop(0)
                yaw_data.pop(0)

    plt.cla()

    plt.plot(time_data, roll_data, label="Roll")
    plt.plot(time_data, pitch_data, label="Pitch")
    plt.plot(time_data, yaw_data, label="Yaw")

    plt.legend()
    plt.grid(True)
    plt.title("Roll / Pitch / Yaw")
    plt.xlabel("Time (ticks)")
    plt.ylabel("Degrees")


def main():
    ser = open_serial()
    if ser is None:
        return

    fig = plt.figure()
    ani = animation.FuncAnimation(fig, animate, fargs=(ser,), interval=50)
    plt.show()


main()
