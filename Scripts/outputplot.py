import matplotlib.pyplot as plt

print("Starting")

measured_value = []
time = []
max_pulse = []
time_max_pulse = []
curve_direction = 'down'
actual_sample = 0
last_sample = 0
min_pulse_threshold = 200
max_value = 0
number_of_pulses = 0

with open('output.txt', 'r') as f:
    read_data = f.readlines()
    for line in read_data:
        splited_line = line.split(" ")
        actual_sample = float(splited_line[0])
        measured_value.append(actual_sample)
        time.append(float(splited_line[1]))
        if float(splited_line[1]) > 7.5:
            if actual_sample > min_pulse_threshold:
                if actual_sample > last_sample:
                    max_value = actual_sample
                    curve_direction = "up"
                else:
                    if curve_direction == "up":
                        max_pulse.append(max_value)
                        time_max_pulse.append(float(splited_line[1]))
                        max_value = 0
                        curve_direction = "down"
        last_sample = actual_sample

    # print(len(max_pulse))

    plt.plot(time, measured_value, time_max_pulse, max_pulse, 'o')
    plt.axis([7.5, 32, -400, 800])
    plt.grid()
    plt.show()

for i in range(len(time_max_pulse)-1):
    dt = time_max_pulse[i+1] - time_max_pulse[i]
    aux = 60/dt
    print(aux)