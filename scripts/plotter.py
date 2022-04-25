import matplotlib.pyplot as plt
import re
import numpy as np


def line_matches_filter(line, substring):
    return substring in line

def parse_number(line):
    val = line.split(":")[-1]
    if "iteration" in line:
        return int(val)
    else:
        return float(val)

FILE_PATH = "/home/mmatak/py3_catkin_ws/src/hand_services/launch/output.txt"
# parse file
fh = open(FILE_PATH, 'r')
lines = fh.readlines()

F_d_x = []
F_d_y = []
F_d_z = []

F_hat_x = []
F_hat_y = []
F_hat_z = []

q_cmd_0 = []
q_cmd_1 = []
q_cmd_2 = []
q_cmd_3 = []

q_cur_0 = []
q_cur_1 = []
q_cur_2 = []
q_cur_3 = []

any_useful_line_filter = "[controlInterface]"
force_desired_filter = "F_d"
force_hat_filter = "F_hat"
q_cmd_filter = "q_cmd"
q_cur_filter = "q_cur"
iteration_filter = "iteration"

iteration = 0

ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')

for line in lines:
    line = ansi_escape.sub('', line)
    if not line_matches_filter(line, any_useful_line_filter):
        continue
    if line_matches_filter(line, force_desired_filter):
        if "F_d.x" in line:
            F_d_x.append(parse_number(line))
        elif "F_d.y" in line:
            F_d_y.append(parse_number(line))
        elif "F_d.z" in line:
            F_d_z.append(parse_number(line))
    elif line_matches_filter(line, force_hat_filter):
        if "F_hat.x" in line:
            F_hat_x.append(parse_number(line))
        elif "F_hat.y" in line:
            F_hat_y.append(parse_number(line))
        elif "F_hat.z" in line:
            F_hat_z.append(parse_number(line))
    elif line_matches_filter(line, q_cmd_filter):
        if "q_cmd[0]" in line:
            q_cmd_0.append(parse_number(line))
        if "q_cmd[1]" in line:
            q_cmd_1.append(parse_number(line))
        if "q_cmd[2]" in line:
            q_cmd_2.append(parse_number(line))
        if "q_cmd[3]" in line:
            q_cmd_3.append(parse_number(line))
    elif line_matches_filter(line, q_cur_filter):
        if "q_cur[0]" in line:
            q_cur_0.append(parse_number(line))
        if "q_cur[1]" in line:
            q_cur_1.append(parse_number(line))
        if "q_cur[2]" in line:
            q_cur_2.append(parse_number(line))
        if "q_cur[3]" in line:
            q_cur_3.append(parse_number(line))
    elif line_matches_filter(line, iteration_filter):
        iteration = parse_number(line)
    else:
        print("not saving: ")
        print(line)

print("iterations: ", iteration)
assert len(F_d_x) == iteration, len(F_d_x)
assert len(F_d_y) == iteration, len(F_d_y)
assert len(F_d_z) == iteration, len(F_d_z)

assert len(F_hat_x) == iteration, len(F_hat_x)
assert len(F_hat_y) == iteration, len(F_hat_y)
assert len(F_hat_z) == iteration, len(F_hat_z)

assert len(q_cmd_0) == iteration, len(q_cmd_0)
assert len(q_cmd_1) == iteration, len(q_cmd_1)
assert len(q_cmd_2) == iteration
assert len(q_cmd_3) == iteration

assert len(q_cur_0) == iteration, len(q_cur_0)
assert len(q_cur_1) == iteration
assert len(q_cur_2) == iteration
assert len(q_cur_3) == iteration

# convert to numpy
F_d_x = np.array(F_d_x)
F_d_y = np.array(F_d_y)
F_d_z = np.array(F_d_z)

F_hat_x = np.array(F_hat_x)
F_hat_y = np.array(F_hat_y)
F_hat_z = np.array(F_hat_z)


# plot values
fig, ax = plt.subplots(2)
plt.subplots_adjust(hspace=0.5)
fig.suptitle("Desired (dashed) and read (continuous) values")
ax[0].title.set_text("Forces: x,y,z -> r,g,b")
ax[0].plot(range(iteration), F_hat_x, 'r-')
ax[0].plot(range(iteration), F_hat_y, 'g-')
ax[0].plot(range(iteration), F_hat_z, 'b-')

ax[0].plot(range(iteration), F_d_x, 'r--')
ax[0].plot(range(iteration), F_d_y, 'g--')
ax[0].plot(range(iteration), F_d_z, 'b--')

ax[1].title.set_text("Joints: 0,1,2,3 -> r,g,b,y")
ax[1].plot(range(iteration), q_cmd_0, 'r-')
ax[1].plot(range(iteration), q_cmd_1, 'g-')
ax[1].plot(range(iteration), q_cmd_2, 'b-')
ax[1].plot(range(iteration), q_cmd_3, 'y-')

ax[1].plot(range(iteration), q_cur_0, 'r--')
ax[1].plot(range(iteration), q_cur_1, 'g--')
ax[1].plot(range(iteration), q_cur_2, 'b--')
ax[1].plot(range(iteration), q_cur_3, 'y--')

plt.savefig("/home/mmatak/forces-plot")



