import matplotlib.pyplot as plt
import re
import math
import numpy as np


def line_matches_filter(line, substring):
    return substring in line

def parse_number(line):
    val = line.split(":")[-1]
    if "iteration" in line:
        return int(val)
    else:
        return float(val)
            
def parse_finger(line):
    fingers = ["index", "middle", "ring", "thumb"]
    for finger in fingers:
        if finger in line:
            return finger

FILE_PATH = "/home/mmatak/py3_catkin_ws/src/hand_services/launch/output.txt"
# parse file
fh = open(FILE_PATH, 'r')
lines = fh.readlines()

F_d_x = {}
F_d_y = {}
F_d_z = {}

F_hat_x = {}
F_hat_y = {}
F_hat_z = {}

F_hat_L2 = {}
F_d_L2 = {}

q_cmd_0 = {}
q_cmd_1 = {}
q_cmd_2 = {}
q_cmd_3 = {}

q_cur_0 = {}
q_cur_1 = {}
q_cur_2 = {}
q_cur_3 = {}

any_useful_line_filter = "[controlInterface]"
force_desired_filter = "F_d"
force_hat_filter = "F_hat"
q_cmd_filter = "q_cmd"
q_cur_filter = "q_cur"
iteration_filter = "iteration"

iteration = 0

ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
# initialize dictionaries
fingers = ["index", "middle", "ring", "thumb"]

for finger in fingers:
    F_d_x[finger] = []
    F_d_y[finger] = []
    F_d_z[finger] = []
    F_d_L2[finger] = []
    F_hat_x[finger] = []
    F_hat_y[finger] = []
    F_hat_z[finger] = []
    F_hat_L2[finger] = []
    q_cmd_0[finger] = []
    q_cmd_1[finger] = []
    q_cmd_2[finger] = []
    q_cmd_3[finger] = []
    q_cur_0[finger] = []
    q_cur_1[finger] = []
    q_cur_2[finger] = []
    q_cur_3[finger] = []

for line in lines:
    line = ansi_escape.sub('', line)
    if not line_matches_filter(line, any_useful_line_filter):
        continue
    if line_matches_filter(line, force_desired_filter):
        if "F_d.x" in line:
            finger = parse_finger(line)
            F_d_x[finger].append(parse_number(line))
        elif "F_d.y" in line:
            finger = parse_finger(line)
            F_d_y[finger].append(parse_number(line))
        elif "F_d.z" in line:
            finger = parse_finger(line)
            F_d_z[finger].append(parse_number(line))
    elif line_matches_filter(line, force_hat_filter):
        if "F_hat.x" in line:
            finger = parse_finger(line)
            F_hat_x[finger].append(parse_number(line))
        elif "F_hat.y" in line:
            finger = parse_finger(line)
            F_hat_y[finger].append(parse_number(line))
        elif "F_hat.z" in line:
            finger = parse_finger(line)
            F_hat_z[finger].append(parse_number(line))
    elif line_matches_filter(line, q_cmd_filter):
        if "q_cmd[0]" in line:
            finger = parse_finger(line)
            q_cmd_0[finger].append(parse_number(line))
        if "q_cmd[1]" in line:
            finger = parse_finger(line)
            q_cmd_1[finger].append(parse_number(line))
        if "q_cmd[2]" in line:
            finger = parse_finger(line)
            q_cmd_2[finger].append(parse_number(line))
        if "q_cmd[3]" in line:
            finger = parse_finger(line)
            q_cmd_3[finger].append(parse_number(line))
    elif line_matches_filter(line, q_cur_filter):
        if "q_cur[0]" in line:
            finger = parse_finger(line)
            q_cur_0[finger].append(parse_number(line))
        if "q_cur[1]" in line:
            finger = parse_finger(line)
            q_cur_1[finger].append(parse_number(line))
        if "q_cur[2]" in line:
            finger = parse_finger(line)
            q_cur_2[finger].append(parse_number(line))
        if "q_cur[3]" in line:
            finger = parse_finger(line)
            q_cur_3[finger].append(parse_number(line))
    elif line_matches_filter(line, iteration_filter):
        iteration = parse_number(line)
    else:
        print("not saving: ")
        print(line)

print("iterations: ", iteration)
for finger in fingers:
    '''
    assert len(F_d_x[finger]) == iteration, len(F_d_x[finger])
    assert len(F_d_y[finger]) == iteration, len(F_d_y[finger])
    assert len(F_d_z[finger]) == iteration, len(F_d_z[finger])

    assert len(F_hat_x[finger]) == iteration, len(F_hat_x[finger])
    assert len(F_hat_y[finger]) == iteration, len(F_hat_y[finger])
    assert len(F_hat_z[finger]) == iteration, len(F_hat_z[finger])

    assert len(q_cmd_0[finger]) == iteration, len(q_cmd_0[finger])
    assert len(q_cmd_1[finger]) == iteration, len(q_cmd_1[finger])
    assert len(q_cmd_2[finger]) == iteration
    assert len(q_cmd_3[finger]) == iteration

    assert len(q_cur_0[finger]) == iteration, len(q_cur_0[finger])
    assert len(q_cur_1[finger]) == iteration
    assert len(q_cur_2[finger]) == iteration
    assert len(q_cur_3[finger]) == iteration
    '''

    # convert to numpy
    F_d_x[finger] = np.array(F_d_x[finger])
    F_d_y[finger] = np.array(F_d_y[finger])
    F_d_z[finger] = np.array(F_d_z[finger])

    F_hat_x[finger] = np.array(F_hat_x[finger])
    F_hat_y[finger] = np.array(F_hat_y[finger])
    F_hat_z[finger] = np.array(F_hat_z[finger])

    for i in range(len(F_hat_x[finger])):
        F_hat_L2[finger].append(math.sqrt(F_hat_x[finger][i] ** 2 + F_hat_y[finger][i] ** 2 + F_hat_z[finger][i] ** 2))
        F_d_L2[finger].append(math.sqrt(F_d_x[finger][i] ** 2 + F_d_y[finger][i] ** 2 + F_d_z[finger][i] ** 2))


# plot values
fig, ax = plt.subplots(3, 4, sharey="row")
fig.suptitle("Desired (dashed) and read (continuous) values\n First row: forces: x,y,z mapped to R,G,B \n Second row: L2 force \n Third row: joints: 0,1,2,3 mapped to R,G,B,Y")
for i, finger in enumerate(fingers):
    ax[0,i].title.set_text("[" + finger + "]")# forces: x,y,z -> r,g,b")
    # forces 3D
    ax[0,i].plot(range(iteration), F_hat_x[finger], 'r-')
    ax[0,i].plot(range(iteration), F_hat_y[finger], 'g-')
    ax[0,i].plot(range(iteration), F_hat_z[finger], 'b-')

    ax[0,i].plot(range(iteration), F_d_x[finger], 'r--')
    ax[0,i].plot(range(iteration), F_d_y[finger], 'g--')
    ax[0,i].plot(range(iteration), F_d_z[finger], 'b--')

    # L2 
    ax[1,i].plot(range(iteration), F_hat_L2[finger], 'b-')
    ax[1,i].plot(range(iteration), F_d_L2[finger], 'b--')

    # joints
    ax[2,i].plot(range(iteration), q_cmd_0[finger], 'r-')
    ax[2,i].plot(range(iteration), q_cmd_1[finger], 'g-')
    ax[2,i].plot(range(iteration), q_cmd_2[finger], 'b-')
    ax[2,i].plot(range(iteration), q_cmd_3[finger], 'y-')

    ax[2,i].plot(range(iteration), q_cur_0[finger], 'r--')
    ax[2,i].plot(range(iteration), q_cur_1[finger], 'g--')
    ax[2,i].plot(range(iteration), q_cur_2[finger], 'b--')
    ax[2,i].plot(range(iteration), q_cur_3[finger], 'y--')

ax[0,0].set_ylabel("Force [N]")
ax[1,0].set_ylabel("Force magnitude [N]")
ax[2,0].set_ylabel("joint position [rad]")
fig.set_size_inches(18.5, 10.5)

plt.savefig("/home/mmatak/forces-plot")



