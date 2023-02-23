############## Import statements #############
import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import csv
# import pandas as pd
import time

########################################################################
################# working with mujoco file ###############
xml_path = 'jlr.xml'    #xml file (assumes this is in the same folder as this file)
simend = 20               #simulation time
print_camera_config = 0  #set to 1 to print camera config
                          #this is useful for initializing view of the model)

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

def sin(x):
    return np.sin(x)

def cos(x):
    return np.cos(x)

def jacob(q1,q2,q3,q4):

    j = np.zeros((3,4))
    j[0,0] = (261*sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)))/1000 - (261*cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/1000 - (8*sin(q1)*sin(q2))/25 - (7*cos(q2)*sin(q1)*sin(q3))/20 - (7*cos(q3)*sin(q1)*sin(q2))/20;

    j[0,1] = (8*cos(q1)*cos(q2))/25 + (261*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/1000 - (261*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/1000 + (7*cos(q1)*cos(q2)*cos(q3))/20 - (7*cos(q1)*sin(q2)*sin(q3))/20;

    j[0,2] = (261*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/1000 - (261*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/1000 + (7*cos(q1)*cos(q2)*cos(q3))/20 - (7*cos(q1)*sin(q2)*sin(q3))/20;

    j[0,3] = (261*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/1000 - (261*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/1000;

    j[1,0] = (8*cos(q1)*sin(q2))/25 + (261*cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/1000 + (261*sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/1000 + (7*cos(q1)*cos(q2)*sin(q3))/20 + (7*cos(q1)*cos(q3)*sin(q2))/20;

    j[1,1] = (8*cos(q2)*sin(q1))/25 - (261*cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)))/1000 - (261*sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/1000 - (7*sin(q1)*sin(q2)*sin(q3))/20 + (7*cos(q2)*cos(q3)*sin(q1))/20;

    j[1,2] = (7*cos(q2)*cos(q3)*sin(q1))/20 - (261*sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/1000 - (7*sin(q1)*sin(q2)*sin(q3))/20 - (261*cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)))/1000;

    j[1,3] = -(261*cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)))/1000 - (261*sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/1000;

    j[2,0] = 0;

    j[2,1] = -(8*sin(q2))/25 - (7*cos(q2)*sin(q3))/20 - (7*cos(q3)*sin(q2))/20 - (261*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))/1000 - (261*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))/1000;

    j[2,2] = -(7*cos(q2)*sin(q3))/20 - (7*cos(q3)*sin(q2))/20 - (261*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))/1000 - (261*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))/1000;

    j[2,3] = -(261*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))/1000 - (261*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))/1000;

    return j

def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):
    #put the controller here. This function is called inside the simulation.
    pass


def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)


#get the full path
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1080, 720, "Snake Simulation", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_100.value)

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

# setting camera configuration
cam.azimuth = 150.7775590551181 ; cam.elevation = -49.887795275590776 ; cam.distance =  4.818166273932765
cam.lookat =np.array([ 0.0 , 0.0 , 0.0 ])


#initialize the controller here. This function is called once, in the beginning
init_controller(model,data)

#set the controller
mj.set_mjcb_control(controller)

#initialize the model parameters 

# df = pd.read_csv('input.csv')
# angles = df.iloc[0,1:9].to_numpy().reshape(8,1)
# goal = df.iloc[1,1:4].to_numpy().reshape(3,1)

goal = np.array([0.1, 0.70, 0.96])

initial_angles = np.zeros(4)

for i in range(4):
    data.qpos[i] = initial_angles[i]

# dq_prev = np.zeros((4,1))
mj.mj_forward(model,data)
pos_ee = data.site_xpos[0]

# print(pos_ee,"\n")

filepath1 = os.path.join(dirname + "/" + "xyz_file_manip.csv")
filepath2 = os.path.join(dirname + "/" + "jang_file_manip.csv")
filepath3 = os.path.join(dirname + "/" + "delq_file_manip.csv")
filepath4 = os.path.join(dirname + "/" + "err_file_manip.csv")
filepath5 = os.path.join(dirname + "/" + "jvel_file_manip.csv")

xyz_file = open(filepath1, "w")
jang_file = open(filepath2, "w")
delq_file = open(filepath3, "w")
err_file = open(filepath4, "w")
jvel_file = open(filepath5, "w")

xyz_file.write("Timestamp, X, Y, Z\n")
jang_file.write("Timestamp, j1, j2, j3, j4\n")
delq_file.write("Timestamp, jdel_1, del_j2, del_j3, del_j4\n")
err_file.write("Timestamp, delX, delY, delZ\n")
jvel_file.write("Timestamp, jvel_1, jvel_2, jvel_3, jvel_4\n")

tolerance = 1e-2
flag = False    # flag to check if goal is reached or not 
start_time = time.time()
t0 = 0
dt = 0.1

while True:

    while not glfw.window_should_close(window):
        
        current_time = time.time()
        elapsed_time = current_time - start_time
        if elapsed_time >= dt:
            start_time = current_time
            t0 += 0.1
        else:
            continue

        pos_ee = data.site_xpos[0]
        # j = np.zeros((3,4))
        # mj.mj_jac(model, data, j, None, pos_ee, 4)
        j = jacob(data.qpos[0],data.qpos[1],data.qpos[2],data.qpos[3])
        jpinv = np.linalg.pinv(j)
        # print(j,'\n')
        error = np.array([goal[0]-pos_ee[0], goal[1]-pos_ee[1], goal[2]-pos_ee[2]])
        dX = error.reshape(3,1)
        
        if (np.linalg.norm(error) < tolerance):
            print('Goal reached !!\n\nError in Position of end effector is :\n',error)
            flag = True
            break

        # print(jpinv,"\n")

        dq = jpinv @ dX

        for k in range(4):
            while(abs(dq[k][0]) >= 0.0085):
                dq[k][0] /= 10
        
        # print(dq,"\n")
        # val = f'{round(t0,2)},{data.qpos[0]},{data.qpos[1]},{data.qpos[2]},{data.qpos[3]}\n'
        
        del_q = f'{round(t0,2)},{dq[0][0]},{dq[1][0]},{dq[2][0]},{dq[3][0]}\n'

        q = f'{round(t0,2)},{data.qpos[0]},{data.qpos[1]},{data.qpos[2]},{data.qpos[3]}\n'
        
        xyz = f'{round(t0,2)},{pos_ee[0]},{pos_ee[1]},{pos_ee[2]}\n'
        
        err = f'{round(t0,2)},{goal[0]-pos_ee[0]},{goal[1]-pos_ee[1]},{goal[2]-pos_ee[2]}\n'
        
        jvel = f'{round(t0,2)},{dq[0][0]/dt},{dq[1][0]/dt},{dq[2][0]/dt},{dq[3][0]/dt}\n'

        # er = f'{dX[0]},{dX[1]},{dX[2]}\n'
        # val = f'{round(t0,2)}, {pos_ee[0]}, {pos_ee[1]}, {pos_ee[2]}\n'
        
        xyz_file.write(xyz)
        jang_file.write(q)
        delq_file.write(del_q)
        err_file.write(err)
        jvel_file.write(jvel)

        for i in range(4):
            data.qpos[i] += dq[i][0]
            # if (i > 0):
            #     if (data.qpos[i] >= 1.22):
            #         data.qpos[i] = 1.22
            #     elif (data.qpos[i] < -1.22):
            #         data.qpos[i] = -1.22

        mj.mj_forward(model, data)
            

        # get framebuffer viewport
        viewport_width, viewport_height = glfw.get_framebuffer_size(window)
        viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

        # # print camera configuration (help to initialize the view)
        if (print_camera_config==1):
            print('cam.azimuth =',cam.azimuth,';','cam.elevation =',cam.elevation,';','cam.distance = ',cam.distance)
            print('cam.lookat =np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')
            print('\n\n')

        # # Update scene and render
        mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
        mj.mjr_render(viewport, scene, context)

        ## swap OpenGL buffers (blocking call due to v-sync)
        glfw.swap_buffers(window)

        ## process pending GUI events, call GLFW callbacks
        glfw.poll_events()
    if (flag == True):
        time.sleep(2)   # model will be displayed till 5 seconds after goal is reached
        print('\nFinal position of end effector is :\n',data.site_xpos[0])
        
        xyz_file.close()
        jang_file.close()
        delq_file.close()
        err_file.close()
        jvel_file.close()

        break

    glfw.terminate()

