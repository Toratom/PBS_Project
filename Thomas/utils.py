import taichi as ti
import numpy as np
import math

###### QUATERNIONS ######
# Quaternions are represented with taichi 4-vectors
# with the format q = ti.Vector([x, y, z, w]) = w + ix + jy + kz


@ti.func
def quaternion_multiply(p, q):
    ret = ti.Vector(
        [
            p.x * q.w + p.w * q.x + p.y * q.z - p.z * q.y,
            p.y * q.w + p.w * q.y + p.z * q.x - p.x * q.z,
            p.z * q.w + p.w * q.z + p.x * q.y - p.y * q.x,
            p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z,
        ]
    )
    return ret

@ti.func
def quaternion_to_matrix(q):
    # First row of the rotation matrix
    r00 = 2. * (q.w * q.w + q.x * q.x) - 1.
    r01 = 2. * (q.x * q.y - q.w * q.z)
    r02 = 2. * (q.x * q.z + q.w * q.y)

    # Second row of the rotation matrix
    r10 = 2. * (q.x * q.y + q.w * q.z)
    r11 = 2. * (q.w * q.w + q.y * q.y) - 1.
    r12 = 2. * (q.y * q.z - q.w * q.x)

    # Third row of the rotation matrix
    r20 = 2. * (q.x * q.z - q.w * q.y)
    r21 = 2. * (q.y * q.z + q.w * q.x)
    r22 = 2. * (q.w * q.w + q.z * q.z) - 1.

    # 3x3 rotation matrix
    rot_matrix = ti.Matrix([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])
    return rot_matrix


# @ti.func
# def matrix_to_quaternion(M):
#     qw = 0.5 * ti.sqrt(1 + M[0, 0] + M[1, 1] + M[2, 2])
#     qx = (M[2, 1] - M[1, 2]) / (4 * qw)
#     qy = (M[0, 2] - M[2, 0]) / (4 * qw)
#     qz = (M[1, 0] - M[0, 1]) / (4 * qw)

#     q = ti.Vector([qx, qy, qz, qw])
#     return q

@ti.func
def matrix_to_quaternion(M) :
    #IMPORTANT More stable around PI rad
    # https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm

    tr = M[0, 0] + M[1, 1] + M[2, 2]

    q = ti.Vector([0., 0., 0., 1.])
    if (tr > 0) : 
        S = ti.sqrt(tr + 1.0) * 2. # S=4*qw 
        q.w = 0.25 * S
        q.x = (M[2, 1] - M[1, 2]) / S
        q.y = (M[0, 2] - M[2, 0]) / S
        q.z = (M[1, 0] - M[0, 1]) / S 
    elif ((M[0, 0] > M[1, 1])&(M[0, 0] > M[2, 2])) :
        S = ti.sqrt(1.0 + M[0, 0] - M[1, 1] - M[2, 2]) * 2 # S=4*qx 
        q.w = (M[2, 1] - M[1, 2]) / S
        q.x = 0.25 * S
        q.y = (M[0, 1] + M[1, 0]) / S
        q.z = (M[0, 2] + M[2, 0]) / S 
    elif (M[1, 1] > M[2, 2]) :
        S = ti.sqrt(1.0 + M[1, 1] - M[0, 0] - M[2, 2]) * 2 # S=4*qy
        q.w = (M[0, 2] - M[2, 0]) / S
        q.x = (M[0, 1] + M[1, 0]) / S
        q.y = 0.25 * S
        q.z = (M[1, 2] + M[2, 1]) / S
    else :
        S = ti.sqrt(1.0 + M[2, 2] - M[0, 0] - M[1, 1]) * 2 # S=4*qz
        q.w = (M[1, 0] - M[0, 1]) / S
        q.x = (M[0, 2] + M[2, 0]) / S
        q.y = (M[1, 2] + M[2, 1]) / S
        q.z = 0.25 * S

    return q

@ti.func
def quaternion_to_angle(q) :
    #https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm

    if (q.w > 1) :
        #The given q is not a unit q, needs to be normalized :
        q = q.normalized()
    angle = 2. * ti.acos(q.w)

    #If angle to close to zero returns 0 cf article required for stability
    if (angle < 0.001 or angle > 6.283 - 0.001) :
        angle = 0.

    return angle

@ti.func
def quaternion_to_axis(q) :
    #https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm

    if (q.w > 1.) :
        #The given q is not a unit q, needs to be normalized :
        q = q.normalized()
    s = ti.sqrt(1. - q.w**2)

    axis = ti.Vector([q.x, q.y, q.z])
    if (s > 0.001) : #Test to avoid division by 0, if 0 means angle = 0 so the axis is not significant
        axis.x = q.x / s
        axis.y = q.y / s
        axis.z = q.z / s

    return axis

@ti.func
def quaternion_inverse(q):
    qInv = ti.Vector([-q.x, -q.y, -q.z, q.w]) / q.norm()
    return qInv


###### MISCELLANEOUS ######

# return skew matrix from vector
@ti.func
def skew(a):
    ret = ti.Matrix([[0., -a.z, a.y], [a.z, 0., -a.x], [-a.y, a.x, 0.]])
    return ret


###### COMMAND LINE DISPLAY ######

# print PBS course exercise framework logo and usage
def print_pbs():
    # message: http://patorjk.com/software/taag/#p=display&v=0&f=Roman&t=PBS%2019
    usage = """
    ooooooooo.   oooooooooo.   .oooooo..o        .oooo.     .o  
    `888   `Y88. `888'   `Y8b d8P'    `Y8      .dP""Y88b  o888  
     888   .d88'  888     888 Y88bo.                 ]8P'  888  
     888ooo88P'   888oooo888'  `"Y8888o.           .d8P'   888  
     888          888    `88b      `"Y88b        .dP'      888  
     888          888    .88P oo     .d8P      .oP     .o  888  
    o888o        o888bood8P'  8""88888P'       8888888888 o888o 

    252-0546-00L Physically-Based Simulation in Computer Graphics @ ETH Zurich
    Course Exercise Framework
  
    -- Mouse view control --
    Left button + drag         : Rotate.
    Ctrl + left button + drag  : Translate.
    Wheel button + drag        : Translate.
    Shift + left button + drag : Roll.
    Wheel                      : Zoom in/out.

    -- Keyboard view control --
    [/]          : Increase/decrease field of view.
    R            : Reset simulation & view point.
    Ctrl/Cmd + C : Copy current view status into the clipboard.
    Ctrl/Cmd + V : Paste view status from clipboard.

    -- General control --
    Q, Esc       : Exit window.
    H            : Print help message.
    P, PrtScn    : Take a screen capture.
    D            : Take a depth capture.
    O            : Take a capture of current rendering settings."""
    print(usage)


# print PBS course exercise framework logo and usage
def print_pbs_logo():
    # message: http://patorjk.com/software/taag/#p=display&v=0&f=Roman&t=PBS%2019
    logo = """
    ooooooooo.   oooooooooo.   .oooooo..o        .oooo.     .o
    `888   `Y88. `888'   `Y8b d8P'    `Y8      .dP""Y88b  o888
     888   .d88'  888     888 Y88bo.                 ]8P'  888
     888ooo88P'   888oooo888'  `"Y8888o.           .d8P'   888
     888          888    `88b      `"Y88b        .dP'      888
     888          888    .88P oo     .d8P      .oP     .o  888
    o888o        o888bood8P'  8""88888P'       8888888888 o888o

    252-0546-00L Physically-Based Simulation in Computer Graphics @ ETH Zurich
    Course Exercise Framework
    """
    print(logo)
