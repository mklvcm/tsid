from gait import *

import pinocchio as se3
import tsid
import numpy as np
import numpy.matlib as matlib
from numpy import nan
from numpy.linalg import norm as norm
import os
import gepetto.corbaserver
import time
import commands
import sys
#sys.path += [os.getcwd()+'/../exercizes']
#import plot_utils as plut
#import matplotlib.pyplot as plt

display = True

np.set_printoptions(precision=3, linewidth=200, suppress=True)

mu = 0.3                        # friction coefficient
fMin = 1.0                      # minimum normal force
fMax = 100.0                    # maximum normal force
contact_frames = ['BL_contact', 'BR_contact', 'FL_contact', 'FR_contact']
contactNormal = np.matrix([0., 0., 1.]).T   # direction of the normal to the contact surface

w_com = 1.0                     # weight of center of mass task
w_posture = 1e-3                # weight of joint posture task
w_forceRef = 1e-5               # weight of force regularization task

kp_contact = 10.0               # proportional gain of contact constraint
kp_com = 10.0                   # proportional gain of center of mass task
kp_posture = 10.0               # proportional gain of joint posture task

dt = 0.001                      # controller time step
PRINT_N = 500                   # print every PRINT_N time steps
DISPLAY_N = 25                  # update robot configuration in viwewer every DISPLAY_N time steps
N_SIMULATION = 6000             # number of time steps simulated

def AttachContact(name):
    i = contact_frames.index(name)
    contacts[i] = tsid.ContactPoint(name, robot, name, contactNormal, mu, fMin, fMax)
    contacts[i].setKp(kp_contact * matlib.ones(3).T)
    contacts[i].setKd(2.0 * np.sqrt(kp_contact) * matlib.ones(3).T)
    H_rf_ref = robot.framePosition(data, robot.model().getFrameId(name))
    contacts[i].setReference(H_rf_ref)
    contacts[i].useLocalFrame(False)
    invdyn.addRigidContact(contacts[i], w_forceRef, 1.0, 1)

def DetachContact(name):
    invdyn.removeRigidContact(name, 0.1)

def SetDesiredCom(gait):
    sampleCom.pos(np.matrix([0.0, gait._comLocation, gait.COM_HEIGHT]).T)
    sampleCom.vel(np.matrix([0.0, speed, 0.0]).T)
    sampleCom.acc(np.matrix([0.0, 0.0, 0.0]).T)
    comTask.setReference(sampleCom)

def SetDesiredJoints(gait):
    pos = matlib.zeros(8).T

    pos[0] = gait._contacts[1]._shAngle
    pos[1] = gait._contacts[1]._knAngle

    pos[2] = gait._contacts[3]._shAngle
    pos[3] = gait._contacts[3]._knAngle

    pos[4] = gait._contacts[2]._shAngle
    pos[5] = gait._contacts[2]._knAngle

    pos[6] = gait._contacts[0]._shAngle
    pos[7] = gait._contacts[0]._knAngle

    samplePosture.pos(pos)
    postureTask.setReference(samplePosture)

    if gait._contacts[0]._justAttached: 
        AttachContact('FR_contact')
    if gait._contacts[0]._justDetached: 
        DetachContact('FR_contact')

    if gait._contacts[1]._justAttached: 
        AttachContact('BL_contact')
    if gait._contacts[1]._justDetached: 
        DetachContact('BL_contact')

    if gait._contacts[2]._justAttached: 
        AttachContact('FL_contact')
    if gait._contacts[2]._justDetached: 
        DetachContact('FL_contact')

    if gait._contacts[3]._justAttached: 
        AttachContact('BR_contact')
    if gait._contacts[3]._justDetached: 
        DetachContact('BR_contact')    

# load urdf

filename = str(os.path.dirname(os.path.abspath(__file__)))
os.chdir(filename)
path = filename + '/../models'
urdf = path + '/quadruped/urdf/quadruped.urdf'
vector = se3.StdVec_StdString()
vector.extend(item for item in path)
robot = tsid.RobotWrapper(urdf, vector, se3.JointModelFreeFlyer(), False)

# for gepetto viewer
robot_display = se3.RobotWrapper.BuildFromURDF(urdf, [path, ], se3.JointModelFreeFlyer())
l = commands.getstatusoutput("ps aux |grep 'gepetto-gui'|grep -v 'grep'|wc -l")
if int(l[1]) == 0:
    os.system('gepetto-gui &')
time.sleep(1)

if display:
    cl = gepetto.corbaserver.Client()
    gui = cl.gui
    robot_display.initDisplay(loadModel=True)

q = matlib.zeros(robot.nq).T

q[2] += 0.5
for i in range(4):
  q[7 + 2*i] = -0.8
  q[8 + 2*i] = 1.6

v = matlib.zeros(robot.nv).T

if display:
    robot_display.displayCollisions(False)
    robot_display.displayVisuals(True)
    robot_display.display(q)

assert [robot.model().existFrame(name) for name in contact_frames]

t = 0.0                         # time
invdyn = tsid.InverseDynamicsFormulationAccForce("tsid", robot, False)
invdyn.computeProblemData(t, q, v)
data = invdyn.data()

# Place the robot onto the ground.
id_contact = robot_display.model.getFrameId(contact_frames[0])
q[2] -= robot.framePosition(data, id_contact).translation[2, 0]
robot.computeAllTerms(data, q, v)

#sys.exit()

contacts = 4*[None]
for i, name in enumerate(contact_frames):
    AttachContact(name)

# COM task

comTask = tsid.TaskComEquality("task-com", robot)
comTask.setKp(kp_com * matlib.ones(3).T)
comTask.setKd(2.0 * np.sqrt(kp_com) * matlib.ones(3).T)
invdyn.addMotionTask(comTask, w_com, 1, 0.0)

#region posture task

postureTask = tsid.TaskJointPosture("task-posture", robot)
postureTask.setKp(kp_posture * matlib.ones(robot.nv-6).T)
postureTask.setKd(2.0 * np.sqrt(kp_posture) * matlib.ones(robot.nv-6).T)
invdyn.addMotionTask(postureTask, w_posture, 1, 0.0)

com_ref = robot.com(data)
trajCom = tsid.TrajectoryEuclidianConstant("traj_com", com_ref)
sampleCom = trajCom.computeNext()

q_ref = q[7:]
trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", q_ref)
samplePosture = trajPosture.computeNext()

solver = tsid.SolverHQuadProgFast("qp solver")
    
speed = 0.1
gait = Gait()

for i in range(0, N_SIMULATION):
    time_start = time.time()

    gait.Tick(t, speed, 0)

    SetDesiredCom(gait)
    SetDesiredJoints(gait)

    # solve

    HQPData = invdyn.computeProblemData(t, q, v)
    #if i == 0: HQPData.print_all()

    sol = solver.solve(HQPData)
    if(sol.status!=0):
        print ("[%d] QP problem could not be solved! Error code:"%(i), sol.status)
        break
    
    tau = invdyn.getActuatorForces(sol)
    dv = invdyn.getAccelerations(sol)

    # if i%PRINT_N == 0:
    #     print "Time %.3f"%(t)
    #     print "\tNormal forces: ",
    #     for contact in contacts:
    #         if invdyn.checkContact(contact.name, sol):
    #             f = invdyn.getContactForce(contact.name, sol)
    #             print "%4.1f"%(contact.getNormalForce(f)),

    #     print "\n\ttracking err %s: %.3f"%(comTask.name.ljust(20,'.'),       norm(comTask.position_error, 2))
    #     print "\t||v||: %.3f\t ||dv||: %.3f"%(norm(v, 2), norm(dv))

    v_mean = v + 0.5*dt*dv
    v += dt*dv
    q = se3.integrate(robot.model(), q, dt*v_mean)
    t += dt
    
    if display and i%DISPLAY_N == 0: robot_display.display(q)

    time_spent = time.time() - time_start
    if(time_spent < dt): time.sleep(dt-time_spent)

#  0 - right/left
#  1 - front/back
#  2 - down/up
#  ...
#  7 - LBB
#  8 - LBK
#  9 - RBB
# 10 - RBK
# 11 - LFB
# 12 - LFK
# 13 - RFB
# 14 - RFK