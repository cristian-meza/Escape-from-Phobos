#############################################################################################
# DON'T CHANGE ANY OF THE BELOW. NECESSARY FOR CORRECTLY JOINING THE COMPETITION SIMULATION #
#############################################################################################

import os, sys, time, datetime, traceback
import spaceteams as st
def custom_exception_handler(exctype, value, tb):
    error_message = "".join(traceback.format_exception(exctype, value, tb))
    st.logger_fatal(error_message)
sys.excepthook = custom_exception_handler
st.connect_to_sim(sys.argv)

import numpy as np
import numpy.typing as npt


this = st.GetThisSystem()

doSpawning = st.SimGlobals.GetSimEntity().GetParam(st.VarType.bool, "DoSpawning")
if doSpawning:
    while True:
        time.sleep(1.0 / this.GetParam(st.VarType.double, "LoopFreqHz"))

phobos: st.Entity = this.GetParam(st.VarType.entityRef, "Phobos")
phobosFrame = phobos.GetBodyFixedFrame()
shuttle: st.Entity = this.GetParam(st.VarType.entityRef, "Shuttle")
shuttleFrame = shuttle.GetBodyFixedFrame()
target: st.Entity = this.GetParam(st.VarType.entityRef, "Target")

topCamera: st.Entity = this.GetParam(st.VarType.entityRef, "TopCamera")
topCameraFrame = topCamera.GetBodyFixedFrame()
bottomCamera: st.Entity = this.GetParam(st.VarType.entityRef, "BottomCamera")
bottomCameraFrame = bottomCamera.GetBodyFixedFrame()


def GetNearestDebrisLocations(N: int) -> list[npt.NDArray]:
    allDebrisInfo: list[npt.NDArray] = shuttle.GetParamArray(st.VarType.doubleM3x3, 
                                                            ["Radar", "SortedDebrisData"])
    nearest = allDebrisInfo[:min(N, len(allDebrisInfo))]
    return nearest


def PanTiltCamera(camera: st.Entity, yaw_deg: float, pitch_deg: float) -> None:
    if camera == topCamera:
        yaw_deg = np.clip(yaw_deg, -89.0, 89.0)
        pitch_deg = np.clip(pitch_deg, -50.0, 0.0)
    elif camera == bottomCamera:
        yaw_deg = np.clip(yaw_deg, -89.0, 89.0)
        pitch_deg = np.clip(pitch_deg, 0.0, 50.0)
    camera.setRotation_DCM(st.math.Euler321_to_DCM([np.deg2rad(yaw_deg), np.deg2rad(pitch_deg), 0.0]), shuttleFrame)


def SetCommand(accelCommand: npt.NDArray, angAccelCommand: npt.NDArray) -> None:
    shuttle.SetParam(st.VarType.doubleV3, ["JetSelection", "CommandedAccel"], accelCommand)
    shuttle.SetParam(st.VarType.doubleV3, ["JetSelection", "CommandedAngAccel"], angAccelCommand)


#############################################################################################
# DON'T CHANGE ANY OF THE ABOVE. NECESSARY FOR CORRECTLY JOINING THE COMPETITION SIMULATION #
#############################################################################################


#########################################################
# EXAMPLE USER CODE (NOTE: RUNNING A LOOP IS NECESSARY) #
#########################################################


import copy
from PIL import Image
from pathlib import Path


"""
Get angular velocity from two quaternions representing the orientation of 
an object at time (t0) and (t0 + dt). Assumes scalar-last quaternions.
"""
def AngVelFromQuats(q1: npt.NDArray, q2: npt.NDArray, dt: float):
    return (2.0 / dt) * np.array([
        q1[3]*q2[0] - q1[0]*q2[3] - q1[1]*q2[2] + q1[2]*q2[1],
        q1[3]*q2[1] + q1[0]*q2[2] - q1[1]*q2[3] - q1[2]*q2[0],
        q1[3]*q2[2] - q1[0]*q2[1] + q1[1]*q2[0] - q1[2]*q2[3]])


###############################
# Get initial simulation time #
###############################

t0_sim: st.timestamp = st.SimGlobals.GetSimEntity().GetParam(st.VarType.timestamp, ["Time", "StartDateTAI"])
t0: datetime.datetime = t0_sim.as_datetime()

##################
# Start the loop #
##################

iteration = 0
imageNumber = 0
exit_flag: bool = False
while not exit_flag:

    ################################
    # Wait between loop iterations #
    ################################
    
    time.sleep(1.0 / this.GetParam(st.VarType.double, "LoopFreqHz"))

    ###################################
    # Get elapsed time from sim start #
    ###################################

    t_now: datetime.datetime = st.SimGlobals.SimClock.GetTimeNow().as_datetime()
    t_from_start: float = (t_now - t0).total_seconds()

    #####################################
    # Example for getting vehicle state #
    #####################################

    locVelAcc = shuttle.getLocVelAcc()
    loc = locVelAcc.loc_WRT_ExprIn(phobosFrame)
    vel = locVelAcc.vel_WRT_ExprIn(phobosFrame)
    rot = shuttle.getRotation().Quat_WRT(phobosFrame)
    angvel = shuttle.getAngVelocity().WRT_ExprIn(phobosFrame)

    #####################################################
    # Example for getting additional vehicle properties #
    #####################################################

    # mass = shuttle.getMass()
    # inertiaBodyFrame = shuttle.getInertiaTensor(shuttleFrame)
    # damage: float = shuttle.GetParam(st.VarType.double, ["Stats", "Damage"])
    # fuelUsed: float = shuttle.GetParam(st.VarType.double, ["Stats", "FuelUsed"])

    ##################################################################
    # Example for rotating the top camera 30 deg left and 10 deg up, #
    # and the bottom camera 20 deg right and 15 deg down.            #
    ##################################################################

    if iteration % 20 == 0:
        PanTiltCamera(topCamera, 30.0, -10.0)
        PanTiltCamera(bottomCamera, -20.0, 15.0)

    ####################################################################################
    # Example for obtaining an image from the top camera with FOV = 60 deg and EV = 12 #
    ####################################################################################

    if iteration % 20 == 0:  # Don't hot-loop the image captures
        cameraParams = st.CaptureImageProperties()
        cameraParams.nonphys_EV = 10.0
        cameraParams.FOV = 60.0
        cameraParams.ResolutionX = 512
        cameraParams.ResolutionY = 512
        image = st.camera.CaptureImageBlocking(topCamera, cameraParams)

        if image is not None:
            imageR = np.asarray(image.as_RGB8().PixelsR).reshape((cameraParams.ResolutionX, cameraParams.ResolutionY))
            imageG = np.asarray(image.as_RGB8().PixelsG).reshape((cameraParams.ResolutionX, cameraParams.ResolutionY))
            imageB = np.asarray(image.as_RGB8().PixelsB).reshape((cameraParams.ResolutionX, cameraParams.ResolutionY))
            imageRGB = Image.fromarray(np.dstack((imageR, imageG, imageB)).astype(np.uint8))

            # OPTIONAL: SAVE THE IMAGE TO DISK FOR VIEWING

            # savePath = os.path.join(st.path_utils.GetLocalOutputDir(), "STU4")
            # os.makedirs(savePath, exist_ok=True)
            # imageName = f"TestImage{str(imageNumber).zfill(4)}.png"
            # imageRGB.save(os.path.join(savePath, imageName))

            # st.OnScreenLogMessage(f"Saved camera capture to {os.path.join(savePath, imageName)}", "CameraCapture", st.Severity.Info)
            imageNumber += 1
        else:
            st.OnScreenLogMessage("Failed to capture camera image :(", "CameraNoCapture", st.Severity.Info)

    ###################################################################################
    # Example for getting the positions, closest points, range rates, and diameters   #
    # of the nearest 4 pieces of debris and printing the distance to the closest one. #
    ###################################################################################

    # debrisInfo = GetNearestDebrisLocations(4)
    # if len(debrisInfo) > 0:
    #     closestDebris = debrisInfo[0]
    #     closestDebris_loc = closestDebris[0, :]
    #     closestDebris_closestPoint = closestDebris[1, 0]
    #     closestDebris_rangeRate = closestDebris[1, 1]
    #     closestDebris_diameter = closestDebris[1, 2]
    #     st.OnScreenLogMessage(f"Nearest debris: {np.linalg.norm(closestDebris_loc):.1f} m", 
    #                           "UserAPIPrintout", st.Severity.Info)

    ####################################################
    # Example for commanding a beeline acceleration    #
    # toward the target (and facing it the entire way) #
    ####################################################

    # Gains
    accel_gain: float = 1.0
    angvel_gain: float = 0.2
    kp_pitch = 2.0
    kp_yaw = 2.0
    kp_roll = 2.0

    # Translational control
    targetState: st.frames.FramedLocVelAcc = target.getLocVelAcc()
    targetLoc: npt.NDArray = targetState.loc_WRT_ExprIn(phobosFrame)
    targetVel: npt.NDArray = targetState.vel_WRT_ExprIn(phobosFrame)

    goalDirection: npt.NDArray = (targetLoc - loc) / np.linalg.norm(targetLoc - loc)
    goalSpeed_m_s: float = 10.0  # just keep accelerating
    goalVelocity: npt.NDArray = goalSpeed_m_s * goalDirection

    linearAcceleration_m_s2 = accel_gain * (goalVelocity - vel)
    linearAcceleration_m_s2 = st.frames.FramedVector(linearAcceleration_m_s2, phobosFrame).ExprIn(shuttleFrame)

    # Rotational control
    # Align shuttle x-axis to target and shuttle y-axis to Phobos y-axis
    x_hat = copy.deepcopy(goalDirection)
    y_hat = np.array([0.0, -1.0, 0.0])
    z_hat = np.cross(x_hat, y_hat)
    z_hat = z_hat / np.linalg.norm(z_hat)
    T_shuttlePhobos = np.vstack((x_hat, y_hat, z_hat))
    q_shuttlePhobos = st.math.DCM_to_Quat(T_shuttlePhobos)

    desired_AngVel = np.zeros(3)
    if angvel_gain != 0.0:
        desired_AngVel = AngVelFromQuats(rot, q_shuttlePhobos, 1.0 / angvel_gain)
    desired_AngVel = st.frames.FramedAngVel(desired_AngVel, phobosFrame).WRT(phobosFrame).ExprIn(shuttleFrame)

    desired_AngVel_x = desired_AngVel[0]
    desired_AngVel_y = desired_AngVel[1]
    desired_AngVel_z = desired_AngVel[2]

    # get current angular velocity values
    current_AngVel = shuttle.getAngVelocity().WRT(phobosFrame).ExprIn(shuttleFrame)
    current_AngVel_x: float = current_AngVel[0]
    current_AngVel_y: float = current_AngVel[1]
    current_AngVel_z: float = current_AngVel[2]

    # calculate angular velocity error
    AngVel_error_x = desired_AngVel_x - current_AngVel_x
    AngVel_error_y = desired_AngVel_y - current_AngVel_y
    AngVel_error_z = desired_AngVel_z - current_AngVel_z

    strength_torque = shuttle.GetParam(st.VarType.double, ["Thrusters", "TorqueStrength_Nm"])
    torque_x = kp_roll * AngVel_error_x * strength_torque
    torque_y = kp_pitch * AngVel_error_y * strength_torque
    torque_z = kp_yaw * AngVel_error_z * strength_torque

    torque_Nm = np.array([torque_x, torque_y, torque_z])
    angularAcceleration_rad_s2 = np.linalg.inv(shuttle.getInertiaTensor(shuttleFrame)) @ torque_Nm

    # Send acceleration and angular acceleration commands
    SetCommand(linearAcceleration_m_s2, angularAcceleration_rad_s2)

    iteration += 1

st.leave_sim()
