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

###############################
# Get initial simulation time #
###############################

t0_sim: st.timestamp = st.SimGlobals.GetSimEntity().GetParam(st.VarType.timestamp, ["Time", "StartDateTAI"])
t0: datetime.datetime = t0_sim.as_datetime()

##################
# Start the loop #
##################

iteration = 0
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
    loc = locVelAcc.locvel().loc_WRT_ExprIn(phobosFrame)
    vel = locVelAcc.locvel().vel_WRT_ExprIn(phobosFrame)
    rot = shuttle.getRotation().Quat_WRT(phobosFrame)
    angvel = shuttle.getAngVelocity().WRT_ExprIn(phobosFrame)

    #####################################################
    # Example for getting additional vehicle properties #
    #####################################################

    mass = shuttle.getMass()
    inertiaBodyFrame = shuttle.getInertiaTensor(shuttleFrame)
    damage: float = shuttle.GetParam(st.VarType.double, ["Stats", "Damage"])
    fuelUsed: float = shuttle.GetParam(st.VarType.double, ["Stats", "FuelUsed"])

    ##################################################################
    # Example for rotating the top camera 30 deg left and 10 deg up, #
    # and the bottom camera 20 deg right and 15 deg down.            #
    ##################################################################

    if iteration % 20 == 0:
        PanTiltCamera(topCamera, 30.0, -10.0)
        PanTiltCamera(bottomCamera, -20.0, 15.0)

    ####################################################################################
    # Example for obtaining an image from the top camera with FOV = 60 deg and EV = 10 #
    ####################################################################################

    # cameraParams = st.CaptureImageProperties()
    # cameraParams.nonphys_EV = 10.0
    # cameraParams.FOV = 60.0
    # cameraParams.ResolutionX = 512
    # cameraParams.ResolutionY = 512
    # image = st.camera.CaptureImageBlocking(topCamera, cameraParams)

    ############################################################################
    # Example for getting the positions, range rates, and diameters of the     #
    # nearest 4 pieces of debris and printing the distance to the closest one. #
    ############################################################################

    debrisInfo = GetNearestDebrisLocations(4)
    if len(debrisInfo) > 0:
        closestDebris = debrisInfo[0]
        closestDebris_loc = closestDebris[0, :]
        closestDebris_closestPoint = closestDebris[1, 0]
        closestDebris_rangeRate = closestDebris[1, 1]
        closestDebris_diameter = closestDebris[1, 2]
        st.OnScreenLogMessage(f"Nearest debris: {np.linalg.norm(closestDebris_loc):.1f} m", 
                              "UserAPIPrintout", st.Severity.Info)

    #######################################################
    # Example for commanding a...                         #
    #   1 m/s^2 linear acceleration along the body-Z axis #
    #   0 rad/s^2 angular acceleration on all axes        #
    #######################################################

    linearAcceleration_m_s2 = np.array([0.0, 0.0, 1.0])
    angularAcceleration_rad_s2 = np.array([0.0, 0.0, 0.0])
    SetCommand(linearAcceleration_m_s2, angularAcceleration_rad_s2)

    iteration += 1

st.leave_sim()
