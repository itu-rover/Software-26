#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as R
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Header
from moveit_commander.conversions import pose_to_list
from sensor_msgs.msg import JointState


def bokkol_IK(eeTform, enforceJointLimits=True, sortByDistance=False, referenceConfig=None):
    dhParams = np.array([
        [0, 1.5707963267949, -0.1055, 0],
        [0.45, 3.14159265358979, -1.45399011698933e-15, 0],
        [0.0874999999985376, -1.57079632679489, -3.04469451306471e-16, 0],
        [1.11022302462516e-16, 1.57079632679489, 0.441410000002068, 0],
        [-1.11022302462516e-15, -1.57079632679489, 1.2362716456105e-15, 0],
        [0, 0, 0.0823399999999999, 0]
    ], dtype=np.longdouble)


    thetaOffsets = np.array([0, -1.5707963267949, 3.5527136788005e-15, 0, 0, 0], dtype=np.longdouble)

    lastThreeAxesSign = np.array([1, -1, 1])
    jointLimits = np.array([
        [-1.8, 1.8],
        [-2.5, 2.5],
        [-2.2, 2.2],
        [-2, 2], # -3.14, 3.14
        [-2.7, 2.7],
        [-3.14, 3.14]
    ], dtype=np.longdouble)
    isJointRevolute = np.array([True, True, True, True, True, True])

    shiftedJointLimits = jointLimits + np.tile(thetaOffsets[:, np.newaxis], (1, 2))

    baseToWorldTform = np.array([
        [-3.49148133884313e-15, 1, 0, -0.010086064817756],
        [1, 3.49148133884313e-15, 0, 0.00273399008096952],
        [0, 0, -1, 0.0119693914735376],
        [0, 0, 0, 1]
    ], dtype=np.longdouble)
    actEEToDhEETform = np.eye(4)
    eePose = baseToWorldTform @ eeTform @ actEEToDhEETform



    if referenceConfig is None:
        referenceConfig = np.zeros(6)

    if not enforceJointLimits:
        shiftedJointLimits = np.tile([-np.inf, np.inf], (jointLimits.shape[0], 1))

    eeToJt5 = np.array([
        [1, 0, 0, 1.11022302462516e-16],
        [0, 3.39190141383284e-15, -1, 1.2362716456105e-15],
        [0, 1, 3.39190141383284e-15, -0.0823399999999999],
        [0, 0, 0, 1]
    ], dtype=np.longdouble)
    jt5Pose = eePose @ eeToJt5


    q123Opts = solveFirstThreeDHJoints(jt5Pose[:3, 3], dhParams)

    numRotationSolns = 2
    q456Opts = np.zeros((numRotationSolns * q123Opts.shape[0], q123Opts.shape[1]))

    eeFixedAlpha = dhParams[3, 1] + dhParams[4, 1] + dhParams[5, 1]
    eeFixedRotation = np.array([
        [1, 0, 0],
        [0, np.cos(eeFixedAlpha), -np.sin(eeFixedAlpha)],
        [0, np.sin(eeFixedAlpha), np.cos(eeFixedAlpha)]
    ], dtype=np.longdouble)
    for jtIdx in range(q123Opts.shape[0]):
        jt4ZeroPose = getJoint4PoseFromDH(q123Opts[jtIdx, :])

        jt4ToEERot = jt4ZeroPose[:3, :3].T @ eePose[:3, :3] @ eeFixedRotation.T

        orientationSolns = convertRotationToZYZAxesAngles(jt4ToEERot, lastThreeAxesSign, shiftedJointLimits[3:, :])
        q456Opts[jtIdx, :] = orientationSolns[0, :]
        q456Opts[jtIdx + q123Opts.shape[0], :] = orientationSolns[1, :]

        q123Opts[jtIdx, :] = q123Opts[jtIdx, :] - thetaOffsets[:3]
        q456Opts[jtIdx, :] = q456Opts[jtIdx, :] - thetaOffsets[3:]
        q456Opts[jtIdx + q123Opts.shape[0], :] = q456Opts[jtIdx + q123Opts.shape[0], :] - thetaOffsets[3:]

        if enforceJointLimits:
            q123Opts[jtIdx, :] = applyJointLimits(q123Opts[jtIdx, :], jointLimits[:3, :], isJointRevolute[:3])
            q456Opts[jtIdx, :] = applyJointLimits(q456Opts[jtIdx, :], jointLimits[3:, :], isJointRevolute[:3])
            q456Opts[jtIdx + q123Opts.shape[0], :] = applyJointLimits(q456Opts[jtIdx + q123Opts.shape[0], :], jointLimits[3:, :], isJointRevolute[:3])

    allSolnOpts = np.tile(q123Opts, (numRotationSolns, 1))
    allSolnOpts = np.hstack((allSolnOpts, q456Opts))
    isValidRowIdx = np.all(~np.isnan(allSolnOpts), axis=1)
    qOptsAllSolns = allSolnOpts[isValidRowIdx, :]

    qOptsWrappedAndRounded = np.round(wrapToPi(qOptsAllSolns) * 1.000000e6) / 1.000000e6

    _, isUniqueValidRowIdx = np.unique(qOptsWrappedAndRounded, axis=0, return_index=True)

    qOpts = qOptsAllSolns[np.sort(isUniqueValidRowIdx), :]
    if sortByDistance:
        qOpts = sortByEuclideanDistance(qOpts, referenceConfig, isJointRevolute)

    return qOpts

def sortByEuclideanDistance(solutions, referenceConfig, isJointRevolute):
    dist = np.linalg.norm(solutions - referenceConfig, axis=1)
    sortedIdx = np.argsort(dist)
    return solutions[sortedIdx, :]

def applyJointLimits(inputConfig, jointLimits, isJointRevolute):
    validConfig = inputConfig.copy()

    for i in range(len(inputConfig)):
        if jointLimits[i, 0] > inputConfig[i] or jointLimits[i, 1] < inputConfig[i]:
            wrappedJointValueOffset = wrapTo2Pi(inputConfig[i] - jointLimits[i, 0])

            if isEqualWithinTolerance(wrappedJointValueOffset, 2 * np.pi):
                wrappedJointValueOffset = 0

            jointRange = jointLimits[i, 1] - jointLimits[i, 0]

            if isJointRevolute[i] and (wrappedJointValueOffset < jointRange or isEqualWithinTolerance(wrappedJointValueOffset, jointRange)):
                wrappedJointValueOffset = min(wrappedJointValueOffset, jointRange)
                validConfig[i] = jointLimits[i, 0] + wrappedJointValueOffset
            else:
                return np.full_like(validConfig, np.nan)

    return validConfig

def convertRotationToZYZAxesAngles(tgtRotation, axesSign, jointLim):
    eulAngles = R.from_matrix(tgtRotation).as_euler('zyz', degrees=False)

    jointsInGimbalLock = np.array([0, 0, 0])

    if isEqualWithinTolerance(eulAngles[1], 0):
        newTgtRotation = tgtRotation.copy()
        newTgtRotation[:2, 2] = 0
        newTgtRotation[2, :2] = 0
        newTgtRotation[2, 2] = 1
        eulAngles = R.from_matrix(newTgtRotation).as_euler('zyz', degrees=False)
        variableJtIdx = [0, 2]
        jointsInGimbalLock[variableJtIdx] = [1, 1]
        totalRotation = np.sum(eulAngles[variableJtIdx] , dtype=np.longdouble)
        eulAngles[variableJtIdx] = distributeRotationOverJoints(totalRotation, axesSign[variableJtIdx], jointLim[variableJtIdx, :])

        actAngles = np.vstack((eulAngles, np.full(3, np.nan)))
    else:
        eulAltUnwrapped = eulAngles.copy()
        eulAltUnwrapped[1] = -eulAltUnwrapped[1]
        eulAltUnwrapped += np.pi
        eulAltUnwrapped[1] -= np.pi
        eulAnglesAlt = wrapToPi(eulAltUnwrapped)

        actAngles = np.vstack((eulAngles, eulAnglesAlt)) * axesSign

    return actAngles

def distributeRotationOverJoints(totalRotation, axesSigns, jointLim):
    N = jointLim.shape[0]
    jointAngles = np.zeros(N)

    jointLim = np.tile(axesSigns[:, np.newaxis], (1, 2)) * jointLim
    jointLim = np.sort(jointLim, axis=1)

    jointRange = jointLim[:, 1] - jointLim[:, 0]
    isRevJointFullRange = jointRange > 2 * np.pi

    if np.any(isRevJointFullRange):
        jointIdxVector = np.arange(N)
        jointsWithIncompleteRange = jointIdxVector[~isRevJointFullRange]
        for i in jointsWithIncompleteRange:
            jointAngles[i] = np.sum(jointLim[i, :]) / 2

        wrappedRemainder = wrapToPi(totalRotation - np.sum(jointAngles))
        jointsWithCompleteRange = jointIdxVector[isRevJointFullRange]
        for j in jointsWithCompleteRange:
            jointAngles[j] = wrappedRemainder / len(jointsWithCompleteRange)
    else:
        jointAngles = np.sum(jointLim, axis=1) / 2

        jointIdxVector = np.arange(N)[::-1]
        wrappedTotalRotation = wrapToPi(totalRotation)
        for jointIdx in jointIdxVector:
            diffRotation = wrapToPi(wrappedTotalRotation - np.sum(jointAngles))
            jointAngles[jointIdx] += np.sign(diffRotation) * min(abs(diffRotation), jointRange[jointIdx] / 2)

        if not isEqualWithinTolerance(wrapToPi(np.sum(jointAngles)), wrappedTotalRotation):
            return np.full(N, np.nan)

    return jointAngles * axesSigns

def solveFirstThreeDHJoints(jt5Pos, dhParams):
    a1, a2, a3 = dhParams[0, 0], dhParams[1, 0], dhParams[2, 0]
    alpha1, alpha2, alpha3 = dhParams[0, 1], dhParams[1, 1], dhParams[2, 1]
    d1, d2, d3, d4 = dhParams[0, 2], dhParams[1, 2], dhParams[2, 2], dhParams[3, 2]

    z3 = jt5Pos[2]
    R3 = jt5Pos[0]**2 + jt5Pos[1]**2 + (jt5Pos[2] - d1)**2
    z = z3 - d1

    hSolns, _, hasPiSoln = solveForHCaseZeroA1(R3, a2, a3, alpha2, alpha3, d2, d3, d4)
    possThetas = np.zeros((16,3))
    possThetas[:,2] = np.nan
    # possThetas = np.full((16, 3), np.nan)
    for hIdx, h3 in enumerate(hSolns):
        h3 = replaceImagWithNaN(h3)
        if np.isnan(h3):
            possThetas[hIdx,2] = np.nan
            possThetas[hIdx+4,2] = np.nan
        else:
            possThetas[hIdx, 2] = 2 * np.arctan2(h3, 1)
            possThetas[hIdx + 4, 2] = 2 * np.arctan2(-h3, -1)

    if hasPiSoln:
        possThetas[len(hSolns), 2] = np.pi

    for theta3Idx in range(8):
        theta3 = possThetas[theta3Idx, 2]
        if np.isnan(theta3):
            possThetas[theta3Idx, :] = np.nan
            continue

        f = computef13SupportingEquations(a3, alpha3, d3, d4, theta3)
        F = computeF14SupportingEquations(a1, a2, alpha1, alpha2, f[0], f[1], f[2], d2)

        theta2Opts = solveTrigEquations(-F[1] * np.sin(alpha1), F[0] * np.sin(alpha1), z - F[3])
        rhsConst = jt5Pos[2] - d1 - np.cos(alpha1) * (np.sin(alpha2) * f[1] + np.cos(alpha2) * f[2] + d2)
        lhsCosCoeff = -np.sin(alpha1) * (-np.cos(alpha2) * f[1] + np.sin(alpha2) * f[2])
        lhsSinCoeff = np.sin(alpha1) * (a2 + f[0])
        theta2Constraint = solveTrigEquations(lhsCosCoeff, lhsSinCoeff, rhsConst)

        theta2 = chooseCorrectSolution(theta2Opts, theta2Constraint, 1.000000e-6)

        for theta2Idx in range(2):
            solIdx = theta3Idx + 8 * theta2Idx
            possThetas[solIdx, 2] = theta3
            possThetas[solIdx, 1] = theta2[theta2Idx]

            if np.isnan(possThetas[solIdx, 1]):
                possThetas[solIdx, :2] = np.nan
                continue

            g = computeg12SupportingEquations(a1, a2, alpha1, alpha2, f[0], f[1], f[2], d2, theta2[theta2Idx])
            theta1Opts = solveTrigEquations(g[0], g[1], jt5Pos[0])
            theta1Constraint = solveTrigEquations(-g[1], g[0], jt5Pos[1])
            theta1Opts = chooseCorrectSolution(theta1Opts, theta1Constraint, 1.000000e-6)

            theta1 = theta1Opts[0]
            possThetas[solIdx, 0] = replaceImagWithNaN(theta1)

    #outputThetas = possThetas[~np.isnan(possThetas).any(axis=1), :]

    return possThetas

def computef13SupportingEquations(a3, alpha3, s3, s4, theta3):
    t2 = np.sin(alpha3, dtype=np.longdouble)
    t3 = np.cos(theta3, dtype=np.longdouble)
    t4 = np.sin(theta3, dtype=np.longdouble)

    f = np.array([
        a3 * t3 + s4 * t2 * t4,
        a3 * t4 - s4 * t2 * t3,
        s3 + s4 * np.cos(alpha3)
    ], dtype=np.longdouble)
    return f

def computeF14SupportingEquations(a1, a2, alpha1, alpha2, f1, f2, f3, s2):
    t2 = np.cos(alpha2, dtype=np.longdouble)
    t3 = np.sin(alpha2, dtype=np.longdouble)
    F = np.array([
        a2 + f1,
        -f2 * t2 + f3 * t3,
        s2 * (f3 * t2 + f2 * t3) * 2.0 + a2 * f1 * 2.0 + a1**2 + a2**2 + f1**2 + f2**2 + f3**2 + s2**2,
        np.cos(alpha1) * (s2 + f3 * t2 + f2 * t3)
    ], dtype=np.longdouble)
    return F

def computeg12SupportingEquations(a1, a2, alpha1, alpha2, f1, f2, f3, s2, theta2):
    t2 = np.cos(alpha1, dtype=np.longdouble)
    t3 = np.cos(alpha2, dtype=np.longdouble)
    t4 = np.sin(alpha2, dtype=np.longdouble)
    t5 = np.cos(theta2, dtype=np.longdouble)
    t6 = np.sin(theta2, dtype=np.longdouble)
    t7 = a2 + f1
    t8 = f2 * t3
    t9 = f3 * t4
    t10 = -t9
    t11 = t8 + t10

    g = np.array([
        a1 + t5 * t7 - t6 * t11,
        np.sin(alpha1) * (s2 + f2 * t4 + f3 * t3) - t2 * t6 * t7 - t2 * t5 * t11
    ], dtype=np.longdouble)
    return g

def solveTrigEquations(a, b, c):
    if isEqualWithinTolerance(a, 0) and isEqualWithinTolerance(b, 0) and isEqualWithinTolerance(c, 0):
        return np.array([0, np.nan], dtype=np.longdouble)
    elif isEqualWithinTolerance(a, 0) and isEqualWithinTolerance(b, 0) and not isEqualWithinTolerance(c, 0):
        return np.array([np.nan, np.nan], dtype=np.longdouble)

    d = np.sqrt(a**2 + b**2)
    cPrime = c / d
    if cPrime < 1 or isEqualWithinTolerance(cPrime, 1):
        cPrime = cPrime+0j
        phi1 = np.arctan2(a, b)
        phi2 = np.arctan2(-a, -b)
        theta = np.array([
            np.real(np.arcsin(cPrime)) - phi1,
            -np.real(np.arcsin(cPrime)) - phi2
        ], dtype=np.longdouble)
    return theta
    
    # Aytin bu satırı commentledi, matlab koduna göre bu returnun zaten asla çalışmaması lazım
    #else:
    #   return np.array([np.nan, np.nan])

def chooseCorrectSolution(solutionPair1, solutionPair2, solTolerance):
    realSolutionPair1 = np.array([wrapToPi(s) for s in solutionPair1], dtype=np.longdouble)
    realSolutionPair2 = np.array([wrapToPi(s) for s in solutionPair2], dtype=np.longdouble)

    correctSolution = np.full(2, np.nan)
    for i in range(2):
        for j in range(2):
            if np.abs(wrapToPi(realSolutionPair1[i] - realSolutionPair2[j])) < solTolerance:
                correctSolution[i] = realSolutionPair1[i]

    return np.sort(correctSolution)

def replaceImagWithNaN(qToCheck):
    if not qToCheck:
        qToCheck = np.nan
    elif not isEqualWithinTolerance(np.imag(qToCheck), 0):
        qToCheck = np.nan
    elif np.iscomplex(qToCheck):
        qToCheck = np.real(qToCheck)
        
    return qToCheck

def solveForHCaseZeroA1(R3, a2, a3, alpha2, alpha3, d2, d3, d4):
    A = a2**2 - 2 * a2 * a3 - R3 + a3**2 + d2**2 + d3**2 + d4**2 + 2 * d2 * d3 * np.cos(alpha2) + 2 * d3 * d4 * np.cos(alpha3) + 2 * d2 * d4 * np.cos(alpha2) * np.cos(alpha3) + 2 * d2 * d4 * np.sin(alpha2) * np.sin(alpha3)
    B = 4 * a3 * d2 * np.sin(alpha2) + 4 * a2 * d4 * np.sin(alpha3)
    C = 2 * a2 * a3 - R3 + a2**2 + a3**2 + d2**2 + d3**2 + d4**2 + 2 * d2 * d3 * np.cos(alpha2) + 2 * d3 * d4 * np.cos(alpha3) + 2 * d2 * d4 * np.cos(alpha2) * np.cos(alpha3) - 2 * d2 * d4 * np.sin(alpha2) * np.sin(alpha3)

    if isEqualWithinTolerance(A, 0) and isEqualWithinTolerance(B, 0) and isEqualWithinTolerance(C, 0):
        hasFiniteNumSol = False
        hSolns = np.array([], dtype=np.longdouble)
    elif isEqualWithinTolerance(A, 0):
        hSolns = np.array([-C / B], dtype=np.longdouble)
        hasFiniteNumSol = True
    else:
        discriminant = B**2 - 4 * A * C
        if discriminant < 0:
            h1 = (-B - np.sqrt(complex(discriminant))) / (2 * A)
            h2 = (-B + np.sqrt(complex(discriminant))) / (2 * A)
        else:
            h1 = (-B - np.sqrt(discriminant)) / (2 * A)
            h2 = (-B + np.sqrt(discriminant)) / (2 * A)
        hSolns = np.array([h1, h2], dtype=np.longdouble)
        hasFiniteNumSol = True

    if hasFiniteNumSol:
        localA1 = 0
        localTheta = np.pi
        fTerms = computef13SupportingEquations(a3, alpha3, d3, d4, localTheta)
        dummyAlpha1 = 0
        FTerms = computeF14SupportingEquations(localA1, a2, dummyAlpha1, alpha2, fTerms[0], fTerms[1], fTerms[2], d2)
        hasPiSoln =isEqualWithinTolerance(FTerms[2], R3)
    else:
        hasPiSoln = True

    return hSolns, hasFiniteNumSol, hasPiSoln

def getJoint4PoseFromDH(q123):
    dhParams = np.array([
        [0, 1.5707963267949, -0.1055, 0],
        [0.45, 3.14159265358979, -1.45399011698933e-15, 0],
        [0.0874999999985376, -1.57079632679489, -3.04469451306471e-16, 0],
        [1.11022302462516e-16, 1.57079632679489, 0.441410000002068, 0],
        [-1.11022302462516e-15, -1.57079632679489, 1.2362716456105e-15, 0],
        [0, 0, 0.0823399999999999, 0]
    ], dtype=np.longdouble)

    jt4Pose = np.eye(4, dtype=np.longdouble)
    for i in range(3):
        a = dhParams[i, 0]
        alpha = dhParams[i, 1]
        d = dhParams[i, 2]
        theta = q123[i]

        Ttheta = np.array([
            [np.cos(theta), -np.sin(theta), 0, 0],
            [np.sin(theta), np.cos(theta), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.longdouble)
        TFixed = np.array([
            [1, 0, 0, a],
            [0, np.cos(alpha), -np.sin(alpha), 0],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ], dtype=np.longdouble)

        jt4Pose = jt4Pose @ Ttheta @ TFixed

    return jt4Pose

def isEqualWithinTolerance(mat1, mat2, tol=1.0e-6):

    diff_mat = np.abs(mat1 - mat2)
    return np.all(diff_mat < tol)

def wrapToPi(angles):
    return (angles + np.pi) % (2 * np.pi) - np.pi

def wrapTo2Pi(angles):
    return angles % (2 * np.pi)


class bokkol():
    def __init__(self):
        rospy.init_node('my_order', anonymous=True)
        self.positions_publisher = rospy.Publisher('/manipulator_controller/command', JointTrajectory, queue_size=10)
        self.current_state = np.array([0,0,0,0,0,0], dtype=np.longdouble)
        get_current_state = rospy.Subscriber('/joint_states', JointState, self.callback)

        target_xyz = np.array([0,0,0], dtype=np.longdouble)

        while 1:
            print("Enter the x-y-z value: ")
            target_xyz[0] = input()
            target_xyz[1] = input()
            target_xyz[2] = input()

            self.go_to_target(target_xyz)

            print("Hedefe ulaşıldı!")

    def callback(self, data):
        for i in range(6):
            self.current_state[i] = data.position[i]


    def go_to_target(self, target_xyz):
        # Define target position and orientation
        target_position = target_xyz # [0.5, -0.3, 0.7]  # x, y, z position
        target_orientation = [0, 0, np.pi/2]  # roll, pitch, yaw (in radians)

        position_transform = np.eye(4)  # Initialize a 4x4 identity matrix
        position_transform[:3, 3] = target_position  # Set the translation vector

        # Convert Euler angles to rotation matrix
        rotation = R.from_euler('ZYX', target_orientation, degrees=False).as_matrix()  

        orientation_transform = np.eye(4)  # Initialize a 4x4 identity matrix
        orientation_transform[:3, :3] = rotation  # Set the rotation part
        

        target_transform = position_transform @ orientation_transform
        q_sol = bokkol_IK(target_transform)
        print("q_sol: ", q_sol)
        print(" --------------------------- \n\n")

        solution = np.array([0,0,0,0,0,0], dtype=np.longdouble)
        for i in range(6):
            solution[i] = q_sol[0, i]

        # 4. ve 6. eksen sonuçları yer değiştirilir
        temp_4_6 = solution[3]
        solution[3] = solution[5]
        solution[5] = temp_4_6
        
        solution[0] = solution[0] * (-1)
        
        solution[1] = solution[1] - 1.78
        solution[2] = solution[2] - (np.pi/2)
        solution[4] = solution[4] * (-1)
        #solution[4] = solution[4] + (np.pi/2)
        solution[5] = -np.pi/2
    

        print("solution: ", solution)


        # 2. eksen: (çıkan sonuç) - 94 
        # 3. eksen: (çıkan sonuç) - 90
        # 4. eksen: limitler artırma
        # 5. eksen: -1 * (çıkan sonuç)
        # 6. eksen: önemsiz
        # rpy: pitch: 90, yaw: 0, roll: önemsiz

        rate = rospy.Rate(10)
        trajectory_msg = JointTrajectory()
        trajectory_msg.header = Header()
        trajectory_msg.header.frame_id = 'arm_base' 
        trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        point = JointTrajectoryPoint()
        point.positions = solution
        point.time_from_start = rospy.Duration(2)
        trajectory_msg.points.append(point)

        while not rospy.is_shutdown():

            trajectory_msg.header.stamp = rospy.Time.now()
            self.positions_publisher.publish(trajectory_msg)
            #rospy.loginfo("position updated")

            if (abs(self.current_state - solution) < 0.001).all():
                break   

            rate.sleep()



if __name__ == '__main__':
    try:
        bokkol()
    except rospy.ROSInterruptException:
        pass 




    
    



    
