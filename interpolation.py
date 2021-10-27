import sdurw
import sdurwsim
import sdurw_kinematics
import sdurw_proximitystrategies
import numpy as np

def getConfigurations(nameGoal, nameTcp, robot, wc, state):
    # Get, make and print name of frames
    robotName = robot.getName()
    nameRobotBase = robotName + "." + "Base"
    nameRobotTcp = robotName + "." + "TCP"

    # Find frames and check for existence
    frameGoal = wc.findFrame(nameGoal)
    frameTcp = wc.findFrame(nameTcp)
    frameRobotBase = wc.findFrame(nameRobotBase)
    frameRobotTcp = wc.findFrame(nameRobotTcp)
    if frameGoal == None or frameTcp == None or frameRobotBase == None or frameRobotTcp == None:
        print(" ALL FRAMES NOT FOUND:")
        print(" Found \"", nameGoal, "\": ", "NO!" if frameGoal == None else "YES!")
        print(" Found \"", nameTcp, "\": ", "NO!" if frameTcp == None else "YES!")
        print(" Found \"", nameRobotBase, "\": ", "NO!" if frameRobotBase == None else "YES!")
        print(" Found \"", nameRobotTcp, "\": ", "NO!" if frameRobotTcp == None else "YES!")

    # Make "helper" transformations
    frameBaseTGoal = sdurw_kinematics.Kinematics_frameTframe(frameRobotBase, frameGoal, state)
    frameTcpTRobotTcp = sdurw_kinematics.Kinematics_frameTframe(frameTcp, frameRobotTcp, state)

    # get grasp frame in robot tool frame
    targetAt = frameBaseTGoal * frameTcpTRobotTcp
    closedFormSovler = sdurw.ClosedFormIKSolverUR(robot.cptr(), state)

    print(closedFormSovler.getTCP().getName())
    return closedFormSovler.solve(targetAt, state)



if __name__ == "__main__":
    #Load workcell
    wc = sdurw.WorkCellLoaderFactory.load("./Project_WorkCell/Scene.wc.xml")
    state = wc.getDefaultState()

    if wc.isNull():
        raise Exception ("Could not load workcell")
    else :
        print ("Workcell is loaded")
    
    

    #Load Robot
    RobotUR6 = wc.findSerialDevice("UR-6-85-5-A")
    if RobotUR6.isNull():
        raise Exception ("Could not find robot")
    else :
        print ("The robot is loaded")
    #Load gripper
    Gripper = wc.findTreeDevice("WSG50")
    if Gripper.isNull():
        raise Exception ("Could not find gripper")
    else:
        print("The gripper is loaded")
    Gripper.setQ(sdurw.Q(0.055), state)

    robotName = RobotUR6.getName()

    # Find frames and check for existence
    gripper = wc.findFrame("Gripper")

    controller = sdurwsim.SerialDeviceController(RobotUR6)


    q_start = RobotUR6.getQ(state)
    print(RobotUR6.getQ(state))

    solutions = getConfigurations("Bottle", "GraspTCP", RobotUR6, wc, state)

    collisionFreeSolutions = []
    detector = sdurw.CollisionDetector(wc, sdurw_proximitystrategies.ProximityStrategyFactory_makeDefaultCollisionStrategy())

    print(solutions.size())

    for i in range(len(solutions)):
        # set the robot in that configuration and check if it is in collision
        RobotUR6.setQ(solutions[i], state)
        res1 = sdurw.ProximityData()
        if not detector.inCollision(state, res1):
            collisionFreeSolutions.append(solutions[i]) # save it
            break # we only need one
    print(collisionFreeSolutions)

    if collisionFreeSolutions:
        interpolator = sdurw.LinearInterpolatorQ(q_start, collisionFreeSolutions[0], 3)
        
        tStatePath = sdurw.PathTimedState()

        for i in np.arange(0, 3, 0.1):
            RobotUR6.setQ(interpolator.x(i), state)
            tStatePath.push_back(sdurw.TimedState(i,state))

        tStatePath.save("visu.rwplay", wc)

        Gripper.setQ(sdurw.Q(0.03), state)
        tStatePath.push_back(sdurw.TimedState(3.1, state))
        print('Visualisationfile saved')
    else:
        print("No collisionfree solutions")