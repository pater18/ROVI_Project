import sdurw
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

    # get grasp fraPoint to Point interpolator, you should select at least 6 pme in robot tool frame
    targetAt = frameBaseTGoal * frameTcpTRobotTcp
    closedFormSovler = sdurw.ClosedFormIKSolverUR(robot.cptr(), state)
    return closedFormSovler.solve(targetAt, state)



if __name__ == "__main__":
    #Load workcell
    wc = sdurw.WorkCellLoaderFactory.load("./Project_WorkCell/Scene.wc.xml")
    if wc.isNull():
        raise Exception ("Could not load workcell")
    else :
        print ("Workcell is loaded")
    
    #Load objects
    bottleFrame = wc.findMovableFrame("Bottle")
    cylinderFrame = wc.findMovableFrame("Cylinder")
    squareFrame = wc.findMovableFrame("Square")
    if cylinderFrame == None or bottleFrame == None or squareFrame == None:
        raise Exception ("A frame could not be loaded") 
    else :
        print ("Objects on table loaded")

    #Load Robot
    RobotUR6 = wc.findSerialDevice("UR-6-85-5-A")
    if RobotUR6.isNull():
        raise Exception ("Could not find robot")
    else :
        print ("The robot is loaded")

    detector = sdurw.CollisionDetector(wc, sdurw_proximitystrategies.ProximityStrategyFactory_makeDefaultCollisionStrategy())
    state = wc.getDefaultState()
    collisionFreeSolutions = []

    for i in range (50):
        cylinderFrame.moveTo(sdurw.Transform3Dd(cylinderFrame.getTransform(state).P(), sdurw.RPYd(np.deg2rad(i),0,0)), state)
        solutions = getConfigurations("GraspTarget", "GraspTCP", RobotUR6, wc, state)

        for i in range(len(solutions)):
            # set the robot in that configuration and check if it is in collision
            RobotUR6.setQ(solutions[i], state)
            res1 = sdurw.ProximityData()
            if not detector.inCollision(state, res1):
                collisionFreeSolutions.append(solutions[i]) # save it
                break # we only need one


    print("Current position of the robot vs object to be grasped has: ", len(collisionFreeSolutions), " collision-free inverse kinematics solutions!")


    # visualize them
    tStatePath = sdurw.PathTimedState()
    time=0
    for i in range(len(collisionFreeSolutions)):
        RobotUR6.setQ(collisionFreeSolutions[i], state)
        tStatePath.push_back(sdurw.TimedState(time,state))
        time+=0.01

    tStatePath.save("./visu.rwplay", wc)