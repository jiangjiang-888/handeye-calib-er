from robotcontrol import * 
import os.path 

def save_file(path,data):
    if str(path).startswith("~"):
        path = path.replace("~",str(os.getenv("HOME")))
    with open(path,'a') as wf:
        wf.write(str(data))
        wf.close()
        
def main():
    logger_init()
    Auboi5Robot.initialize()

    robot = Auboi5Robot()
    handle = robot.create_context()
    try:
        ip = '192.168.1.100'
        port = 8899
        result = robot.connect(ip, port)
        robot.enable_robot_event()
        if result != RobotErrorType.RobotError_SUCC:
            logger.info("connect server{0}:{1} failed.".format(ip, port))
        else:
            data = ""
            while True:
                command = "r"
                if command == "r":
                    r=robot.get_current_waypoint()
                    data += str(str(r['pos'])[:-1]+","+str(r['ori'])[1:])[1:-1]+"\n"
                    save_file("./data.txt",data)
                    print(data)
                    break
                if command == "q":
                    break;
                if command == "s":
                    break

            robot.disconnect()

    except RobotError as e:
        logger.error("robot Event:{0}".format(e))
    finally:
        if robot.connected:
            robot.disconnect()
        Auboi5Robot.uninitialize()

main()