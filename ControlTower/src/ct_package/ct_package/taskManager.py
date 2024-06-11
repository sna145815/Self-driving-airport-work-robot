import rclpy
from rclpy.node import Node

# RobotControl Node import
from ct_package.drobot_control import RobotControl, RobotStatus, OrderStatus
from interface_package.srv import NodeNum

# A-star class import
from ct_package.A_Star import aStar

class robotTaskManager(RobotControl):
    def __init__(self):
        super().__init__()

        self.mazeInit()

        self.aStar = aStar()

        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.robotStatusCheck_timer_callback)

        # service server define
        self.robotArrivalSigServer1 = self.create_service(NodeNum, 'robotArrival_1', self.robotArrival_callback_service_1)
        self.robotArrivalSigServer2 = self.create_service(NodeNum, 'robotArrival_2', self.robotArrival_callback_service_2)
        self.robotArrivalSigServer3 = self.create_service(NodeNum, 'robotArrival_3', self.robotArrival_callback_service_3)

        self.shortGoalClient1 = self.create_client(NodeNum,'shortGoal_1')
        self.shortGoalClient2 = self.create_client(NodeNum,'shortGoal_2')
        self.shortGoalClient3 = self.create_client(NodeNum,'shortGoal_3')
        self.reqShortGoal1 = NodeNum.Request()
        self.reqShortGoal2 = NodeNum.Request()
        self.reqShortGoal3 = NodeNum.Request()
            

        # 최종 목적지로 이동중이면 1로 set, R1, K11, S11 등에 도착한 상태이면 0으로 clear
        # self.robot_1.movingFlg

        self.cnt = 0

        self.goalNode = {
            "S11" : 0,
            "S12" : 0,
            "S21" : 0,
            "S22" : 0,
            "K11" : 0,
            "K12" : 0,
            "K21" : 0,
            "K22" : 0,
            "R1"  : 1,
            "R2"  : 1,
            "R3"  : 1,
        }

        # self.priority = []
        # self.priority[0] = None # 우선순위 0  ex) "R-1"
        # self.priority[1] = None # 우선순위 1
        # self.priority[2] = None # 우선순위 2

        self.priority = None # "R-1", "R-2", "R-3" 우선순위 점유한 로봇이 무엇인지 확인하는 변수


        self.printCnt = 0

    def mazeInit(self):
        # 5x9
        # self.maze = [[99, 0, 99, 99, 0, 99, 99, 0, 99],
        #              [0, 0, 0, 0, 0, 0, 0, 0, 0],
        #              [99, 0, 99, 0, 99, 0, 99, 0, 99],
        #              [0, 0, 0, 0, 0, 0, 0, 0, 0],
        #              [99, 0, 99, 0, 99, 0, 99, 0, 99]]
        
        self.maze = [[1, 0, 1, 1, 0, 1, 1, 0, 1],
                     [0, 0, 0, 0, 0, 0, 0, 0, 0],
                     [1, 0, 1, 0, 1, 0, 1, 0, 1],
                     [0, 0, 0, 0, 0, 0, 0, 0, 0],
                     [1, 0, 1, 0, 1, 0, 1, 0, 1]]
        
        
        self.xyAlias = {
            "S11":(4,7),
            "S12":(4,5),
            "S21":(4,3),
            "S22":(4,1),
            "K11":(1,8),
            "K12":(3,8),
            "K21":(3,0),
            "K22":(1,0),
            "R-1":(0,7),
            "R-2":(0,4),
            "R-3":(0,1),
        }

    def checkNode(self, goal):
        x = self.xyAlias[goal][0]
        y = self.xyAlias[goal][1]
        return self.maze[x][y]


    def updateMazeSet(self, xy):
        x = xy[0]
        y = xy[1]
        self.maze[x][y] = 1


    def updateMazeClear(self, xy):
        x = xy[0]
        y = xy[1]
        self.maze[x][y] = 0


    def mazeSet(self, pathList):
        for i in range(len(pathList)):
            x = pathList[i][0]
            y = pathList[i][1]
            self.maze[x][y] = 1
        
    def mazeClear(self, XY):
        print(XY)
        x = XY[0]
        y = XY[1]
        print("x :", x)
        print("y :", y)
        print("maze xy : ", self.maze[x][y])
        self.maze[x][y] = 0


    def robotStatusCheck_timer_callback(self):
        if self.priority is None:
            for robot in self.robots:
                robotId = robot.robot_id # R-1, R-2, R-3

                if self.printCnt == 10:
                    self.printCnt = 0
                    # print("robot is_active : ", robot.is_active)
                    self.get_logger().info(f"{robotId} is active : {robot.is_active}")
                    self.get_logger().info(f"{robotId} current_status : {robot.current_status}")
                    self.get_logger().info(f"{robotId} order_status : {robot.current_order_status}")
                else:
                    self.printCnt = self.printCnt + 1

                if robot.is_active:
                    if robot.movingFlg == 0:    # 목적지 도착상태 (K11, S11, R-1 등등)
                        # 여기서 로봇의 path 점유 우선순위를 결정하자
                        # 필요 없을수도
                        self.priority = robotId
                        self.assignGoal(robot)

                        break
                    else:   # 목적지로 이동중(아직 waypoint)
                        self.get_logger().info(f"{robotId} : Still being delivered")
        else:
            self.get_logger().info(f"우선순위 이미 있음")
            # 우선순위 있는 녀석을 먼저 경로 계산시켜
            if self.priority == "R-1":
                robot = self.robot_1
            elif self.priority == "R-2":
                robot = self.robot_2
            elif self.priority == "R-3":
                robot = self.robot_3
            else:
                pass

            print("robot id : ",robot.robot_id)

            if robot.is_active:
                if robot.movingFlg == 0:    # 목적지 도착상태 (K11, S11, R-1 등등)
                    if robot.assignedFlg == 0:
                        # 여기서 로봇의 path 점유 우선순위를 결정하자
                        # 필요 없을수도
                        self.priority = robot.robot_id
                        robot.assignedFlg = 1
                        self.assignGoal(robot)
                    else:
                        print("assignedFlg 11111")
                else:   # 목적지로 이동중(아직 waypoint)
                    self.get_logger().info(f"{robot.robot_id} : Still being delivered")
        
        print("----------------------------------------------")





        # for robot in self.robots:
        #     # print("robot is_active : ", robot.is_active)
        #     robotId = robot.robot_id # R-1, R-2, R-3
        #     self.get_logger().info(f"{robotId} is active : {robot.is_active}")
        #     self.get_logger().info(f"{robotId} current_status : {robot.current_status}")
        #     self.get_logger().info(f"{robotId} order_status : {robot.current_order_status}")

        #     if robot.is_active:
        #         if robot.movingFlg == 0:    # 목적지 도착상태 (K11, S11, R-1 등등)
        #             # 여기서 로봇의 path 점유 우선순위를 결정하자
        #             # 필요 없을수도
        #             self.priority = robotId
        #             break
        #         else:   # 목적지로 이동중(아직 waypoint)
        #             self.get_logger().info(f"{robotId} : Still being delivered")
        #     print("----------------------------------------------")
        
        # self.assignGoal(robot)


    


    def assignGoal(self, robot):
        print("------------------assignGoal start----------------")
        robotId = robot.robot_id # R-1, R-2, R-3
        print("robot id : ", robotId)

        if robot.current_status == RobotStatus.HOME:                    # 집이냐(=배차주문 이냐)
            if robot.movingFlg == 0:
                robot.movingFlg = 1
                robot.lastEndPointXY = self.xyAlias[robotId]
                robot.startPointXY = robot.lastEndPointXY

                # 우선순위 밀려서 다시 시작한 경우는 nav_callback 하면 안되
                if robot.assignedFlg == 0:
                    robot.endPoint = self.nav_callback(0, robot.store_id)            # 세부목적지(K11, S11 등등) 설정 및 점유 상태 업데이트
                    robot.endPointXY = self.xyAlias[robot.endPoint]
                else:
                    pass

                self.get_logger().info(f"{robotId} start : {robot.startPointXY}")
                self.get_logger().info(f"{robotId} end : {robot.endPointXY}")
                # Astar
                robot.pathList = self.aStar.calculatePath(self.maze, robot.startPointXY, robot.endPointXY)
        
                if robot.pathList is not None:
                    self.priority = None
                    self.get_logger().info(f"pathList len : {len(robot.pathList)}")
                    self.get_logger().info(f"pathList : {robot.pathList[0]}")
                    # self.maze 에서 pathList 전부 1로 set
                    self.mazeSet(robot.pathList)
                    robot.step = robot.step + 1
                    
                    robot.midPointXY = robot.pathList[robot.step]
                    self.get_logger().info(f"{robotId} nextNode : {robot.midPointXY}")
                    self.shortGoal_serviceCall(robotId, robot.startPointXY, robot.midPointXY)
                else:
                    self.get_logger().info(f"{robotId} path is None")
                    robot.movingFlg = 0
                    robot.assignedFlg = 0
        elif (robot.current_status == RobotStatus.AT_STORE) :           # 배달이냐
            if robot.current_order_status == OrderStatus.DELIVERY_START:            # 카드 찍혔냐
                print("배달 시작")
                if robot.movingFlg == 0:
                    robot.movingFlg = 1
                    robot.lastEndPointXY = robot.endPointXY
                    print("robot.lastEndPointXY : ", robot.lastEndPointXY)
                    robot.startPointXY = robot.lastEndPointXY
                    # 이전 스토어 위치 점유 여부 클리어
                    # self.goalNode[robot.endPoint] = 0
                
                    robot.endPoint = self.nav_callback(1, robot.kiosk_id)            # 목적지 설정 및 점유 상태 업데이트
                    robot.endPointXY = self.xyAlias[robot.endPoint]
                    # Astar
                    robot.pathList = self.aStar.calculatePath(self.maze, robot.startPointXY, robot.endPointXY)
                
                    if robot.pathList is not None:
                        self.priority = None
                        self.get_logger().info(f"pathList len : {len(robot.pathList)}")
                        self.get_logger().info(f"pathList : {robot.pathList[0]}")
                        self.mazeSet(robot.pathList)

                        robot.midPointXY = robot.pathList[1]
                        self.shortGoal_serviceCall(robotId, robot.startPointXY, robot.midPointXY)
                else:
                    self.get_logger().info(f"{robotId} path is None")
                    robot.movingFlg = 0     
            else:
                self.get_logger().info(f"{robotId} : Waiting RFID tag")
                robot.assignedFlg = 0
        elif (robot.current_status == RobotStatus.AT_KIOSK) :           # 복귀냐
            if robot.current_order_status == OrderStatus.DELIVERY_FINISH:           # 카드찍혔냐
                if robot.movingFlg == 0:
                    robot.movingFlg = 1
                    robot.lastEndPointXY = robot.endPointXY
                    robot.startPointXY = robot.lastEndPointXY
                    # 이전 키오스크 위치 점유 여부 클리어
                    # self.goalNode[robot.endPoint] = 0
                    
                    robot.endPoint = self.nav_callback(2, robotId)                   # 목적지 설정 및 점유 상태 업데이트
                    robot.endPointXY = self.xyAlias[robot.endPoint]
                    # Astar
                    robot.pathList = self.aStar.calculatePath(self.maze, robot.startPointXY, robot.endPointXY)
                    
                    if robot.pathList is not None:
                        self.priority = None
                        self.get_logger().info(f"pathList len : {len(robot.pathList)}")
                        self.get_logger().info(f"pathList : {robot.pathList[0]}")
                        self.mazeSet(robot.pathList)

                        robot.midPointXY = robot.pathList[1]
                        self.shortGoal_serviceCall(robotId, robot.startPointXY, robot.midPointXY)
                        robot.returning()
                    else:
                        self.get_logger().info(f"{robotId} path is None")
                        robot.movingFlg = 0
                        
            else:
                self.get_logger().info(f"{robotId} : Waiting RFID tag")
                robot.assignedFlg = 0
        else:
            self.get_logger().info(f"{robotId} STATUS : {robot.current_status} and {robot.current_order_status}")
            robot.assignedFlg = 0


    
    def shortGoal_serviceCall(self, robotId, current, next):
        print("-------------------  service call start------------------")
        if robotId == "R-1":
            self.reqShortGoal1.current_x = current[0]
            self.reqShortGoal1.current_y = current[1]
            self.reqShortGoal1.next_x = next[0]
            self.reqShortGoal1.next_y = next[1]
            self.shortGoalClient1.call_async(self.reqShortGoal1)
            self.get_logger().info(f"{robotId} : request complete")
        elif robotId == "R-2":
            self.reqShortGoal2.current_x = current[0]
            self.reqShortGoal2.current_y = current[1]
            self.reqShortGoal2.next_x = next[0]
            self.reqShortGoal2.next_y = next[1]
            self.shortGoalClient2.call_async(self.reqShortGoal2)
            self.get_logger().info(f"{robotId} : request complete")
        elif robotId == "R-3":
            self.reqShortGoal3.current_x = current[0]
            self.reqShortGoal3.current_y = current[1]
            self.reqShortGoal3.next_x = next[0]
            self.reqShortGoal3.next_y = next[1]
            self.shortGoalClient3.call_async(self.reqShortGoal3)
            self.get_logger().info(f"{robotId} : request complete")
        else:
            self.get_logger().info(f"wrong Robot Id is entered...")
        
        print("-------------------  service call end------------------")


    # robot1이 노드이동을 완료했을때, req날라오면 실행되는함수
    def robotArrival_callback_service_1(self, request, response):
        try:
            print("------robotArrival_callback_service Start 1111111--------")
            self.get_logger().info(f"robot 111 current req.x : {request.current_x}")
            self.get_logger().info(f"robot 111 current req.y : {request.current_y}")
            self.robot_1.assignedFlg = 0

            # 이전 위치 점유 clear
            self.get_logger().info(f"robot 111 lastEndPointXY : {self.robot_1.lastEndPointXY}")
            self.mazeClear(self.robot_1.lastEndPointXY)

            self.robot_1.currentNode = (request.current_x, request.current_y)
            nodeType = 0    # 0: Invalid, 1: way, 2: 종착지

            # 지금 도착한 node가 종착지인지 확인
            for key, value in self.xyAlias.items():
                if self.robot_1.currentNode == value:
                    nodeType = 2
                    break
                else:
                    nodeType = 1

            if nodeType == 1:   # waypoint 도착
                self.get_logger().info(f"robot 111 way 도착")
                response.success = True

                self.robot_1.lastEndPointXY = self.robot_1.midPointXY
                # Astar
                # pathList = self.aStar.calculatePath(self.maze, self.robot_1.currentNode, self.robot_1.endPointXY)
                self.robot_1.step = self.robot_1.step + 1

                if self.robot_1.pathList[self.robot_1.step] is not None:
                    self.get_logger().info(f"pathList len : {self.robot_1.pathList[self.robot_1.step]}")
                    self.robot_1.midPointXY = self.robot_1.pathList[self.robot_1.step]
                    self.shortGoal_serviceCall(self.robot_1.robot_id, self.robot_1.currentNode, self.robot_1.midPointXY)
                else:
                    self.robot_1.wait = 1
            elif nodeType == 2: # 종착지 도착
                response.success = True
                self.robot_1.movingFlg = 0
                self.robot_1.pathList = None
                self.robot_1.step = 0
            else:
                response.success = False
                self.get_logger().info(f"R-1 : wrong node xy request!")

            return response
        except Exception as e:
            self.get_logger().info(f"robotArrival service1 Error : {e}")   


    # robot2이 노드이동을 완료했을때, req날라오면 실행되는함수
    def robotArrival_callback_service_2(self, request, response):
        try:
            print("------robotArrival_callback_service Start 2222222--------")
            self.get_logger().info(f"robot 222 current req.x : {request.current_x}")
            self.get_logger().info(f"robot 222 current req.y : {request.current_y}")
            self.robot_2.assignedFlg = 0

            # 이전 위치 점유 clear
            self.get_logger().info(f"robot 222 lastEndPointXY : {self.robot_2.lastEndPointXY}")
            self.mazeClear(self.robot_2.lastEndPointXY)

            self.robot_2.currentNode = (request.current_x, request.current_y)
            nodeType = 0    # 0: Invalid, 1: way, 2: 종착지

            # 지금 도착한 node가 종착지인지 확인
            for key, value in self.xyAlias.items():
                if self.robot_2.currentNode == value:
                    nodeType = 2
                    break
                else:
                    nodeType = 1

            if nodeType == 1:   # waypoint 도착
                self.get_logger().info(f"robot 111 way 도착")
                response.success = True

                self.robot_2.lastEndPointXY = self.robot_2.midPointXY
                # Astar
                # pathList = self.aStar.calculatePath(self.maze, self.robot_1.currentNode, self.robot_1.endPointXY)
                self.robot_2.step = self.robot_2.step + 1

                if self.robot_2.pathList[self.robot_2.step] is not None:
                    self.get_logger().info(f"pathList len : {self.robot_2.pathList[self.robot_2.step]}")
                    self.robot_2.midPointXY = self.robot_2.pathList[self.robot_2.step]
                    self.shortGoal_serviceCall(self.robot_2.robot_id, self.robot_2.currentNode, self.robot_2.midPointXY)
                else:
                    self.robot_2.wait = 1
            elif nodeType == 2: # 종착지 도착
                response.success = True
                self.robot_2.movingFlg = 0
                self.robot_2.pathList = None
                self.robot_2.step = 0
            else:
                response.success = False
                self.get_logger().info(f"R-1 : wrong node xy request!")

            return response
        except Exception as e:
            self.get_logger().info(f"robotArrival service1 Error : {e}")  

    
    # robot3이 노드이동을 완료했을때, req날라오면 실행되는함수
    def robotArrival_callback_service_3(self, request, response):
        try:
            print("------robotArrival_callback_service Start 3333333--------")
            self.get_logger().info(f"robot 333 current req.x : {request.current_x}")
            self.get_logger().info(f"robot 333 current req.y : {request.current_y}")

            self.robot_3.assignedFlg = 0

            # 이전 위치 점유 clear
            self.get_logger().info(f"robot 333 lastEndPointXY : {self.robot_3.lastEndPointXY}")
            self.mazeClear(self.robot_3.lastEndPointXY)

            self.robot_3.currentNode = (request.current_x, request.current_y)
            nodeType = 0    # 0: Invalid, 1: way, 2: 종착지

            # 지금 도착한 node가 종착지인지 확인
            for key, value in self.xyAlias.items():
                if self.robot_3.currentNode == value:
                    nodeType = 2
                    break
                else:
                    nodeType = 1

            if nodeType == 1:   # waypoint 도착
                self.get_logger().info(f"robot 333 way 도착")
                response.success = True

                self.robot_3.lastEndPointXY = self.robot_3.midPointXY
                # Astar
                # pathList = self.aStar.calculatePath(self.maze, self.robot_1.currentNode, self.robot_1.endPointXY)
                self.robot_3.step = self.robot_3.step + 1

                if self.robot_3.pathList[self.robot_3.step] is not None:
                    self.get_logger().info(f"pathList len : {self.robot_3.pathList[self.robot_3.step]}")
                    self.robot_3.midPointXY = self.robot_3.pathList[self.robot_3.step]
                    self.shortGoal_serviceCall(self.robot_3.robot_id, self.robot_3.currentNode, self.robot_3.midPointXY)
                    
                else:
                    self.robot_3.wait = 1
            elif nodeType == 2: # 종착지 도착
                self.get_logger().info(f"robot 333 종점 도착")
                self.priority = None
                response.success = True
                self.robot_3.movingFlg = 0
                self.robot_3.pathList = None
                self.robot_3.step = 0
            else:
                response.success = False
                self.get_logger().info(f"R-1 : wrong node xy request!")

            return response
        except Exception as e:
            self.get_logger().info(f"robotArrival service1 Error : {e}")  
    


    def nav_callback(self, cmd, endPointName):
        # 최종 목적지 도착할때 까지 계속 호출해야됨
        print("---------------nav callback start-------------------------")
        endGoal = None
        try:
            if cmd == 0: # 배차
                print("배차 상세 목적지 설정")
                print("cmd:",cmd,"endPointName : ", endPointName)
                if endPointName == "S-1":
                    if self.goalNode["S11"] == 0:
                        self.goalNode["S11"] = 1
                        endGoal = "S11"
                    elif self.goalNode["S12"] == 0:
                        self.goalNode["S12"] = 1
                        endGoal = "S12"
                    else:
                        self.get_logger().info(f"Store1 is busy...")
                    
                    self.get_logger().info(f"endGoal : {endGoal}")
                elif endPointName == "S-2":
                    if self.goalNode["S21"] == 0:
                        self.goalNode["S21"] = 1
                        endGoal = "S21"
                    elif self.goalNode["S22"] == 0:
                        self.goalNode["S22"] = 1
                        endGoal = "S22"
                    else:
                        self.get_logger().info(f"Store2 is busy...")
                    
                    self.get_logger().info(f"endGoal : {endGoal}")
                else:
                    self.get_logger().info(f"Invalid Store ID is entered...")
            elif cmd == 1: # 배달
                print("배달 상세 목적지 설정")
                print("cmd:",cmd,"endPointName : ", endPointName)
                if endPointName == "K-1":
                    if self.goalNode["K11"] == 0:
                        self.goalNode["K11"] = 1
                        endGoal = "K11"
                    elif self.goalNode["K12"] == 0:
                        self.goalNode["K12"] = 1
                        endGoal = "K12"
                    else:
                        self.get_logger().info(f"Kiosk1 is busy...")
                    
                    self.get_logger().info(f"endGoal : {endGoal}")
                elif endPointName == "K-2":
                    if self.goalNode["K21"] == 0:
                        self.goalNode["K21"] = 1
                        endGoal = "K21"
                    elif self.goalNode["K22"] == 0:
                        self.goalNode["K22"] = 1
                        endGoal = "K22"
                    else:
                        self.get_logger().info(f"Kiosk2 is busy...")
                    
                    self.get_logger().info(f"endGoal : {endGoal}")
                else:
                    self.get_logger().info(f"Invalid Kiosk ID is entered...")
            elif cmd == 2: # 복귀
                print("복귀 상세 목적지 설정")
                print("cmd:",cmd,"endPointName : ", endPointName)
                if endPointName == "R-1":
                    endGoal = "R-1"
                    self.get_logger().info(f"endGoal : {endGoal}")
                elif endPointName == "R-2":
                    endGoal = "R-2"
                    self.get_logger().info(f"endGoal : {endGoal}")
                elif endPointName == "R-3":
                    endGoal = "R-3"
                    self.get_logger().info(f"endGoal : {endGoal}")
                else:
                    self.get_logger().info(f"Invalid Robot ID is entered...")
            else:
                self.get_logger().info(f"command is invalid")

            print("---------------nav callback end-------------------------")

            return endGoal

        except Exception as e:
            self.get_logger().info(f"nav_callback Exception Error : {e}")
        except KeyboardInterrupt:
            self.get_logger().info(f"nav_callback keyboard terminate")
               


def main(args=None):
    rclpy.init(args=args)
    robotTaskManagerInst = robotTaskManager()

    try:
        rclpy.spin(robotTaskManagerInst)
    finally:
        robotTaskManagerInst.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()