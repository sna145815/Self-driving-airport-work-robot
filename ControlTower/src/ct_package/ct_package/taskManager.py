import sys
import os

import rclpy
from rclpy.node import Node

from ct_package.pathDict import callPathDict
from ct_package.pathDict import deliPathDict
from ct_package.pathDict import returnPathDict
from ct_package.pathDict import endPoint_Dict


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

    def mazeInit(self):
        # 5x9
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
            "K21":(1,0),
            "K22":(3,0),
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



    def robotStatusCheck_timer_callback(self):
        for robot in self.robots:
            # print("robot is_active : ", robot.is_active)
            robotId = robot.robot_id # R-1, R-2, R-3
            self.get_logger().info(f"{robotId} is active : {robot.is_active}")
            self.get_logger().info(f"{robotId} current_status : {robot.current_status}")
            self.get_logger().info(f"{robotId} order_status : {robot.current_order_status}")

            if robot.is_active:
                if robot.movingFlg == 0:
                    if robot.current_status == RobotStatus.HOME:                    # 집이냐(=배차주문 이냐)
                        robot.movingFlg = 1
                        robot.lastEndPoint = self.xyAlias[robotId]

                        robot.startPoint = robot.lastEndPoint
                        robot.endPoint = self.nav_callback(0, robot.store_id)            # 세부목적지(K11, S11 등등) 설정 및 점유 상태 업데이트
                        # Astar
                        pathList = self.aStar.calculatePath(self.maze, robot.startPoint, robot.endPoint)
                        if pathList is not None:
                            nextNode = pathList[1]
                            self.shortGoal_serviceCall(robotId, robot.startPoint, nextNode)
                            self.updateMazeSet(nextNode)
                            self.updateMazeClear(robot.startPoint)
                        else:
                            robot.movingFlg = 0
                    elif (robot.current_status == RobotStatus.AT_STORE) :           # 배달이냐
                        if robot.current_order_status == OrderStatus.DELIVERY_START:            # 카드 찍혔냐
                            robot.movingFlg = 1
                            robot.lastEndPoint = robot.endPoint

                            robot.startPoint = robot.lastEndPoint
                            robot.endPoint = self.nav_callback(1, robot.kiosk_id)            # 목적지 설정 및 점유 상태 업데이트
                            # Astar
                            pathList = self.aStar.calculatePath(self.maze, robot.startPoint, robot.endPoint)
                            if pathList is not None:
                                nextNode = pathList[1]
                                self.shortGoal_serviceCall(robotId, robot.startPoint, nextNode)
                                self.updateMazeSet(nextNode)
                                self.updateMazeClear(robot.startPoint)
                            else:
                                robot.movingFlg = 0
                        else:
                            self.get_logger().info(f"{robotId} : Waiting RFID tag")
                    elif (robot.current_status == RobotStatus.AT_KIOSK) :           # 복귀냐
                        if robot.current_order_status == OrderStatus.DELIVERY_FINISH:           # 카드찍혔냐
                            robot.movingFlg = 1
                            robot.lastEndPoint = robot.endPoint

                            robot.startPoint = robot.lastEndPoint
                            robot.endPoint = self.nav_callback(2, robotId)                   # 목적지 설정 및 점유 상태 업데이트
                            # Astar
                            pathList = self.aStar.calculatePath(self.maze, robot.startPoint, robot.endPoint)
                            if pathList is not None:
                                nextNode = pathList[1]
                                self.shortGoal_serviceCall(robotId, robot.startPoint, nextNode)
                                self.updateMazeSet(nextNode)
                                self.updateMazeClear(robot.startPoint)
                                robot.returning()
                            else:
                                robot.movingFlg = 0
                        else:
                            self.get_logger().info(f"{robotId} : Waiting RFID tag")
                    else:
                        self.get_logger().info(f"{robotId} STATUS : {robot.current_status} and {robot.current_order_status}")
                else:
                    # waypoint간 이동 상태일때는, 서비스 콜백 함수에서 처리함
                    self.cnt = self.cnt + 1
                    if robot.wait == 1:
                        if self.cnt > 6:
                            self.cnt = 0
                            # astar ㄱㄱ
                            pathList = self.aStar.calculatePath(self.maze, robot.currentNode, self.robot_1.endPoint)
                            if pathList is not None:
                                robot.wait = 0
                                nextNode = pathList[1]
                                self.updateMazeClear(self.robot_1.currentNode)
                                self.updateMazeSet(nextNode)
                                self.shortGoal_serviceCall(self.robot_1.robot_id, self.robot_1.currentNode, nextNode)
                            else:
                                self.get_logger().info(f"{robotId} : path findig...")
                        else:
                            pass
                    else:
                        self.cnt = 0
                        self.get_logger().info(f"{robotId} : Still being delivered")
        print("----------------------------------------------")

    
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
            self.robot_1.currentNode[0] = request.current_x
            self.robot_1.currentNode[1] = request.current_y
            nodeType = 0    # 0: Invalid, 1: way, 2: 종착지

            # 지금 도착한 node가 종착지인지 확인
            for key, value in self.xyAlias.items():
                if self.robot_1.currentNode == value:
                    nodeType = 2
                    break
                else:
                    nodeType = 1

            if nodeType == 1:   # waypoint 도착
                response.success = True
                # Astar
                pathList = self.aStar.calculatePath(self.maze, self.robot_1.currentNode, self.robot_1.endPoint)
                if pathList is not None:
                    nextNode = pathList[1]
                    self.shortGoal_serviceCall(self.robot_1.robot_id, self.robot_1.currentNode, nextNode)
                    self.updateMazeSet(nextNode)
                    self.updateMazeClear(self.robot_1.currentNode)
                else:
                    self.robot_1.wait = 1

            elif nodeType == 2: # 종착지 도착
                response.success = True
                self.robot_1.movingFlg = 0
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
            self.robot_2.currentNode[0] = request.current_x
            self.robot_2.currentNode[1] = request.current_y
            nodeType = 0    # 0: Invalid, 1: way, 2: 종착지

            # 지금 도착한 node가 종착지인지 확인
            for key, value in self.xyAlias.items():
                if self.robot_2.currentNode == value:
                    nodeType = 2
                    break
                else:
                    nodeType = 1

            if nodeType == 1:   # waypoint 도착
                response.success = True
                # Astar
                pathList = self.aStar.calculatePath(self.maze, self.robot_2.currentNode, self.robot_2.endPoint)
                if pathList is not None:
                    nextNode = pathList[1]
                    self.shortGoal_serviceCall(self.robot_2.robot_id, self.robot_2.currentNode, nextNode)
                    self.updateMazeSet(nextNode)
                    self.updateMazeClear(self.robot_2.currentNode)
                else:
                    self.robot_2.wait = 1

            elif nodeType == 2: # 종착지 도착
                response.success = True
                self.robot_2.movingFlg = 0
            else:
                response.success = False
                self.get_logger().info(f"R-2 : wrong node xy request!")

            return response
        except Exception as e:
            self.get_logger().info(f"robotArrival service1 Error : {e}")

    
    # robot3이 노드이동을 완료했을때, req날라오면 실행되는함수
    def robotArrival_callback_service_3(self, request, response):
        try:
            print("------robotArrival_callback_service Start 3333333--------")
            self.robot_3.currentNode[0] = request.current_x
            self.robot_3.currentNode[1] = request.current_y
            nodeType = 0    # 0: Invalid, 1: way, 2: 종착지

            # 지금 도착한 node가 종착지인지 확인
            for key, value in self.xyAlias.items():
                if self.robot_3.currentNode == value:
                    nodeType = 2
                    break
                else:
                    nodeType = 1

            if nodeType == 1:   # waypoint 도착
                response.success = True
                # Astar
                pathList = self.aStar.calculatePath(self.maze, self.robot_3.currentNode, self.robot_3.endPoint)
                if pathList is not None:
                    nextNode = pathList[1]
                    self.shortGoal_serviceCall(self.robot_3.robot_id, self.robot_3.currentNode, nextNode)
                    self.updateMazeSet(nextNode)
                    self.updateMazeClear(self.robot_3.currentNode)
                else:
                    self.robot_3.wait = 1

            elif nodeType == 2: # 종착지 도착
                response.success = True
                self.robot_3.movingFlg = 0
            else:
                response.success = False
                self.get_logger().info(f"R-3 : wrong node xy request!")

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
                    if self.checkNode["S11"] == 0:
                        self.updateMazeSet(self.xyAlias["S11"])
                        endGoal = self.xyAlias["S11"]
                    elif self.checkNode["S12"] == 0:
                        self.updateMazeSet(self.xyAlias["S12"])
                        endGoal = self.xyAlias["S12"]
                    else:
                        self.get_logger().info(f"Store1 is busy...")
                elif endPointName == "S-2":
                    if self.checkNode["S21"] == 0:
                        self.updateMazeSet(self.xyAlias["S21"])
                        endGoal = self.xyAlias["S21"]
                    elif self.checkNode["S22"] == 0:
                        self.updateMazeSet(self.xyAlias["S22"])
                        endGoal = self.xyAlias["S22"]
                    else:
                        self.get_logger().info(f"Store2 is busy...")
                else:
                    self.get_logger().info(f"Invalid Store ID is entered...")
            elif cmd == 1: # 배달
                print("배달 상세 목적지 설정")
                print("cmd:",cmd,"endPointName : ", endPointName)
                if endPointName == "K-1":
                    if self.checkNode["K11"] == 0:
                        self.updateMazeSet(self.xyAlias["K11"])
                        endGoal = self.xyAlias["K11"]
                    elif self.checkNode["K12"] == 0:
                        self.updateMazeSet(self.xyAlias["K12"])
                        endGoal = self.xyAlias["K12"]
                    else:
                        self.get_logger().info(f"Kiosk1 is busy...")
                elif endPointName == "K-2":
                    if self.checkNode["K21"] == 0:
                        self.updateMazeSet(self.xyAlias["K21"])
                        endGoal = self.xyAlias["K21"]
                    elif self.checkNode["K22"] == 0:
                        self.updateMazeSet(self.xyAlias["K22"])
                        endGoal = self.xyAlias["K22"]
                    else:
                        self.get_logger().info(f"Kiosk2 is busy...")
                else:
                    self.get_logger().info(f"Invalid Kiosk ID is entered...")
            elif cmd == 2: # 복귀
                print("복귀 상세 목적지 설정")
                print("cmd:",cmd,"endPointName : ", endPointName)
                if endPointName == "R-1":
                    if self.checkNode["R-1"] == 0:
                        self.updateMazeSet(self.xyAlias["R-1"])
                        endGoal = self.xyAlias["R-1"]
                    else:
                        self.get_logger().info(f"R-1 is busy...")
                elif endPointName == "R-2":
                    if self.checkNode["R-2"] == 0:
                        self.updateMazeSet(self.xyAlias["R-2"])
                        endGoal = self.xyAlias["R-2"]
                    else:
                        self.get_logger().info(f"R-2 is busy...")
                elif endPointName == "R-3":
                    if self.checkNode["R-3"] == 0:
                        self.updateMazeSet(self.xyAlias["R-3"])
                        endGoal = self.xyAlias["R-3"]
                    else:
                        self.get_logger().info(f"R-3 is busy...")
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