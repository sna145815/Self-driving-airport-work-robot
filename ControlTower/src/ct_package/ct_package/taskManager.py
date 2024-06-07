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


class robotTaskManager(RobotControl):
    def __init__(self):
        super().__init__()

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
        

        # 각 노드의 점유상태 플래그 초기화
        self.node = [0] * 7
        self.node[0] = 0    # way1
        self.node[1] = 0    # way2
        self.node[2] = 0    # way3
        self.node[3] = 0    # way4 
        self.node[4] = 0    # way5
        self.node[5] = 0    # way6
        self.node[6] = 0    # way7

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
 
        
        # 관리할 로봇별 변수 초기화      
        self.shortGoal1 = None
        self.shortGoal2 = None
        self.shortGoal3 = None

        # path를 이동할때, 몇번째 단계인가
        self.step1 = 0
        self.step2 = 0
        self.step3 = 0

        # 계획된 path에서 경유지가 없을때, 1로 set (ex) S11 -> K12 바로 갈때
        self.noWay1 = 0
        self.noWay2 = 0
        self.noWay3 = 0

        # 최종 목적지로 이동중이면 1로 set, R1, K11, S11 등에 도착한 상태이면 0으로 clear
        # self.robot_1.movingFlg


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
                        robot.lastEndPoint = robotId
                        robot.pathDict = callPathDict
                        # robot.startPoint = robotId
                        robot.startPoint = robot.lastEndPoint
                        robot.endPoint = self.nav_callback(robotId, 0, robot.store_id)                # 세부목적지(K11, S11 등등) 설정 및 점유 상태 업데이트
                        self.gotoGoal(robotId, robot.pathDict, robot.startPoint, robot.endPoint)                 # path 계산               
                    elif (robot.current_status == RobotStatus.AT_STORE) :           # 배달이냐
                        if robot.current_order_status == OrderStatus.DELIVERY_START:            # 카드 찍혔냐
                            robot.movingFlg = 1
                            robot.lastEndPoint = robot.endPoint
                            robot.pathDict = deliPathDict
                            # robot.startPoint = robot.store_id
                            robot.startPoint = robot.lastEndPoint
                            robot.endPoint = self.nav_callback(robotId, 1, robot.kiosk_id)            # 목적지 설정 및 점유 상태 업데이트
                            self.gotoGoal(robotId, robot.pathDict, robot.startPoint, robot.endPoint)      # path계산
                        else:
                            self.get_logger().info(f"{robotId} : Waiting RFID tag")
                    elif (robot.current_status == RobotStatus.AT_KIOSK) :           # 복귀냐
                        if robot.current_order_status == OrderStatus.DELIVERY_FINISH:           # 카드찍혔냐
                            # robot.is_active = False
                            robot.movingFlg = 1
                            robot.lastEndPoint = robot.endPoint
                            robot.pathDict = returnPathDict
                            # robot.startPoint = robot.kiosk_id
                            robot.startPoint = robot.lastEndPoint
                            robot.endPoint = self.nav_callback(robotId, 2, robotId)                   # 목적지 설정 및 점유 상태 업데이트
                            self.gotoGoal(robotId, robot.pathDict, robot.startPoint, robot.endPoint)    # path계산
                            robot.returning()
                        else:
                            self.get_logger().info(f"{robotId} : Waiting RFID tag")
                    else:
                        self.get_logger().info(f"{robotId} STATUS : {robot.current_status} and {robot.current_order_status}")
                else:
                    # waypoint간 이동 상태일때는, 서비스 콜백 함수에서 처리함
                    self.get_logger().info(f"{robotId} : Still being delivered")

        print("----------------------------------------------")
    

    def shortGoal_serviceCall(self, robotId, nodeNum):
        print("-------------------  service call start------------------")
        if robotId == "R-1":
            self.reqShortGoal1.nodenum = nodeNum
            self.shortGoalClient1.call_async(self.reqShortGoal1)
            self.get_logger().info(f"{robotId} : request complete")
        elif robotId == "R-2":
            self.reqShortGoal2.nodenum = nodeNum
            self.shortGoalClient2.call_async(self.reqShortGoal2)
            self.get_logger().info(f"{robotId} : request complete")
        elif robotId == "R-3":
            self.reqShortGoal3.nodenum = nodeNum
            self.shortGoalClient3.call_async(self.reqShortGoal3)
            self.get_logger().info(f"{robotId} : request complete")
        else:
            self.get_logger().info(f"wrong Robot Id is entered...")
        
        print("-------------------  service call end------------------")


    # robot1이 노드이동을 완료했을때, req날라오면 실행되는함수
    def robotArrival_callback_service_1(self, request, response):
        try:
            print("------robotArrival_callback_service Start 1111111--------")
            # 노드번호가 종착지인지 way인지 구분해서 다음 동작을 정의해줘라
            num = request.nodenum
            pathDict = self.robot_1.pathDict
            start = self.robot_1.startPoint     # R-1, S11, K11
            end = self.robot_1.endPoint         # R-1, S11, K11
            if 0 <= num < 28:   # waypoint 도착
                response.success = True
                self.gotoGoal(self.robot_1.robot_id, pathDict, start, end)    # path계산 & service req까지
            elif 28 <= num < 39:    # 종착지 도착
                # 종착지 도착 플래그 set
                response.success = True
                self.robot_1.movingFlg = 0
            else:
                response.success = False
                self.get_logger().info(f"R-1 : wrong node number request!")

            return response
        except Exception as e:
            self.get_logger().info(f"robotArrival service1 Error : {e}")
    

    # robot2이 노드이동을 완료했을때, req날라오면 실행되는함수
    def robotArrival_callback_service_2(self, request, response):
        try:
            print("------robotArrival_callback_service Start 2222222--------")
            # 노드번호가 종착지인지 way인지 구분해서 다음 동작을 정의해줘라
            num = request.nodenum
            pathDict = self.robot_2.pathDict
            start = self.robot_2.startPoint     # R-1, S11, K11
            end = self.robot_2.endPoint         # R-1, S11, K11
            if 0 <= num < 28:   # waypoint 도착
                response.success = True
                self.gotoGoal(self.robot_2.robot_id, pathDict, start, end)    # path계산 & service req까지
            elif 28 <= num < 39:    # 종착지 도착
                # 종착지 도착 플래그 set
                response.success = True
                self.robot_2.movingFlg = 0
            else:
                response.success = False
                self.get_logger().info(f"R-2 : wrong node number request!")

            return response
        except Exception as e:
            self.get_logger().info(f"robotArrival service2 Error : {e}")


    # robot3이 노드이동을 완료했을때, req날라오면 실행되는함수
    def robotArrival_callback_service_3(self, request, response):
        try:
            print("------robotArrival_callback_service Start 3333333--------")
            # 노드번호가 종착지인지 way인지 구분해서 다음 동작을 정의해줘라
            num = request.nodenum
            pathDict = self.robot_3.pathDict
            start = self.robot_3.startPoint     # R-1, S11, K11
            end = self.robot_3.endPoint         # R-1, S11, K11
            if 0 <= num < 28:   # waypoint 도착
                response.success = True
                self.gotoGoal(self.robot_3.robot_id, pathDict, start, end)    # path계산 & service req까지
            elif 28 <= num < 39:    # 종착지 도착
                # 종착지 도착 플래그 set
                response.success = True
                self.robot_3.movingFlg = 0
            else:
                response.success = False
                self.get_logger().info(f"R-3 : wrong node number request!")

            return response
        except Exception as e:
            self.get_logger().info(f"robotArrival service3 Error : {e}")
            

    def nav_callback(self, robotId, cmd, endPointName):
        # 최종 목적지 도착할때 까지 계속 호출해야됨
        print("---------------nav callback start-------------------------")
        endGoal = None
        try:
            if cmd == 0: # 배차
                print("배차 상세 목적지 설정")
                if robotId == "R-1":
                    self.robot_1.pathDict = callPathDict
                elif robotId == "R-2":
                    self.robot_2.pathDict = callPathDict
                elif robotId == "R-3":
                    self.robot_3.pathDict = callPathDict

                print("cmd:",cmd,"endPointName : ", endPointName)
                if endPointName == "S-1":
                    if self.goalNode["S11"] == 0:
                        self.goalNode["S11"] = 1
                        endGoal = "S11"
                    elif self.goalNode["S12"] == 0:
                        self.goalNode["S12"] = 1
                        endGoal = "S12"
                    else:
                        print("Store1 is busy, wait a minute")
                elif endPointName == "S-2":
                    if self.goalNode["S21"] == 0:
                        self.goalNode["S21"] = 1
                        endGoal = "S21"
                    elif self.goalNode["S22"] == 0:
                        self.goalNode["S22"] = 1
                        endGoal = "S22"
                    else:
                        print("Store2 is busy, wait a minute")
                else:
                    print("Invalid Store ID entered")
            elif cmd == 1: # 배달
                print("배달 상세 목적지 설정")
                if robotId == "R-1":
                    self.robot_1.pathDict = deliPathDict
                elif robotId == "R-2":
                    self.robot_2.pathDict = deliPathDict
                elif robotId == "R-3":
                    self.robot_3.pathDict = deliPathDict

                print("cmd:",cmd,"endPointName : ", endPointName)
                if endPointName == "K-1":
                    if self.goalNode["K11"] == 0:
                        self.goalNode["K11"] = 1
                        endGoal = "K11"
                    elif self.goalNode["K12"] == 0:
                        self.goalNode["K12"] = 1
                        endGoal = "K12"
                    else:
                        self.get_logger().info(f"{robotId}, {endPointName}: no delivery goal, wait")
                elif endPointName == "K-2":
                    if self.goalNode["K21"] == 0:
                        self.goalNode["K21"] = 1
                        endGoal = "K21"
                    elif self.goalNode["K22"] == 0:
                        self.goalNode["K22"] = 1
                        endGoal = "K22"
                    else:
                        self.get_logger().info(f"{robotId}, {endPointName}: no delivery goal, wait")
                else:
                    print("Invalid Kiosk ID entered")
            elif cmd == 2: # 복귀
                print("복귀 상세 목적지 설정")
                if robotId == "R-1":
                    self.robot_1.pathDict = returnPathDict
                elif robotId == "R-2":
                    self.robot_2.pathDict = returnPathDict
                elif robotId == "R-3":
                    self.robot_3.pathDict = returnPathDict

                print("cmd:",cmd,"endPointName : ", endPointName)
                if endPointName == "R-1":
                    endGoal = "R-1"
                elif endPointName == "R-2":
                    endGoal = "R-2"
                elif endPointName == "R-3":
                    endGoal = "R-3"
                else:
                    print("Invalid Robot address entered")
            else:
                print("Command is invalid")

            print("---------------nav callback end-------------------------")

            return endGoal

        except Exception as e:
            self.get_logger().info(f"nav_callback Exception Error : {e}")
        except KeyboardInterrupt:
            self.get_logger().info(f"nav_callback keyboard terminate")


    def gotoGoal(self, robotId, pathDict, startPoint, finalGoal):
        print("---------------gotoGoal start-------------------------")
        print("robotId : ",robotId)
        print("startPoint : ",startPoint)
        print("finalGoal : ",finalGoal)

        if robotId == "R-1":
            lastEndPoint = self.robot_1.lastEndPoint
            noWay = self.noWay1
            step = self.step1
            shortGoal = self.shortGoal1
        elif robotId == "R-2":
            lastEndPoint = self.robot_2.lastEndPoint
            noWay = self.noWay2
            step = self.step2
            shortGoal = self.shortGoal2
        elif robotId == "R-3":
            lastEndPoint = self.robot_3.lastEndPoint
            noWay = self.noWay3
            step = self.step3
            shortGoal = self.shortGoal3
        else:
            print("wrong robotId")

        print("robotId : ", robotId)
        print("-------------------------------------------------")

        pathNum = len(pathDict[startPoint][finalGoal])
        print("pathNum : ", pathNum)

        for i in range(pathNum): # 1~3개 정도
            # 다음 노드를 찾을때, 현재 노드 기준으로 검색할 필요가 있어
            # 이전 step의 값이 현재 이전 shortGoal인 path에서 찾기 
            
            pathLength = len(pathDict[startPoint][finalGoal][i])
            print("pathLength : ", pathLength)
            if pathDict[startPoint][finalGoal][i][0] is None:
                # 경유지 없음
                noWay = 1

            if (step < 1) and (noWay == 0):
                print("########  first node  ################")
                # path로 첫 진입,
                # 이전 위치의 점유 플래그 reset 해줘야됨
                self.goalNode[startPoint] = 0

                nodeNum = pathDict[startPoint][finalGoal][i][step] # 0 ~ 38
                
                # node 플래그 숫자로 변환
                num = self.transformNodeNum(nodeNum)    # 0 ~ 6

                if self.node[num] == 0: # 점유상태 아니면
                    step = step + 1
                    # shortGoal = wayList[nodeNum]
                    print("nodeNum : ", nodeNum)
                    print("WayPoint name : ", num)
                    print("path step : ", step)
                    self.node[num] = 1 # 목표한 waypoint 점유 플래그 set

                    # server에 shortGoal service requst
                    self.shortGoal_serviceCall(robotId, nodeNum)
                    # 현재 노드 점유 플래그 clear
                    self.goalNode[lastEndPoint] = 0
                    break
                else:
                    # 대기
                    self.get_logger().info(f"{robotId} : Waypoint{num+1} is busy")
            elif (0 < step < pathLength) and (noWay == 0):    # 아직 way point인지 확인
                print("#######  2nd node  ################")
                print("step : ", step)
                # 이전 스텝의 값이랑 shortGoal 비교 (엉뚱한 path의 step으로 이동하면 안되니까)
                lastNodeNum = pathDict[startPoint][finalGoal][i][step-1]
                lastNum = self.transformNodeNum(lastNodeNum)
                print("lastNodeNum : ", lastNodeNum)

                # self.node[lastNum] = 0 # 이전 노드 점령 플래그 reset

                if shortGoal == lastNodeNum:
                    # 이길이 맞아 keep going
                    # step에 따라 검토
                    nodeNum = pathDict[startPoint][finalGoal][i][step]
                    print("@@nodeNum : ",nodeNum)
                    # node 플래그 숫자로 변환
                    num = self.transformNodeNum(nodeNum)

                    if self.node[num] == 0:
                        step = step + 1
                        # shortGoal = wayList[nodeName]
                        print("점령한 waypoint : ", num)
                        self.node[num] = 1 # 현재 점령한 노드 플래그값 set

                        # server에 shortGoal service requst
                        self.shortGoal_serviceCall(robotId, nodeNum)
                        # 현재 노드 점유 플래그 clear
                        self.node[lastNum] = 0

                        break
                    else:
                        # 이동가능한 다른 노드를 찾아봐
                        self.get_logger().info(f"{robotId} : Waypoint{num+1} is busy")
                else:
                    # 가던 길이 아니야, 원래 가던 path 찾아
                    print("wrong pathhhhh")
            elif (noWay == 1) or (step == pathLength):   # 최종 목적지 node로 이동할 차례
                print("################# final destination #####################")
                print("step : ", step)
                nodeNum = endPoint_Dict[finalGoal]
                print("nodeNum : ", nodeNum)

                # 최종 목적지의 점유상태는 nav_callback에서 이미 set함
                # server에 shortGoal service requst
                self.shortGoal_serviceCall(robotId, nodeNum)
                
                # 현재 노드 점유 플래그 clear
                if noWay == 1:  # S11 or K11 node 
                    self.goalNode[startPoint] = 0
                else:           # waypoint node
                    lastNodeName = pathDict[startPoint][finalGoal][i][step-1]
                    lastNum = self.transformNodeNum(lastNodeName)
                    self.node[lastNum] = 0 

                step = 0
                noWay = 0
                break

        print("for end$$$$$$$$$$$")
        if robotId == "R-1":
            self.step1 = step
            self.shortGoal1 = nodeNum
            self.noWay1 = 0
        elif robotId == "R-2":
            self.step2 = step
            self.shortGoal2 = nodeNum
            self.noWay1 = 0
        elif robotId == "R-3":
            self.step3 = step
            self.shortGoal3 = nodeNum
            self.noWay1 = 0
        else:
            print("wrong robotId")

        
        print("---------------gotoGoal end-------------------------")


    def transformNodeNum(self, nodeName):
        if 0 <= nodeName < 4 :
            Num = 0
        elif 4 <= nodeName < 8:
            Num = 1
        elif 8 <= nodeName < 12:
            Num = 2
        elif 12 <= nodeName < 16:
            Num = 3
        elif 16 <= nodeName < 20:
            Num = 4
        elif 20 <= nodeName < 24:
            Num = 5
        elif 24 <= nodeName < 28:
            Num = 6
        else:
            pass

        return Num
    


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