import sys
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import time
from tf_transformations import quaternion_from_euler
import numpy as np
import sys
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import TaskResult
from action_msgs.msg import GoalStatus

from ct_package.pathDict import callPathDict
from ct_package.pathDict import deliPathDict
from ct_package.pathDict import returnPathDict

from ct_package.pathDict import robotXY_Dict
from ct_package.pathDict import storeXY_Dict
from ct_package.pathDict import kioskXY_Dict
from ct_package.pathDict import wayList
# sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# RobotControl Node import
from ct_package.drobot_control import RobotControl
from rclpy.executors import MultiThreadedExecutor



class robotTaskManager(Node):
    def __init__(self, robotControlNode):
        super().__init__()
        self.robotControl = robotControlNode

        self.timer_period = 1.0
        self.timer = self.create_timer(self.timer_period, self.robotStatusCheck_timer_callback)

        # self.subscription = self.create_subscription(String, '/chatter', self.cmd_callback, 5)
        # self.subscription = self.create_subscription(String, '/chatter', self.cmd_callback, 5)
        # self.subscription = self.create_subscription(String, '/chatter', self.cmd_callback, 5)
        # self.subscription

        self.basicNaviInit()


        self.moveCmd = "대기"

        self.node = [0] * 7
        self.node[0] = 0    # way1
        self.node[1] = 0    # way2
        self.node[2] = 0    # way3
        self.node[3] = 0    # way4 
        self.node[4] = 0    # way5
        self.node[5] = 0    # way6
        self.node[6] = 1    # way7


        self.goalNode = {
            "S11" : 0,
            "S12" : 0,
            "S21" : 0,
            "S22" : 0,
            "K11" : 0,
            "K12" : 0,
            "K21" : 0,
            "K22" : 0,
            "R1"  : 0,
            "R2"  : 0,
            "R3"  : 0,
        }


        # 현위치
        # self.currentXY = RobotName # ex) R1, R2, R3 초기값은 로봇 실행하때, yaml에서 str값 하나 받으면 좋겠는데
        self.currentXY = [None] * 3
        self.currentXY1 = None
        self.pathDict1 = None
        self.endGoal1 = None

        self.currentXY2 = None
        self.pathDict2 = None
        self.endGoal2 = None

        self.currentXY3 = None
        self.pathDict3 = None
        self.endGoal3 = None

        # path를 이동할때, 몇번째 단계인가
        self.step = 0

        self.movingFlg1 = 0
        self.movingFlg2 = 0
        self.movingFlg3 = 0

    def robotStatusCheck_timer_callback(self):
        for robot in self.robotControl.robots:
            msg = String()
            if robot.is_active:
                robot_id = robot.robot_id
                if robot.current_status == self.robotControl.RobotStatus.HOME or robot.current_status == self.robotControl.RobotStatus.AT_HOME:
                    # to 배차모드
                    self.cmd_callback("배차", robot.store_id)
                    self.get_logger().info(f"{robot_id} go to {robot.store_id}")
                    msg.data = robot.store_id
                    self.robotControl.goal_publishers[robot_id].publish(msg)

                    cmd = "배차"
                    endpoint = robot.store_id   # ex) S-1
                elif robot.current_status == self.robotControl.RobotStatus.AT_STORE and robot.current_order_status == self.robotControl.OrderStatus.DELIVERY_START:
                    # to 배달모드
                    self.get_logger().info(f"{robot_id} go to {robot.kiosk_id}")
                    msg.data = robot.kiosk_id
                    self.robotControl.goal_publishers[robot_id].publish(msg)

                    cmd = "배달"
                    endpoint = robot.kiosk_id   # ex) K-1
                elif robot.current_status == self.robotControl.RobotStatus.AT_KIOSK and robot.current_order_status == self.robotControl.OrderStatus.DELIVERY_FINISH:
                    # to 복귀모드
                    self.get_logger().info(f"go to robot {robot_id} home")
                    msg.data = f"H-{robot_id[2]}" # 추가 처리 필요
                    self.robotControl.goal_publishers[robot_id].publish(msg)
                    robot.reset()
                    print(robot)

                    cmd = "복귀"
                    endpoint = robot_id # ex) R-1

                if robot_id == "R-1":
                    self.cmd_callback1(cmd, endpoint)
                elif robot_id == "R-2":
                    self.cmd_callback2(cmd, endpoint)
                elif robot_id == "R-3":
                    self.cmd_callback3(cmd, endpoint)
                else:
                    pass

            else:
                pass
                ## 에러 처리 해야할 듯


    # robot 1
    def cmd_callback1(self, cmd, endPoint):

        if (cmd == "배차") or (cmd == "배달") or (cmd == "복귀"):
            # 임무 할당 플래그 set하기
            if self.movingFlg1 == 0:
                self.movingFlg1 = 1
                self.endGoal1 = self.nav_callback(cmd, endPoint)     # 최종 목적지 할당 및 점유 플래그 set
                print("임무 할당!!")
            elif self.movingFlg1 == 1:
                # 최종목적지 할당되어있고 waypoint 이동중
                print("keep going")
            else:
                pass

            self.gotoGoal(self.pathDict1, self.xyDict1, self.currentXY1, self.endGoal1)
        else:
            print("wrong topic msg...")


        # 임무 할당 플래그 set하기
        # if self.movingFlg == 0:
        #     self.movingFlg = 1
        #     self.nav_callback()     # 최종 목적지 할당 및 점유 플래그 set
        #     print("임무 할당!!")
        # elif self.movingFlg == 1:
        #     # 최종목적지 할당되어있고 waypoint 이동중
        #     self.gotoGoal(self.pathDict, self.xyDict, self.currentXY, self.endGoal)
        #     print("keep going")
        # else:
        #     print("wrong topic msg...")


    def basicNaviInit(self):
        try:
            # rclpy.init()
            self.nav = BasicNavigator()
            self.pose_current = PoseWithCovarianceStamped()
            self.start_time = time.time()
            self.nav.waitUntilNav2Active()
            print("basicNavi init complete!")
        except Exception as e:
            print("basicNaviInit error : ", e)
        except KeyboardInterrupt:
            self.safe_shutdown()

    def run1(self):
        try:
            while True:
                keyboardInput = input("input cmd : ")
                if keyboardInput == "1":
                    self.moveCmd = "배차"
                    self.endPoint = "S1"
                    self.nav_callback()
                elif keyboardInput == "2":
                    self.moveCmd = "배차"
                    self.endPoint = "S2"
                    self.nav_callback()
                elif keyboardInput == "3":
                    self.moveCmd = "배달"
                    self.endPoint = "K2"
                    self.nav_callback()
                else:
                    print("wrong cmd")            
        except Exception as e:
            print("error code : ",e)
        except KeyboardInterrupt:
            print("keyboard terminate")

    def run(self):
        pass

    def nav_callback(self, cmd, endPoint):
        # 최종 목적지 도착할때 까지 계속 호출해야됨
        # 명령어 발생시에만 아래의 내용이 동작하는 콜백함수로 만들어야 겠다.
        try:
            # 목적지의 점유 현황을 업데이트
            # self.updateGoalNode()

            if cmd == "배차":
                self.pathDict = callPathDict
                self.xyDict = storeXY_Dict
                if self.endPoint == "S1":
                    print(cmd, endPoint)
                    if self.goalNode["S11"] == 0:
                        self.goalNode["S11"] = 1
                        self.endGoal = "S11"
                        # 노드 점유 서비스 콜 날려
                        # self.gotoGoal(callPathDict, storeXY_Dict, self.currentXY, "S11")
                    elif self.goalNode["S12"] == 0:
                        self.goalNode["S12"] = 1
                        self.endGoal = "S12"
                        # 노드 점유 서비스 콜 날려
                        # self.gotoGoal(callPathDict, storeXY_Dict, self.currentXY, "S12")
                    else:
                        print("Store1 is busy, wait a minute")
                elif self.endPoint == "S2":
                    if self.goalNode["S21"] == 0:
                        self.goalNode["S21"] = 1
                        self.endGoal = "S21"
                        # 노드 점유 서비스 콜 날려
                        # self.gotoGoal(callPathDict, storeXY_Dict, self.currentXY, "S21")
                    elif self.goalNode["S22"] == 0:
                        self.goalNode["S22"] = 1
                        self.endGoal = "S22"
                        # 노드 점유 서비스 콜 날려
                        # self.gotoGoal(callPathDict, storeXY_Dict, self.currentXY, "S22")
                    else:
                        print("Store2 is busy, wait a minute")
                else:
                    print("Invalid Store ID entered")
            elif cmd == "배달":
                self.pathDict = deliPathDict
                self.xyDict = kioskXY_Dict
                print(self.currentXY)
                if self.endPoint == "K1":
                    if self.goalNode["K11"] == 0:
                        self.goalNode["K11"] = 1
                        self.endGoal = "K11"
                        # 노드 점유 서비스 콜 날려
                        # self.gotoGoal(deliPathDict, kioskXY_Dict, self.currentXY, "K11")
                    elif self.goalNode["K12"] == 0:
                        self.goalNode["K12"] = 1
                        self.endGoal = "K12"
                        # 노드 점유 서비스 콜 날려
                        # self.gotoGoal(deliPathDict, kioskXY_Dict, self.currentXY, "K12")
                    else:
                        pass
                elif self.endPoint == "K2":
                    if self.goalNode["K21"] == 0:
                        self.goalNode["K21"] = 1
                        self.endGoal = "K21"
                        # 노드 점유 서비스 콜 날려
                        # self.gotoGoal(deliPathDict, kioskXY_Dict, self.currentXY, "K21")
                    elif self.goalNode["K22"] == 0:
                        self.goalNode["K22"] = 1
                        self.endGoal = "K22"
                        # 노드 점유 서비스 콜 날려
                        # self.gotoGoal(deliPathDict, kioskXY_Dict, self.currentXY, "K22")
                    else:
                        pass
                else:
                    print("Invalid Kiosk ID entered")
            elif cmd == "복귀":
                self.pathDict = returnPathDict
                self.xyDict = robotXY_Dict
                if self.endPoint == "R1":
                    self.endGoal = "R1"
                    # self.gotoGoal(returnPathDict, robotXY_Dict, self.currentXY, "R1")
                elif self.endPoint == "R2":
                    self.endGoal = "R2"
                    # self.gotoGoal(returnPathDict, robotXY_Dict, self.currentXY, "R2")
                elif self.endPoint == "R3":
                    self.endGoal = "R3"
                    # self.gotoGoal(returnPathDict, robotXY_Dict, self.currentXY, "R3")
                else:
                    print("Invalid Robot address entered")
            else:
                print("Command is invalid")

            return endGoal

        except Exception as e:
            print("Exception error : ", e)
        except KeyboardInterrupt:
            print("force terminate!!")


    def gotoGoal(self, pathDict, xyDict, startPoint, endPoint):  

        pathNum = len(pathDict[startPoint][endPoint])
        print("pathNum : ", pathNum)
        noWay = 0

        for i in range(pathNum): # 1~3개 정도
            # 다음 노드를 찾을때, 현재 노드 기준으로 검색할 필요가 있어
            # 이전 step의 값이 현재 이전 shortGoal인 path에서 찾기
            
            
            
            # for문 시작할때, 노드 점유 플래그 업데이트 req 필요
            # response로 14개 노드의 플래그 상태를 받아서 업뎃
            # self.updateWayNode()


            pathLength = len(pathDict[startPoint][endPoint][i])
            if pathDict[startPoint][endPoint][i][0] is None:
                noWay = 1

            if (self.step < 1) and (noWay == 0):
                print("stepA :", self.step)
                # path로 첫 진입,
                # 이전 위치의 점유 플래그 reset 해줘야됨
                self.goalNode[startPoint] = 0

                nodeName = pathDict[startPoint][endPoint][i][self.step]
                
                # node 플래그 숫자로 변환
                num = self.transformNodeNum(nodeName)
                print("way 상태 : ", self.node[num])


                if self.node[num] == 0:
                    self.step = self.step + 1
                    self.shortGoal = wayList[nodeName]
                    print(num)
                    self.node[num] = 1 # 현재 점령한 노드 플래그값 set
                    # 노드 점유했다고 서버로 req 날려야됨


                    self.moveNavi(self.shortGoal)
                    # server에 도착했다고 응답 보내기, or 이동중 플래그 값 reset하기
                    break
                else:
                    # 대기
                    pass
            elif (self.step < pathLength) and (noWay == 0):    # 아직 way point인지 확인
                print("stepB :", self.step)
                # 이전 스텝의 값이랑 shortGoal 비교 (엉뚱한 path의 step으로 이동하면 안되니까)
                lastNodeName = pathDict[startPoint][endPoint][i][self.step-1]
                # node 플래그 숫자로 변환
                lastNum = self.transformNodeNum(lastNodeName)
                self.node[lastNum] = 0 # 이전 노드 점령 플래그 reset
                if self.shortGoal == wayList[lastNodeName]:
                    # 이길이 맞아 keep going
                    # step에 따라 검토
                    nodeName = pathDict[startPoint][endPoint][i][self.step]
                    # node 플래그 숫자로 변환
                    num = self.transformNodeNum(nodeName)
                    print("way 상태 : ", self.node[num])

                    if self.node[num] == 0:
                        self.step = self.step + 1
                        self.shortGoal = wayList[nodeName]
                        print(num)
                        self.node[num] = 1 # 현재 점령한 노드 플래그값 set
                        # 노드 점유했다고 서버로 req 날려야됨

                        self.moveNavi(self.shortGoal)
                        # server에 도착했다고 응답 보내기, or 이동중 플래그 값 reset하기
                        break
                    else:
                        # 이동가능한 다른 노드를 찾아봐
                        print("이 노드는 점령중입니다 : ", num+1)
                        pass
                else:
                    # 가던 길이 아니야, 원래 가던 path 찾아
                    pass
            elif (noWay == 1) or (self.step == pathLength):   # 최종 목적지 node로 이동할 차례
                print(self.step)
                self.shortGoal = xyDict[endPoint]
                if noWay == 1:
                    # K or S 의 node 점령 플래그 reset
                    self.goalNode[startPoint] = 0
                    pass
                else:
                    # 이전 way node 점령 플래그 reset
                    lastNodeName = pathDict[startPoint][endPoint][i][self.step-1]
                    # node 플래그 숫자로 변환
                    lastNum = self.transformNodeNum(lastNodeName)

                    self.node[lastNum] = 0 # 이전 노드 점령 플래그 reset

                # lastNodeName = pathDict[startPoint][endPoint][i][self.step-1] - 1  # index 맞추기 위해 -1
                # self.node[lastNodeName] = 0 # 이전 노드 점령 플래그 reset
                print(self.shortGoal)

                self.moveNavi(self.shortGoal)

                self.currentXY = endPoint
                self.movingFlg = 0  # 임무할당 해제
                print(self.currentXY)
                self.step = 0

                # 도착완료!!!! 신호 날려줘야함
                break

    def moveNavi(self, goal):
        x = goal[0]
        y = goal[1]
        degree = goal[2]
        gp = self.goalPose(x, y, degree)
        self.nav.goToPose(gp)
        k = 0

        while not self.nav.isTaskComplete():
            k = k + 1
            feedback = self.nav.getFeedback()    # msg.feedback
            
            # if feedback and k % 5 == 0:
            #     print('Distance remaining: ' + '{:.2f}'.format(feedback.distance_remaining) + ' meters.')

            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=20.0):
                self.nav.cancelTask()

        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            # 여기에서 뭔가를 pub
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
            
    def updateNodeStatus(self):
        # req 날려서 7개 노드의 점유상태를 response 받아
        # response 값을 list로 변환해

        # 18개 노드
        # 1, 2, 3, 4, 5, 6, 7, S11, S12, S21, S22, K11, K12, K21, K22, R1, R2, R3
        responseList = [0, 0, 0, 0, 0, 0, 0]
        responseList = msg.data.split('/')
        for i in range(len(self.node)):
            self.node[i] = responseList[i]

        self.goalNode["S11"] = responseList[7]
        self.goalNode["S12"] = responseList[8]
        self.goalNode["S21"] = responseList[9]
        self.goalNode["S22"] = responseList[10]
        self.goalNode["K11"] = responseList[11]
        self.goalNode["K12"] = responseList[12]
        self.goalNode["K21"] = responseList[13]
        self.goalNode["K22"] = responseList[14]
        self.goalNode["R1"] = responseList[15]
        self.goalNode["R2"] = responseList[16]
        self.goalNode["R3"] = responseList[17]

    
    # def updateGoalNode(self):   # ex) res.S11 : 0, res.S12 : 1, ......
    #     # self.goalNode["S11"] = res.S11
    #     pass

    def safe_shutdown(self):
        self.nav.destroy_node()
        # nav.lifecycleShutdown()  # Shut down the navigator
        rclpy.shutdown()  # Shut down rclpy
        print("Node has been safely shut down.")

    def goalPose(self, p_x, p_y, degree):
        tmp = [0, 0, degree]
        # print(np.array(tmp)*to_radian)
        orientation_val = quaternion_from_euler(tmp[0], tmp[1], tmp[2])
        # print(orientation_val)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = p_x
        goal_pose.pose.position.y = p_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = orientation_val[2]
        goal_pose.pose.orientation.w = orientation_val[3]

        return goal_pose
    
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

    executor = MultiThreadedExecutor()

    robotControlInst = RobotControl()
    robotTaskManagerInst = robotTaskManager(robotControlNode = robotControlInst)

    executor.add_node(robotControlInst)
    executor.add_node(robotTaskManagerInst)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        robotControlInst.destroy_node()
        robotTaskManagerInst.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()