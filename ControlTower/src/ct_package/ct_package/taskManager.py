import sys
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16, Float32MultiArray


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
from ct_package.pathDict import endPoint_Dict
from ct_package.pathDict import wayList
# sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# RobotControl Node import
from ct_package.drobot_control import RobotControl
from rclpy.executors import MultiThreadedExecutor

from interface_package.srv import RobotArrival



class robotTaskManager(Node):
    def __init__(self, robotControlNode):
        super().__init__('taskManager')
        self.robotControl = robotControlNode

        self.timer_period = 3.0
        self.timer = self.create_timer(self.timer_period, self.robotStatusCheck_timer_callback)

        self.robotArrivalSigServer = self.create_service(RobotArrival, 'robotArrival', self.robotArrival_callback_service)

        # self.basicNaviInit()

        # self.shortGoalPub = {
        #     "R-1": self.create_publisher(Float32MultiArray, 'shortGoal_1', 10),
        #     "R-2": self.create_publisher(Float32MultiArray, 'shortGoal_2', 10),
        #     "R-3": self.create_publisher(Float32MultiArray, 'shortGoal_3', 10)
        # }

        self.shortGoalPub = {
            "R-1": self.create_publisher(Int16, 'shortGoal_1', 10),
            "R-2": self.create_publisher(Int16, 'shortGoal_2', 10),
            "R-3": self.create_publisher(Int16, 'shortGoal_3', 10)
        }
        


        self.moveCmd = "대기"

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
            "R1"  : 0,
            "R2"  : 0,
            "R3"  : 0,
        }


        # 현위치
        # self.currentXY = RobotName # ex) R1, R2, R3 초기값은 로봇 실행하때, yaml에서 str값 하나 받으면 좋겠는데        
        
        self.currentXY1 = None
        self.currentXY2 = None
        self.currentXY3 = None

        self.pathDict1 = None
        self.pathDict2 = None
        self.pathDict3 = None

        self.xyDict1 = None   
        self.xyDict2 = None
        self.xyDict3 = None
        
        self.endGoal1 = None
        self.endGoal2 = None
        self.endGoal3 = None



        # path를 이동할때, 몇번째 단계인가
        self.step1 = 0
        self.step2 = 0
        self.step3 = 0

        self.movingFlg1 = 0
        self.movingFlg2 = 0
        self.movingFlg3 = 0

    def robotArrival_callback_service(self, request, response):
        try:
            if request.arrival == 1:
                self.movingWayFlg1 = 1
                response.success = True
            elif request.arrival == 2:
                self.movingWayFlg2 = 2
                response.success = True
            elif request.arrival == 3:
                self.movingWayFlg3 = 3
                response.success = True
            else:
                response.success = False
        except Exception as e:
            print("robotArrival service Error : ",e)
        

    def robotStatusCheck_timer_callback(self):
        for robot in self.robotControl.robots:
            msg = String()
            if robot.is_active:
                robot_id = robot.robot_id
                if robot.current_status == self.robotControl.RobotStatus.HOME or robot.current_status == self.robotControl.RobotStatus.AT_HOME:
                    # to 배차모드
                    self.get_logger().info(f"{robot_id} go to {robot.store_id}")
                    # msg.data = robot.store_id
                    # self.robotControl.goal_publishers[robot_id].publish(msg)

                    cmd = 0
                    endPoint = robot.store_id   # ex) S-1
                elif robot.current_status == self.robotControl.RobotStatus.AT_STORE and robot.current_order_status == self.robotControl.OrderStatus.DELIVERY_START:
                    # to 배달모드
                    self.get_logger().info(f"{robot_id} go to {robot.kiosk_id}")
                    # msg.data = robot.kiosk_id
                    # self.robotControl.goal_publishers[robot_id].publish(msg)

                    cmd = 1
                    endPoint = robot.kiosk_id   # ex) K-1
                elif robot.current_status == self.robotControl.RobotStatus.AT_KIOSK and robot.current_order_status == self.robotControl.OrderStatus.DELIVERY_FINISH:
                    # to 복귀모드
                    self.get_logger().info(f"go to robot {robot_id} home")
                    # msg.data = f"H-{robot_id[2]}" # 추가 처리 필요
                    # self.robotControl.goal_publishers[robot_id].publish(msg)

                    cmd = 2
                    endPoint = robot_id # ex) R-1

                    robot.reset()
                    print(robot)


                if robot_id == "R-1":
                    self.cmd_callback1(robot_id, cmd, endPoint)
                elif robot_id == "R-2":
                    self.cmd_callback2(robot_id, cmd, endPoint)
                elif robot_id == "R-3":
                    self.cmd_callback3(robot_id, cmd, endPoint)
                else:
                    pass

            else:
                pass
                ## 에러 처리 해야할 듯


    def cmd_callback1(self, robotId, cmd, endPoint):
      
        if (cmd == 0) or (cmd == 1) or (cmd == 2):
            # 임무 할당 플래그 set하기
            if self.movingFlg1 == 0:
                self.movingFlg1 = 1
                self.endGoal1 = self.nav_callback(robotId, cmd, endPoint)     # 최종 목적지 할당 및 점유 플래그 set
                print("robot1 임무 할당!!")
            elif self.movingFlg1 == 1:
                # 최종목적지 할당되어있고 waypoint 이동중
                print("robot1 keep going")
            else:
                pass

            self.gotoGoal(robotId, self.pathDict1, self.xyDict1, self.currentXY1, self.endGoal1)
        else:
            print("wrong topic msg...")

    def cmd_callback2(self, robotId, cmd, endPoint):
      
        if (cmd == 0) or (cmd == 1) or (cmd == 2):
            # 임무 할당 플래그 set하기
            if self.movingFlg2 == 0:
                self.movingFlg2 = 1
                self.endGoal2 = self.nav_callback(robotId, cmd, endPoint)     # 최종 목적지 할당 및 점유 플래그 set
                print("robot2 임무 할당!!")
            elif self.movingFlg2 == 1:
                # 최종목적지 할당되어있고 waypoint 이동중
                print("robot1 keep going")
            else:
                pass

            self.gotoGoal(robotId, self.pathDict2, self.xyDict2, self.currentXY2, self.endGoal2)
        else:
            print("wrong topic msg...")

    def cmd_callback3(self, robotId, cmd, endPoint):
      
        if (cmd == 0) or (cmd == 1) or (cmd == 2):
            # 임무 할당 플래그 set하기
            if self.movingFlg3 == 0:
                self.movingFlg3 = 1
                self.endGoal3 = self.nav_callback(robotId, cmd, endPoint)     # 최종 목적지 할당 및 점유 플래그 set
                print("robot3 임무 할당!!")
            elif self.movingFlg3 == 1:
                # 최종목적지 할당되어있고 waypoint 이동중
                print("robot1 keep going")
            else:
                pass

            self.gotoGoal(robotId, self.pathDict3, self.xyDict3, self.currentXY3, self.endGoal3)
        else:
            print("wrong topic msg...")


    def run(self):
        pass

    def nav_callback(self, robotId, cmd, endPoint):
        # 최종 목적지 도착할때 까지 계속 호출해야됨
        # 명령어 발생시에만 아래의 내용이 동작하는 콜백함수로 만들어야 겠다.
        try:
            if cmd == 0: # 배차
                if robotId == "R-1":
                    self.pathDict1 = callPathDict
                    self.xyDict1 = storeXY_Dict
                elif robotId == "R-2":
                    self.pathDict2 = callPathDict
                    self.xyDict2 = storeXY_Dict
                elif robotId == "R-3":
                    self.pathDict3 = callPathDict
                    self.xyDict3 = storeXY_Dict

                if endPoint == "S-1":
                    print(cmd, endPoint)
                    if self.goalNode["S11"] == 0:
                        self.goalNode["S11"] = 1
                        endGoal = "S11"
                    elif self.goalNode["S12"] == 0:
                        self.goalNode["S12"] = 1
                        endGoal = "S12"
                    else:
                        print("Store1 is busy, wait a minute")
                elif endPoint == "S-2":
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
                if robotId == "R-1":
                    self.pathDict1 = deliPathDict
                    self.xyDict1 = kioskXY_Dict
                elif robotId == "R-2":
                    self.pathDict2 = deliPathDict
                    self.xyDict2 = kioskXY_Dict
                elif robotId == "R-3":
                    self.pathDict3 = deliPathDict
                    self.xyDict3 = kioskXY_Dict

                print(self.currentXY)
                if endPoint == "K-1":
                    if self.goalNode["K11"] == 0:
                        self.goalNode["K11"] = 1
                        endGoal = "K11"
                    elif self.goalNode["K12"] == 0:
                        self.goalNode["K12"] = 1
                        endGoal = "K12"
                    else:
                        pass
                elif endPoint == "K-2":
                    if self.goalNode["K21"] == 0:
                        self.goalNode["K21"] = 1
                        endGoal = "K21"
                    elif self.goalNode["K22"] == 0:
                        self.goalNode["K22"] = 1
                        endGoal = "K22"
                    else:
                        pass
                else:
                    print("Invalid Kiosk ID entered")
            elif cmd == 2: # 복귀
                if robotId == "R-1":
                    self.pathDict1 = returnPathDict
                    self.xyDict1 = robotXY_Dict
                elif robotId == "R-2":
                    self.pathDict2 = returnPathDict
                    self.xyDict2 = robotXY_Dict
                elif robotId == "R-3":
                    self.pathDict3 = returnPathDict
                    self.xyDict3 = robotXY_Dict

                if endPoint == "R-1":
                    endGoal = "R-1"
                elif endPoint == "R-2":
                    endGoal = "R-2"
                elif endPoint == "R-3":
                    endGoal = "R-3"
                else:
                    print("Invalid Robot address entered")
            else:
                print("Command is invalid")

            return endGoal

        except Exception as e:
            print("Exception error : ", e)
        except KeyboardInterrupt:
            print("force terminate!!")


    def gotoGoal(self, robotId, pathDict, xyDict, startPoint, endPoint):
        if robotId == "R-1":
            step = self.step1
            shortGoal = self.shortGoal1
            movingWayFlg = self.movingWayFlg1
        elif robotId == "R-2":
            step = self.step2
            shortGoal = self.shortGoal2
            movingWayFlg = self.movingWayFlg2
        elif robotId == "R-3":
            step = self.step3
            shortGoal = self.shortGoal3
            movingWayFlg = self.movingWayFlg3
        else:
            print("wrong robotId")


        if movingWayFlg==1:
            # 실행
            movingWayFlg = 0
            msg = Int16()

            pathNum = len(pathDict[startPoint][endPoint])
            print("pathNum : ", pathNum)
            noWay = 0

            for i in range(pathNum): # 1~3개 정도
                # 다음 노드를 찾을때, 현재 노드 기준으로 검색할 필요가 있어
                # 이전 step의 값이 현재 이전 shortGoal인 path에서 찾기 
                
                pathLength = len(pathDict[startPoint][endPoint][i])
                if pathDict[startPoint][endPoint][i][0] is None:
                    noWay = 1

                if (step < 1) and (noWay == 0):
                    # path로 첫 진입,
                    # 이전 위치의 점유 플래그 reset 해줘야됨
                    self.goalNode[startPoint] = 0

                    nodeNum = pathDict[startPoint][endPoint][i][step]
                    
                    # node 플래그 숫자로 변환
                    num = self.transformNodeNum(nodeNum)

                    if self.node[num] == 0:
                        step = step + 1
                        # shortGoal = wayList[nodeNum]
                        print(num)
                        self.node[num] = 1 # 현재 점령한 노드 플래그값 set

                        # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
                        # drobot_Motor 한테 self.shortGoal 보내기
                        msg.data = nodeNum # int
                        self.shortGoalPub[robotId].publish(msg)
                        # self.moveNavi(self.shortGoal)
                        # server에 도착했다고 응답 보내기, or 이동중 플래그 값 reset하기
                        break
                    else:
                        # 대기
                        pass
                elif (step < pathLength) and (noWay == 0):    # 아직 way point인지 확인
                    # 이전 스텝의 값이랑 shortGoal 비교 (엉뚱한 path의 step으로 이동하면 안되니까)
                    lastNodeName = pathDict[startPoint][endPoint][i][step-1]
                    # node 플래그 숫자로 변환
                    lastNum = self.transformNodeNum(lastNodeName)
                    self.node[lastNum] = 0 # 이전 노드 점령 플래그 reset
                    if shortGoal == wayList[lastNodeName]:
                        # 이길이 맞아 keep going
                        # step에 따라 검토
                        nodeNum = pathDict[startPoint][endPoint][i][step]
                        # node 플래그 숫자로 변환
                        num = self.transformNodeNum(nodeNum)

                        if self.node[num] == 0:
                            step = step + 1
                            # shortGoal = wayList[nodeName]
                            print(num)
                            self.node[num] = 1 # 현재 점령한 노드 플래그값 set

                            # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
                            # drobot_Motor 한테 self.shortGoal 보내기
                            msg.data = nodeNum
                            self.shortGoalPub[robotId].publish(msg)
                            # self.moveNavi(self.shortGoal)
                            # server에 도착했다고 응답 보내기, or 이동중 플래그 값 reset하기
                            break
                        else:
                            # 이동가능한 다른 노드를 찾아봐
                            print("이 노드는 점령중입니다 : ", num+1)
                    else:
                        # 가던 길이 아니야, 원래 가던 path 찾아
                        pass
                elif (noWay == 1) or (step == pathLength):   # 최종 목적지 node로 이동할 차례
                    print(step)
                    # shortGoal = xyDict[endPoint]
                    nodeNum = endPoint_Dict[endPoint]
                    if noWay == 1:
                        # K or S 의 node 점령 플래그 reset
                        self.goalNode[startPoint] = 0
                    else:
                        # 이전 way node 점령 플래그 reset
                        lastNodeName = pathDict[startPoint][endPoint][i][step-1]
                        # node 플래그 숫자로 변환
                        lastNum = self.transformNodeNum(lastNodeName)
                        # 이전 노드 점령 플래그 reset
                        self.node[lastNum] = 0 

                    # print(shortGoal)

                    # self.moveNavi(self.shortGoal)
                    # @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
                    # drobot_Motor 한테 self.shortGoal 보내기
                    msg.data = nodeNum
                    self.shortGoalPub[robotId].publish(msg)

                    if robotId == "R-1":
                        self.currentXY1 = endPoint
                        self.movingFlg1 = 0  # 임무할당 해제
                        self.step1 = 0
                        print(self.currentXY1)
                    elif robotId == "R-2":
                        self.currentXY2 = endPoint
                        self.movingFlg2 = 0  # 임무할당 해제
                        self.step2 = 0
                        print(self.currentXY2)
                    elif robotId == "R-3":
                        self.currentXY3 = endPoint
                        self.movingFlg3 = 0  # 임무할당 해제
                        self.step3 = 0
                        print(self.currentXY3)
                    else:
                        print("wrong robotId")

                    break

                if robotId == "R-1":
                    self.step1 = step
                    self.shortGoal1 = nodeNum
                    self.movingWayFlg1 = movingWayFlg
                elif robotId == "R-2":
                    self.step2 = step
                    self.shortGoal2 = nodeNum
                    self.movingWayFlg2 = movingWayFlg
                elif robotId == "R-3":
                    self.step3 = step
                    self.shortGoal3 = nodeNum
                    self.movingWayFlg3 = movingWayFlg
                else:
                    print("wrong robotId")
        elif movingWayFlg==0:
            print("waypoint 이동중")


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
    





    # def basicNaviInit(self):
    #     try:
    #         # rclpy.init()
    #         self.nav = BasicNavigator()
    #         self.pose_current = PoseWithCovarianceStamped()
    #         self.start_time = time.time()
    #         self.nav.waitUntilNav2Active()
    #         print("basicNavi init complete!")
    #     except Exception as e:
    #         print("basicNaviInit error : ", e)
    #     except KeyboardInterrupt:
    #         self.safe_shutdown()

    # def safe_shutdown(self):
    #     self.nav.destroy_node()
    #     # nav.lifecycleShutdown()  # Shut down the navigator
    #     rclpy.shutdown()  # Shut down rclpy
    #     print("Node has been safely shut down.")

    # def goalPose(self, p_x, p_y, degree):
    #     tmp = [0, 0, degree]
    #     # print(np.array(tmp)*to_radian)
    #     orientation_val = quaternion_from_euler(tmp[0], tmp[1], tmp[2])
    #     # print(orientation_val)

    #     goal_pose = PoseStamped()
    #     goal_pose.header.frame_id = 'map'
    #     goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
    #     goal_pose.pose.position.x = p_x
    #     goal_pose.pose.position.y = p_y
    #     goal_pose.pose.position.z = 0.0
    #     goal_pose.pose.orientation.x = 0.0
    #     goal_pose.pose.orientation.y = 0.0
    #     goal_pose.pose.orientation.z = orientation_val[2]
    #     goal_pose.pose.orientation.w = orientation_val[3]

    #     return goal_pose
    

    # def moveNavi(self, goal):
    #     x = goal[0]
    #     y = goal[1]
    #     degree = goal[2]
    #     gp = self.goalPose(x, y, degree)
    #     self.nav.goToPose(gp)
    #     k = 0

    #     while not self.nav.isTaskComplete():
    #         k = k + 1
    #         feedback = self.nav.getFeedback()    # msg.feedback
            
    #         # if feedback and k % 5 == 0:
    #         #     print('Distance remaining: ' + '{:.2f}'.format(feedback.distance_remaining) + ' meters.')

    #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=20.0):
    #             self.nav.cancelTask()

    #     result = self.nav.getResult()
    #     if result == TaskResult.SUCCEEDED:
    #         # 여기에서 뭔가를 pub
    #         print('Goal succeeded!')
    #     elif result == TaskResult.CANCELED:
    #         print('Goal was canceled!')
    #     elif result == TaskResult.FAILED:
    #         print('Goal failed!')
            

    
    
    



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