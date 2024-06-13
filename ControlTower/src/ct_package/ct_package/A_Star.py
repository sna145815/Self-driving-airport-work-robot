import heapq

class goalNode:
    def __init__(self, parent=None, position=None):
        self.parent = parent            # 이전 노드 ex) (0,0)
        self.position = position        # 현재 위치 ex) (0,0)
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f

class aStar:
    def __init__(self):
        print("start")

    def heuristicDiagonal(self, node, goal, D=1, D2=2 ** 0.5):  # Diagonal Distance
        dx = abs(node.position[0] - goal.position[0])
        dy = abs(node.position[1] - goal.position[1])
        return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)
    
    def heuristicManhattan(self, node, goal, D=1):
        dx = abs(node.position[0] - goal.position[0])
        dy = abs(node.position[1] - goal.position[1])
        return D * (dx + dy)

    def calculatePath(self, maze, start, end):
        print("calculate")
        # startNode와 endNode 초기화
        startNode = goalNode(None, start)
        endNode = goalNode(None, end)

        # openList, closedList 초기화
        openList = []
        closedSet = set()

        # openList에 시작 노드 추가
        heapq.heappush(openList, startNode)

        # endNode를 찾을 때까지 실행
        while openList:
            # 현재 노드 지정
            currentNode = heapq.heappop(openList)
            closedSet.add(currentNode.position)

            print("currentNode : ",currentNode.position)
    
            # 현재 노드가 목적지면 current.position 추가하고
            # current의 부모로 이동
            if currentNode == endNode:
                print("종점 도착")
                path = []
                while currentNode is not None:
                    path.append(currentNode.position)
                    currentNode = currentNode.parent
                return path[::-1]
            
            print("아직 종점 아님")

            # 인접한 xy좌표 전부, DFS
            # for newPosition in (0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1):
            for newPosition in (0, -1), (0, 1), (-1, 0), (1, 0):
                # 노드 위치 업데이트
                nodePosition = (currentNode.position[0] + newPosition[0], currentNode.position[1] + newPosition[1])

                # 미로 maze index 범위 안에 있어야함
                if nodePosition[0] > len(maze) - 1 or nodePosition[0] < 0 or nodePosition[1] > len(maze[0]) - 1 or nodePosition[1] < 0:
                    continue

                # 장애물이 있으면 다른 위치 불러오기
                if maze[nodePosition[0]][nodePosition[1]] != 0:
                    continue

                new_node = goalNode(currentNode, nodePosition)
                # 자식이 closedList에 있으면 continue
                if new_node.position in closedSet:
                    continue

                # f, g, h값 업데이트
                new_node.g = currentNode.g + 1
                new_node.h = self.heuristicDiagonal(new_node, endNode)
                new_node.f = new_node.g + new_node.h

                # 자식이 openList에 있으고, g값이 더 크면 continue
                # new_node.g >= child.g 로 하면 openList 중복성을 줄일 수도 있습니다.
                # 하지만 이건 시나리오에 따라 다르고, 대부분 엄격히 더 낮은 g 값을 확인하면 충분할 수 있습니다.
                if any(child for child in openList if new_node == child and new_node.g > child.g):
                    continue

                heapq.heappush(openList, new_node)

        print("while end")

