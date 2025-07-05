#include "pathfinder.h"

Pathfinder::Pathfinder(int width, int height) : gridWidth(width), gridHeight(height) {
    // 2차원 배열 동적 할당
    grid = new bool*[gridHeight];
    for (int i = 0; i < gridHeight; i++) {
        grid[i] = new bool[gridWidth];
        for (int j = 0; j < gridWidth; j++) {
            grid[i][j] = false;  // 기본적으로 장애물 없음
        }
    }
}

Pathfinder::~Pathfinder() {
    // 메모리 해제
    for (int i = 0; i < gridHeight; i++) {
        delete[] grid[i];
    }
    delete[] grid;
}

void Pathfinder::begin() {
    Serial.println("[Pathfinder] Initializing pathfinding system...");
    Serial.print("[Pathfinder] Grid size: ");
    Serial.print(gridWidth);
    Serial.print(" x ");
    Serial.println(gridHeight);
}

std::vector<PathPoint> Pathfinder::findPath(int startX, int startY, int goalX, int goalY) {
    Serial.println("[Pathfinder] Starting A* pathfinding...");
    Serial.print("[Pathfinder] Start: (");
    Serial.print(startX);
    Serial.print(", ");
    Serial.print(startY);
    Serial.print("), Goal: (");
    Serial.print(goalX);
    Serial.print(", ");
    Serial.print(goalY);
    Serial.println(")");
    
    // 시작점과 목표점이 유효한지 확인
    if (!isValid(startX, startY) || !isValid(goalX, goalY)) {
        Serial.println("[Pathfinder] Invalid start or goal position");
        return std::vector<PathPoint>();
    }
    
    // 시작점이 장애물인지 확인
    if (isObstacle(startX, startY) || isObstacle(goalX, goalY)) {
        Serial.println("[Pathfinder] Start or goal position is blocked");
        return std::vector<PathPoint>();
    }
    
    // 오픈 리스트 (우선순위 큐)
    std::priority_queue<Node*, std::vector<Node*>, NodeCompare> openList;
    
    // 클로즈드 리스트 (방문한 노드들)
    std::vector<Node*> closedList;
    
    // 시작 노드 생성
    Node* startNode = new Node(startX, startY);
    startNode->h = heuristic(startX, startY, goalX, goalY);
    startNode->f = startNode->h;
    
    openList.push(startNode);
    
    Node* goalNode = nullptr;
    
    while (!openList.empty()) {
        // f값이 가장 작은 노드 선택
        Node* currentNode = openList.top();
        openList.pop();
        
        // 목표점에 도달했는지 확인
        if (currentNode->x == goalX && currentNode->y == goalY) {
            goalNode = currentNode;
            break;
        }
        
        // 클로즈드 리스트에 추가
        closedList.push_back(currentNode);
        
        // 이웃 노드들 확인
        std::vector<Node> neighbors = getNeighbors(*currentNode);
        
        for (const Node& neighbor : neighbors) {
            // 이미 방문한 노드인지 확인
            bool inClosedList = false;
            for (Node* closedNode : closedList) {
                if (closedNode->x == neighbor.x && closedNode->y == neighbor.y) {
                    inClosedList = true;
                    break;
                }
            }
            
            if (inClosedList) continue;
            
            // 새로운 비용 계산
            double newG = currentNode->g + 1.0;  // 대각선 이동의 경우 1.414
            
            // 오픈 리스트에서 이웃 노드 찾기
            bool inOpenList = false;
            Node* existingNode = nullptr;
            
            // 오픈 리스트를 복사하여 확인 (우선순위 큐는 직접 검색이 어려움)
            std::priority_queue<Node*, std::vector<Node*>, NodeCompare> tempQueue = openList;
            while (!tempQueue.empty()) {
                Node* tempNode = tempQueue.top();
                tempQueue.pop();
                if (tempNode->x == neighbor.x && tempNode->y == neighbor.y) {
                    inOpenList = true;
                    existingNode = tempNode;
                    break;
                }
            }
            
            if (!inOpenList) {
                // 새로운 노드 생성
                Node* newNode = new Node(neighbor.x, neighbor.y);
                newNode->parent = currentNode;
                newNode->g = newG;
                newNode->h = heuristic(neighbor.x, neighbor.y, goalX, goalY);
                newNode->f = newG + newNode->h;
                openList.push(newNode);
            } else if (newG < existingNode->g) {
                // 더 나은 경로를 찾았을 때 업데이트
                existingNode->parent = currentNode;
                existingNode->g = newG;
                existingNode->f = newG + existingNode->h;
            }
        }
    }
    
    // 경로 재구성
    std::vector<PathPoint> path;
    if (goalNode) {
        path = reconstructPath(goalNode);
        Serial.print("[Pathfinder] Path found with ");
        Serial.print(path.size());
        Serial.println(" points");
    } else {
        Serial.println("[Pathfinder] No path found");
    }
    
    // 메모리 정리
    cleanupNodes(closedList);
    
    // 오픈 리스트 정리
    while (!openList.empty()) {
        delete openList.top();
        openList.pop();
    }
    
    return path;
}

void Pathfinder::setObstacle(int x, int y, bool isObstacle) {
    if (isValid(x, y)) {
        grid[y][x] = isObstacle;
    }
}

bool Pathfinder::isObstacle(int x, int y) {
    if (!isValid(x, y)) return true;
    return grid[y][x];
}

std::vector<PathPoint> Pathfinder::optimizePath(const std::vector<PathPoint>& path) {
    if (path.size() <= 2) return path;
    
    std::vector<PathPoint> optimizedPath;
    optimizedPath.push_back(path[0]);
    
    for (size_t i = 1; i < path.size() - 1; i++) {
        // 현재 점에서 다음 점까지 직선 경로가 가능한지 확인
        bool canSkip = true;
        int dx = path[i + 1].x - path[i - 1].x;
        int dy = path[i + 1].y - path[i - 1].y;
        
        // 직선 경로상의 모든 점이 장애물이 아닌지 확인
        int steps = max(abs(dx), abs(dy));
        for (int step = 1; step < steps; step++) {
            int x = path[i - 1].x + (dx * step) / steps;
            int y = path[i - 1].y + (dy * step) / steps;
            if (isObstacle(x, y)) {
                canSkip = false;
                break;
            }
        }
        
        if (!canSkip) {
            optimizedPath.push_back(path[i]);
        }
    }
    
    optimizedPath.push_back(path.back());
    
    Serial.print("[Pathfinder] Path optimized: ");
    Serial.print(path.size());
    Serial.print(" -> ");
    Serial.println(optimizedPath.size());
    
    return optimizedPath;
}

void Pathfinder::printPath(const std::vector<PathPoint>& path) {
    Serial.println("[Pathfinder] Path:");
    for (size_t i = 0; i < path.size(); i++) {
        Serial.print("  ");
        Serial.print(i);
        Serial.print(": (");
        Serial.print(path[i].x);
        Serial.print(", ");
        Serial.print(path[i].y);
        Serial.println(")");
    }
}

double Pathfinder::heuristic(int x1, int y1, int x2, int y2) {
    // 맨해튼 거리 사용
    return abs(x2 - x1) + abs(y2 - y1);
}

bool Pathfinder::isValid(int x, int y) {
    return x >= 0 && x < gridWidth && y >= 0 && y < gridHeight;
}

std::vector<Node> Pathfinder::getNeighbors(const Node& node) {
    std::vector<Node> neighbors;
    
    // 8방향 이웃 확인
    int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    
    for (int i = 0; i < 8; i++) {
        int newX = node.x + dx[i];
        int newY = node.y + dy[i];
        
        if (isValid(newX, newY) && !isObstacle(newX, newY)) {
            neighbors.push_back(Node(newX, newY));
        }
    }
    
    return neighbors;
}

std::vector<PathPoint> Pathfinder::reconstructPath(Node* goalNode) {
    std::vector<PathPoint> path;
    Node* current = goalNode;
    
    while (current != nullptr) {
        path.insert(path.begin(), PathPoint(current->x, current->y));
        current = current->parent;
    }
    
    return path;
}

void Pathfinder::cleanupNodes(std::vector<Node*>& nodes) {
    for (Node* node : nodes) {
        delete node;
    }
    nodes.clear();
}
