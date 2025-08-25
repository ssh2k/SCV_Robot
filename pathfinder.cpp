#include "pathfinder.h"
#include "utils.h"
#include <algorithm>

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
#ifdef ARDUINO
    Serial.println("[Pathfinder] Initializing pathfinding system...");
    Serial.print("[Pathfinder] Grid size: ");
    Serial.print(gridWidth);
    Serial.print(" x ");
    Serial.println(gridHeight);
#endif
}

std::vector<PathPoint> Pathfinder::findPath(int startX, int startY, int goalX, int goalY) {
    // 캐시 조회
    auto cacheKey = std::make_tuple(startX, startY, goalX, goalY);
    auto cacheIt = pathCache.find(cacheKey);
    if (cacheIt != pathCache.end()) {
        return cacheIt->second;
    }
    
    // 시작점과 목표점이 유효한지 확인
    if (!isValid(startX, startY) || !isValid(goalX, goalY)) {
        return std::vector<PathPoint>();
    }
    
    // 시작점이 장애물인지 확인
    if (isObstacle(startX, startY) || isObstacle(goalX, goalY)) {
        return std::vector<PathPoint>();
    }
    
    // 오픈 리스트 (우선순위 큐)
    std::priority_queue<Node*, std::vector<Node*>, NodeCompare> openList;
    // 방문 상태 및 최선 g/회전 저장
    std::map<std::pair<int,int>, std::pair<double,int>> bestState; // {g, turnCount}
    // 메모리 정리를 위한 노드 소유 컨테이너
    std::vector<Node*> allNodes;
    
    // 시작 노드 생성
    Node* startNode = new Node(startX, startY);
    startNode->h = heuristic(startX, startY, goalX, goalY);
    startNode->f = startNode->h;
    startNode->hasDirection = false;
    startNode->turnCount = 0;
    
    openList.push(startNode);
    allNodes.push_back(startNode);
    bestState[std::make_pair(startX, startY)] = std::make_pair(0.0, 0);
    
    Node* goalNode = nullptr;
    
    while (!openList.empty()) {
        // f값이 가장 작은 노드 선택
        Node* currentNode = openList.top();
        openList.pop();
        
        // 스테일 노드 스킵: 현재 노드가 그 좌표의 최선 상태가 아니면 건너뜀
        auto bsIt = bestState.find(std::make_pair(currentNode->x, currentNode->y));
        if (bsIt != bestState.end()) {
            double bestG = bsIt->second.first;
            int bestTurns = bsIt->second.second;
            if (currentNode->g - bestG > kEpsilon ||
                (fabs(currentNode->g - bestG) <= kEpsilon && currentNode->turnCount > bestTurns)) {
                continue;
            }
        }
        
        // 목표점에 도달했는지 확인
        if (currentNode->x == goalX && currentNode->y == goalY) {
            goalNode = currentNode;
            break;
        }
        
        // 이웃 노드들 확인
        std::vector<Node> neighbors = getNeighbors(*currentNode);
        
        for (const Node& neighbor : neighbors) {
            // 이동 비용 계산 (수정된 부분)
            double moveCost = 1.0;
            // 대각선 이동인지 확인
            if (abs(neighbor.x - currentNode->x) == 1 && abs(neighbor.y - currentNode->y) == 1) {
                moveCost = 1.414; // √2
            }
            double newG = currentNode->g + moveCost;
            
            // 방향 및 회전 계산
            int stepDX = neighbor.x - currentNode->x;
            int stepDY = neighbor.y - currentNode->y;
            if (stepDX != 0) stepDX = (stepDX > 0) ? 1 : -1;
            if (stepDY != 0) stepDY = (stepDY > 0) ? 1 : -1;
            bool isTurn = currentNode->hasDirection && (currentNode->dirX != stepDX || currentNode->dirY != stepDY);
            int newTurnCount = currentNode->turnCount + (isTurn ? 1 : 0);

            // 베스트 상태와 비교하여 열 필요 여부 결정
            auto key = std::make_pair(neighbor.x, neighbor.y);
            auto bestIt = bestState.find(key);
            bool shouldPush = false;
            if (bestIt == bestState.end()) {
                shouldPush = true;
            } else {
                double bestG = bestIt->second.first;
                int bestTurns = bestIt->second.second;
                if (newG + kEpsilon < bestG ||
                    (fabs(newG - bestG) <= kEpsilon && newTurnCount < bestTurns)) {
                    shouldPush = true;
                }
            }
            if (shouldPush) {
                Node* newNode = new Node(neighbor.x, neighbor.y);
                newNode->parent = currentNode;
                newNode->g = newG;
                newNode->h = heuristic(neighbor.x, neighbor.y, goalX, goalY);
                newNode->f = newG + newNode->h;
                newNode->dirX = stepDX;
                newNode->dirY = stepDY;
                newNode->hasDirection = true;
                newNode->turnCount = newTurnCount;
                openList.push(newNode);
                allNodes.push_back(newNode);
                bestState[key] = std::make_pair(newG, newTurnCount);
            }
        }
    }
    
    // 경로 재구성
    std::vector<PathPoint> path;
    if (goalNode) {
        path = reconstructPath(goalNode);
    }
    
    // 메모리 정리: 생성한 모든 노드 해제
    cleanupNodes(allNodes);
    
    // 경로 캐시에 저장
    pathCache[cacheKey] = path;
    
    return path;
}

void Pathfinder::setObstacle(int x, int y, bool isObstacle) {
    if (isValid(x, y)) {
        grid[y][x] = isObstacle;
        // 장애물 변경 시 캐시 무효화
        clearCache();
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
        int steps = std::max(abs(dx), abs(dy));
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
    
    // 회전 최소화 관점에서 추가 개선 여지 있음
    return optimizedPath;
}

void Pathfinder::printPath(const std::vector<PathPoint>& path) {
#ifdef ARDUINO
    Serial.println("[Pathfinder] Path:");
    for (size_t i = 0; i < path.size(); i++) {
        Serial.print("  ");
        Serial.print((int)i);
        Serial.print(": (");
        Serial.print(path[i].x);
        Serial.print(", ");
        Serial.print(path[i].y);
        Serial.println(")");
    }
#endif
}

double Pathfinder::heuristic(int x1, int y1, int x2, int y2) {
    // utils의 calculateDistance 함수 사용
    return calculateDistance(x1, y1, x2, y2);
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

void Pathfinder::clearCache() {
    pathCache.clear();
}
