#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <Arduino.h>
#include <vector>
#include <queue>
#include <map>
#include <cmath>

// 노드 구조체
struct Node {
    int x, y;
    double g;  // 시작점에서 현재 노드까지의 비용
    double h;  // 현재 노드에서 목표점까지의 휴리스틱 비용
    double f;  // 총 비용 (g + h)
    Node* parent;
    
    Node(int x, int y) : x(x), y(y), g(0), h(0), f(0), parent(nullptr) {}
    
    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

// 경로점 구조체
struct PathPoint {
    int x, y;
    PathPoint(int x, int y) : x(x), y(y) {}
};

class Pathfinder {
public:
    Pathfinder(int gridWidth, int gridHeight);
    ~Pathfinder();
    
    // 초기화
    void begin();
    
    // 경로 찾기
    std::vector<PathPoint> findPath(int startX, int startY, int goalX, int goalY);
    
    // 장애물 설정/해제
    void setObstacle(int x, int y, bool isObstacle);
    bool isObstacle(int x, int y);
    
    // 그리드 크기 반환
    int getGridWidth() const { return gridWidth; }
    int getGridHeight() const { return gridHeight; }
    
    // 경로 최적화
    std::vector<PathPoint> optimizePath(const std::vector<PathPoint>& path);
    
    // 경로 출력 (디버깅용)
    void printPath(const std::vector<PathPoint>& path);

private:
    int gridWidth, gridHeight;
    bool** grid;  // true = 장애물, false = 통행 가능
    
    // 휴리스틱 함수 (맨해튼 거리)
    double heuristic(int x1, int y1, int x2, int y2);
    
    // 노드가 유효한지 확인
    bool isValid(int x, int y);
    
    // 이웃 노드들 반환
    std::vector<Node> getNeighbors(const Node& node);
    
    // 경로 재구성
    std::vector<PathPoint> reconstructPath(Node* goalNode);
    
    // 노드 비교 함수
    struct NodeCompare {
        bool operator()(const Node* a, const Node* b) const {
            return a->f > b->f;
        }
    };
    
    // 메모리 정리
    void cleanupNodes(std::vector<Node*>& nodes);
};

#endif
