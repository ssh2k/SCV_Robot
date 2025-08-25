#ifndef PATHFINDER_H
#define PATHFINDER_H

#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <vector>
#include <queue>
#include <map>
#include <cmath>
#include <tuple>

// 노드 구조체
struct Node {
    int x, y;
    double g;  // 시작점에서 현재 노드까지의 비용
    double h;  // 현재 노드에서 목표점까지의 휴리스틱 비용
    double f;  // 총 비용 (g + h)
    Node* parent;
    // 회전 최소화를 위한 상태
    int dirX;      // 부모에서 현재로 오는 방향 벡터 X(-1,0,1)
    int dirY;      // 부모에서 현재로 오는 방향 벡터 Y(-1,0,1)
    bool hasDirection; // 시작 노드 여부 판단
    int turnCount; // 시작부터 현재까지의 회전 횟수
    
    Node(int x, int y) : x(x), y(y), g(0), h(0), f(0), parent(nullptr), dirX(0), dirY(0), hasDirection(false), turnCount(0) {}
    
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
    
    // 경로 캐시 초기화
    void clearCache();
    
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
    
    // 경로 캐시 (장애물 변경 시 무효화)
    std::map<std::tuple<int,int,int,int>, std::vector<PathPoint>> pathCache;
    
    static constexpr double kEpsilon = 1e-6;
    
    // 휴리스틱 함수 (유클리드 거리)
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
            // f 우선, f가 사실상 같으면 회전 카운트가 적은 쪽 우선
            if (fabs(a->f - b->f) > Pathfinder::kEpsilon) {
                return a->f > b->f;
            }
            if (a->turnCount != b->turnCount) {
                return a->turnCount > b->turnCount;
            }
            // 마지막으로 h가 작은 쪽 우선 (좀 더 목표에 가까운 것)
            return a->h > b->h;
        }
    };
    
    // 메모리 정리
    void cleanupNodes(std::vector<Node*>& nodes);
};

#endif
