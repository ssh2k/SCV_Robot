#pragma once
#include <vector>
#include <queue>
#include <utility> // for std::pair

struct Node {
    int x, y;
    int g, h; // g = 시작점에서 비용, h = 휴리스틱(목표까지 추정 비용)
    Node* parent;
};

class Pathfinder {
public:
    Pathfinder(int width, int height, std::vector<std::vector<int>>& map);

    // A* 경로 탐색
    std::vector<std::pair<int,int>> findPath(std::pair<int,int> start, std::pair<int,int> goal);

private:
    int width, height;
    std::vector<std::vector<int>>& gridMap; // 0=빈칸, 1=벽

    bool isValid(int x, int y);
    int heuristic(int x1, int y1, int x2, int y2);
};
