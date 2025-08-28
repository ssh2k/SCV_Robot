#include "pathfinder.h"
#include <cmath>
#include <set>
#include <algorithm>

Pathfinder::Pathfinder(int width, int height, std::vector<std::vector<int>>& map)
    : width(width), height(height), gridMap(map) {}

bool Pathfinder::isValid(int x, int y) {
    return (x >= 0 && x < width && y >= 0 && y < height && gridMap[y][x] == 0);
}

int Pathfinder::heuristic(int x1, int y1, int x2, int y2) {
    // 맨해튼 거리
    return abs(x1 - x2) + abs(y1 - y2);
}

std::vector<std::pair<int,int>> Pathfinder::findPath(std::pair<int,int> start, std::pair<int,int> goal) {
    std::vector<std::pair<int,int>> path;
    std::set<std::pair<int,int>> closedSet;
    std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));

    auto cmp = [](Node* a, Node* b) {
        return (a->g + a->h) > (b->g + b->h);
    };
    std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> openSet(cmp);

    Node* startNode = new Node{start.first, start.second, 0, heuristic(start.first, start.second, goal.first, goal.second), nullptr};
    openSet.push(startNode);

    while(!openSet.empty()) {
        Node* current = openSet.top();
        openSet.pop();

        if(current->x == goal.first && current->y == goal.second) {
            // 경로 역추적
            while(current) {
                path.push_back({current->x, current->y});
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());
            break;
        }

        visited[current->y][current->x] = true;

        // 상하좌우 이동
        int dx[4] = {0, 1, 0, -1};
        int dy[4] = {-1, 0, 1, 0};

        for(int i=0; i<4; i++) {
            int nx = current->x + dx[i];
            int ny = current->y + dy[i];

            if(isValid(nx, ny) && !visited[ny][nx]) {
                Node* neighbor = new Node{nx, ny, current->g + 1, heuristic(nx, ny, goal.first, goal.second), current};
                openSet.push(neighbor);
            }
        }
    }

    return path;
}