#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <algorithm>
#include <climits>
#include <set>
#include <map>
#ifdef _WIN32
#include <windows.h>
#endif
using namespace std;

class Graph {
private:
    vector<vector<int>> adjMatrix;  // adjacency matrix
    vector<vector<pair<int, int>>> adjList;  // adjacency list
    map<char, int> vertexToIndex;  // map vertex char to index
    map<int, char> indexToVertex;  // map index to vertex char
    int vertexCount;

public:
    // 构造函数
    Graph(int n, const vector<char>& vertices) : vertexCount(n) {
        adjMatrix.resize(n, vector<int>(n, 0));
        adjList.resize(n);
        
        for (int i = 0; i < n; i++) {
            vertexToIndex[vertices[i]] = i;
            indexToVertex[i] = vertices[i];
        }
    }

    // 添加边
    void addEdge(char u, char v, int weight) {
        int uIndex = vertexToIndex[u];
        int vIndex = vertexToIndex[v];
        
        // 更新邻接矩阵
        adjMatrix[uIndex][vIndex] = weight;
        adjMatrix[vIndex][uIndex] = weight;
        
        // 更新邻接表
        adjList[uIndex].push_back({vIndex, weight});
        adjList[vIndex].push_back({uIndex, weight});
    }

    // (1) 输出邻接矩阵
    void printAdjacencyMatrix() {
        cout << "邻接矩阵:" << endl;
        cout << "  ";
        for (int i = 0; i < vertexCount; i++) {
            cout << indexToVertex[i] << " ";
        }
        cout << endl;
        
        for (int i = 0; i < vertexCount; i++) {
            cout << indexToVertex[i] << " ";
            for (int j = 0; j < vertexCount; j++) {
                cout << adjMatrix[i][j] << " ";
            }
            cout << endl;
        }
    }

    // (2) BFS算法
    void BFS(char start) {
        vector<bool> visited(vertexCount, false);
        queue<int> q;
        int startIndex = vertexToIndex[start];
        
        cout << "BFS遍历顺序: ";
        visited[startIndex] = true;
        q.push(startIndex);
        
        while (!q.empty()) {
            int current = q.front();
            q.pop();
            cout << indexToVertex[current] << " ";
            
            for (const auto& neighbor : adjList[current]) {
                int neighborIndex = neighbor.first;
                if (!visited[neighborIndex]) {
                    visited[neighborIndex] = true;
                    q.push(neighborIndex);
                }
            }
        }
        cout << endl;
    }

    // (2) DFS算法
    void DFS(char start) {
        vector<bool> visited(vertexCount, false);
        stack<int> s;
        int startIndex = vertexToIndex[start];
        
        cout << "DFS遍历顺序: ";
        s.push(startIndex);
        
        while (!s.empty()) {
            int current = s.top();
            s.pop();
            
            if (!visited[current]) {
                visited[current] = true;
                cout << indexToVertex[current] << " ";
                
                // 逆序入栈以保证按字母顺序遍历
                for (auto it = adjList[current].rbegin(); it != adjList[current].rend(); ++it) {
                    if (!visited[it->first]) {
                        s.push(it->first);
                    }
                }
            }
        }
        cout << endl;
    }

    // (3) Dijkstra最短路径算法
    void shortestPath(char start) {
        int startIndex = vertexToIndex[start];
        vector<int> dist(vertexCount, INT_MAX);
        vector<int> parent(vertexCount, -1);
        vector<bool> visited(vertexCount, false);
        
        dist[startIndex] = 0;
        
        for (int i = 0; i < vertexCount; i++) {
            // 找到未访问的最小距离顶点
            int minDist = INT_MAX, minIndex = -1;
            for (int j = 0; j < vertexCount; j++) {
                if (!visited[j] && dist[j] < minDist) {
                    minDist = dist[j];
                    minIndex = j;
                }
            }
            
            if (minIndex == -1) break;
            visited[minIndex] = true;
            
            // 更新邻居距离
            for (const auto& neighbor : adjList[minIndex]) {
                int v = neighbor.first;
                int weight = neighbor.second;
                if (!visited[v] && dist[minIndex] + weight < dist[v]) {
                    dist[v] = dist[minIndex] + weight;
                    parent[v] = minIndex;
                }
            }
        }
        
        // 输出最短路径
        cout << "从 " << start << " 出发的最短路径:" << endl;
        for (int i = 0; i < vertexCount; i++) {
            if (i != startIndex) {
                cout << "到 " << indexToVertex[i] << ": 距离=" << dist[i] << ", 路径=";
                printPath(startIndex, i, parent);
                cout << endl;
            }
        }
    }

    // (3) Prim最小支撑树算法
    void minimumSpanningTree(char start) {
        int startIndex = vertexToIndex[start];
        vector<int> key(vertexCount, INT_MAX);
        vector<int> parent(vertexCount, -1);
        vector<bool> inMST(vertexCount, false);
        
        key[startIndex] = 0;
        
        for (int i = 0; i < vertexCount; i++) {
            // 找到不在MST中的最小key顶点
            int minKey = INT_MAX, minIndex = -1;
            for (int j = 0; j < vertexCount; j++) {
                if (!inMST[j] && key[j] < minKey) {
                    minKey = key[j];
                    minIndex = j;
                }
            }
            
            if (minIndex == -1) break;
            inMST[minIndex] = true;
            
            // 更新邻居的key值
            for (const auto& neighbor : adjList[minIndex]) {
                int v = neighbor.first;
                int weight = neighbor.second;
                if (!inMST[v] && weight < key[v]) {
                    key[v] = weight;
                    parent[v] = minIndex;
                }
            }
        }
        
        // 输出最小支撑树
        cout << "最小支撑树边:" << endl;
        int totalWeight = 0;
        for (int i = 0; i < vertexCount; i++) {
            if (parent[i] != -1) {
                cout << indexToVertex[parent[i]] << " - " << indexToVertex[i] 
                     << " (权重: " << adjMatrix[parent[i]][i] << ")" << endl;
                totalWeight += adjMatrix[parent[i]][i];
            }
        }
        cout << "总权重: " << totalWeight << endl;
    }

    // (4) Tarjan算法求双连通分量和关节点
    void findBiconnectedComponents() {
        vector<int> disc(vertexCount, -1);
        vector<int> low(vertexCount, -1);
        vector<int> parent(vertexCount, -1);
        vector<bool> articulation(vertexCount, false);
        stack<pair<int, int>> st;
        int time = 0;
        
        for (int i = 0; i < vertexCount; i++) {
            if (disc[i] == -1) {
                tarjanDFS(i, disc, low, parent, articulation, st, time);
            }
        }
        
        // 输出关节点
        cout << "关节点: ";
        bool hasArticulation = false;
        for (int i = 0; i < vertexCount; i++) {
            if (articulation[i]) {
                cout << indexToVertex[i] << " ";
                hasArticulation = true;
            }
        }
        if (!hasArticulation) {
            cout << "无";
        }
        cout << endl;
    }

private:
    // 辅助函数：打印路径
    void printPath(int start, int end, const vector<int>& parent) {
        if (end == start) {
            cout << indexToVertex[start];
        } else if (parent[end] == -1) {
            cout << "无路径";
        } else {
            printPath(start, parent[end], parent);
            cout << " -> " << indexToVertex[end];
        }
    }

    // Tarjan算法DFS
    void tarjanDFS(int u, vector<int>& disc, vector<int>& low, vector<int>& parent, 
                   vector<bool>& articulation, stack<pair<int, int>>& st, int& time) {
        disc[u] = low[u] = ++time;
        int children = 0;
        
        for (const auto& neighbor : adjList[u]) {
            int v = neighbor.first;
            
            if (disc[v] == -1) {
                children++;
                parent[v] = u;
                st.push({u, v});
                
                tarjanDFS(v, disc, low, parent, articulation, st, time);
                
                low[u] = min(low[u], low[v]);
                
                // 检查u是否是关节点
                if ((parent[u] == -1 && children > 1) || (parent[u] != -1 && low[v] >= disc[u])) {
                    articulation[u] = true;
                    // 输出双连通分量
                    cout << "双连通分量: ";
                    while (!st.empty() && st.top() != make_pair(u, v)) {
                        auto edge = st.top();
                        st.pop();
                        cout << "(" << indexToVertex[edge.first] << "-" << indexToVertex[edge.second] << ") ";
                    }
                    if (!st.empty()) {
                        auto edge = st.top();
                        st.pop();
                        cout << "(" << indexToVertex[edge.first] << "-" << indexToVertex[edge.second] << ") ";
                    }
                    cout << endl;
                }
            } else if (v != parent[u] && disc[v] < disc[u]) {
                low[u] = min(low[u], disc[v]);
                st.push({u, v});
            }
        }
    }
};

// 创建图1
Graph createGraph1() {
    vector<char> vertices = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'};
    Graph g(vertices.size(), vertices);
    
    // 添加边 (根据图1的结构)
    g.addEdge('A', 'B', 4);
    g.addEdge('A', 'D', 6);
    g.addEdge('A', 'G', 7);
    g.addEdge('B', 'C', 12);
    g.addEdge('C', 'D', 9);
    g.addEdge('C', 'E', 1);
    g.addEdge('C', 'F', 2);
    g.addEdge('D', 'G', 2);
    g.addEdge('D', 'E', 13);
    g.addEdge('E', 'F', 5);
    g.addEdge('E', 'H', 8);
    g.addEdge('E', 'G', 11);
    g.addEdge('F', 'H', 3);
    g.addEdge('G', 'H', 14);
    
    return g;
}

// 创建图2
Graph createGraph2() {
    vector<char> vertices = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L'};
    Graph g(vertices.size(), vertices);
    
    // 添加边 (根据图2的结构)
    g.addEdge('A', 'B', 1);
    g.addEdge('A', 'E', 1);
    g.addEdge('B', 'F', 1);
    g.addEdge('C', 'D', 1);
    g.addEdge('C', 'F', 1);
    g.addEdge('C', 'H', 1);
    g.addEdge('D', 'H', 1);
    g.addEdge('E', 'F', 1);
    g.addEdge('E', 'I', 1);
    g.addEdge('F', 'G', 1);
    g.addEdge('F', 'K', 1);
    g.addEdge('F', 'J', 1);
    g.addEdge('F', 'I', 1);
    g.addEdge('G', 'K', 1);
    g.addEdge('J', 'K', 1);
    g.addEdge('K', 'L', 1);
    
    return g;
}

int main() {
    #ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
    #endif
    cout << "=== 图1分析 ===" << endl;
    Graph g1 = createGraph1();
    g1.printAdjacencyMatrix();
    cout << endl;
    
    cout << "从A点出发:" << endl;
    g1.BFS('A');
    g1.DFS('A');
    cout << endl;
    
    g1.shortestPath('A');
    cout << endl;
    
    g1.minimumSpanningTree('A');
    cout << endl;
    
    cout << "=== 图2分析 ===" << endl;
    Graph g2 = createGraph2();
    
    // 测试不同起点的双连通分量
    vector<char> testStarts = {'A', 'D', 'G'};
    for (char start : testStarts) {
        cout << "起点: " << start << endl;
        Graph g2_test = createGraph2();  // 重新创建图以避免状态污染
        g2_test.findBiconnectedComponents();
        cout << endl;
    }
    
    return 0;
}