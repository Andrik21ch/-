#include <unordered_map>
#include <vector>
#include <iostream>
#include <queue>
#include <limits.h>

using namespace std;

class Vertex {
public:
    int id;  // Уникальный идентификатор вершины

    // Конструктор по умолчанию
    Vertex() : id(0) {}

    // Конструктор для инициализации id
    Vertex(int id) : id(id) {}
};

class Edge {
public:
    int source;      // Источник
    int destination; // Приемник
    int weight;      // Вес ребра

    // Конструктор для инициализации ребра
    Edge(int source, int destination, int weight)
        : source(source), destination(destination), weight(weight) {}
};

class Graph {
public:
    unordered_map<int, Vertex> vertices;                      // Словарь для хранения вершин
    unordered_map<int, vector<Edge>> adjacencyList;        // Список смежности для хранения рёбер
    bool isUndirected; // Флаг, указывающий, является ли граф неориентированным

    // Конструктор по умолчанию
    Graph(bool isUndirected = true) : isUndirected(isUndirected) {}

    // Метод для получения списка смежности (по ключу вершины)
    unordered_map<int, vector<Edge>> getAdjacencyList() const {
        cout << "Вызван метод: getAdjacencyList\n";
        return adjacencyList;
    }

    // Метод для добавления вершины
    void addVertex(const Vertex& vertex) {
        cout << "Вызван метод: addVertex\n";
        vertices[vertex.id] = vertex;
        cout << "Вершина с id " << vertex.id << " добавлена в граф.\n";
    }

    // Метод для добавления ребра
    void addEdge(int source, int destination, int weight) {
        cout << "Вызван метод: addEdge\n";
        adjacencyList[source].push_back(Edge(source, destination, weight));
        if (isUndirected) {
            adjacencyList[destination].push_back(Edge(destination, source, weight)); // Для неориентированного графа
        }
        cout << "Добавлено ребро от вершины " << source << " к вершине " << destination << " с весом " << weight << ".\n";
    }

    // Получение соседей для вершины
    vector<Edge> getNeighbors(int vertex) const {
        cout << "Вызван метод: getNeighbors\n";
        return adjacencyList.at(vertex);
    }

    // Проверка на наличие ребра
    bool hasEdge(int source, int destination) const {
        cout << "Вызван метод: hasEdge\n";
        for (const auto& edge : adjacencyList.at(source)) {
            if (edge.destination == destination) {
                return true;
            }
        }
        return false;
    }

    // Вывод графа
    void printGraph() const {
        cout << "Вызван метод: printGraph\n";
        for (const auto& pair : adjacencyList) {
            cout << "Вершина " << pair.first << " имеет рёбра к: ";
            for (const auto& edge : pair.second) {
                cout << edge.destination << " (вес " << edge.weight << "), ";
            }
            cout << endl;
        }
    }

    void fordBellman(int start) {
        cout << "Вызван метод: fordBellman от вершины " << start << "\n";

        // Количество вершин
        int V = vertices.size();

        // Вектор расстояний до вершин
        unordered_map<int, int> distances;

        // Инициализация расстояний: бесконечность для всех вершин, кроме стартовой
        for (const auto& vertex : vertices) {
            distances[vertex.first] = INT_MAX;
        }
        distances[start] = 0;

        // Основной цикл алгоритма
        for (int i = 0; i < V - 1; i++) {
            for (const auto& pair : adjacencyList) {
                for (const auto& edge : pair.second) {
                    if (distances[edge.source] != INT_MAX &&
                        distances[edge.source] + edge.weight < distances[edge.destination]) {
                        distances[edge.destination] = distances[edge.source] + edge.weight;
                    }
                }
            }
        }

        // Проверка на наличие отрицательных циклов
        for (const auto& pair : adjacencyList) {
            for (const auto& edge : pair.second) {
                if (distances[edge.source] != INT_MAX &&
                    distances[edge.source] + edge.weight < distances[edge.destination]) {
                    cout << "Граф содержит отрицательный цикл.\n";
                    return;
                }
            }
        }

        // Вывод кратчайших расстояний от стартовой вершины
        cout << "Кратчайшие расстояния от вершины " << start << " (по Форду-Беллману):\n";
        for (const auto& pair : distances) {
            if (pair.second == INT_MAX) {
                cout << "До вершины " << pair.first << ": бесконечность.\n";
            }
            else {
                cout << "До вершины " << pair.first << ": " << pair.second << ".\n";
            }
        }
    }


    // Алгоритм Дейкстры для поиска кратчайших путей от вершины start
    void dijkstra(int start) {
        cout << "Вызван метод: dijkstra от вершины " << start << "\n";
        unordered_map<int, int> distances; // Словарь расстояний
        for (const auto& vertex : vertices) {
            distances[vertex.first] = INT_MAX; // Изначально расстояния бесконечны
        }
        distances[start] = 0; // Расстояние до стартовой вершины = 0

        using pii = pair<int, int>; // (расстояние, вершина)
        priority_queue<pii, vector<pii>, greater<pii>> pq; // Мин-кучка
        pq.push({ 0, start });

        while (!pq.empty()) {
            int current = pq.top().second;
            int currentDistance = pq.top().first;
            pq.pop();

            // Если уже найдено более короткое расстояние, пропускаем
            if (currentDistance > distances[current]) {
                continue;
            }

            // Обходим соседей
            for (const auto& edge : getNeighbors(current)) {
                int newDist = currentDistance + edge.weight;
                if (newDist < distances[edge.destination]) {
                    distances[edge.destination] = newDist;
                    pq.push({ newDist, edge.destination });
                }
            }
        }

        // Выводим все кратчайшие расстояния от start
        cout << "Кратчайшие расстояния от вершины " << start << ":\n";
        for (const auto& pair : distances) {
            cout << "До вершины " << pair.first << " расстояние: " << pair.second << ".\n";
        }
    }

    // Алгоритм Флойда для поиска кратчайших путей между всеми вершинами
    void floydWarshall() {
        cout << "Вызван метод: floydWarshall\n";
        // Количество вершин
        int V = vertices.size();

        // Матрица расстояний
        vector<vector<int>> dist(V, vector<int>(V, INT_MAX));

        // Инициализация матрицы расстояний
        for (int i = 0; i < V; i++) {
            dist[i][i] = 0; // Расстояние до самой себя = 0
        }

        // Заполнение матрицы по рёбрам
        for (const auto& pair : adjacencyList) {
            for (const auto& edge : pair.second) {
                dist[edge.source][edge.destination] = edge.weight;
                if (isUndirected) {
                    dist[edge.destination][edge.source] = edge.weight; // Для неориентированного графа
                }
            }
        }

        // Алгоритм Флойда
        for (int k = 0; k < V; k++) {
            for (int i = 0; i < V; i++) {
                for (int j = 0; j < V; j++) {
                    if (dist[i][k] != INT_MAX && dist[k][j] != INT_MAX &&
                        dist[i][j] > dist[i][k] + dist[k][j]) {
                        dist[i][j] = dist[i][k] + dist[k][j];
                    }
                }
            }
        }

        // Вывод результата
        cout << "Матрица кратчайших путей (по алгоритму Флойда):\n";
        for (int i = 0; i < V; i++) {
            for (int j = 0; j < V; j++) {
                if (dist[i][j] == INT_MAX) {
                    cout << "INF ";
                }
                else {
                    cout << dist[i][j] << " ";
                }
            }
            cout << endl;
        }
    }

    // Поиск в ширину для нахождения кратчайших путей в графе с единичными весами
    void bfs(int start) {
        cout << "Вызван метод: bfs от вершины " << start << "\n";
        unordered_map<int, int> distances; // Словарь расстояний
        for (const auto& vertex : vertices) {
            distances[vertex.first] = INT_MAX; // Изначально расстояния бесконечны
        }
        distances[start] = 0; // Расстояние до стартовой вершины = 0

        queue<int> q;
        q.push(start);

        while (!q.empty()) {
            int current = q.front();
            q.pop();

            // Обходим соседей
            for (const auto& edge : getNeighbors(current)) {
                if (distances[edge.destination] == INT_MAX) {
                    distances[edge.destination] = distances[current] + 1; // Все рёбра имеют вес 1
                    q.push(edge.destination);
                }
            }
        }

        // Выводим все кратчайшие расстояния от start
        cout << "Кратчайшие расстояния от вершины " << start << " (по BFS):\n";
        for (const auto& pair : distances) {
            cout << "До вершины " << pair.first << " расстояние: " << pair.second << ".\n";
        }
    }
};


int main() {
    setlocale(LC_ALL, "Rus");
    Graph graph(true); // Ориентированный граф (false для ориентированного)
    cout << "\n";
    // Добавление вершин
    graph.addVertex(Vertex(0));
    graph.addVertex(Vertex(1));
    graph.addVertex(Vertex(2));
    graph.addVertex(Vertex(3));
    graph.addVertex(Vertex(4));
    cout << "\n";
    // Добавление рёбер с весами
    graph.addEdge(0, 1, 7);  // Ребро от вершины 0 к вершине 1 с весом 7
    graph.addEdge(0, 3, 2);  // Ребро от вершины 0 к вершине 3 с весом 2
    graph.addEdge(1, 2, 5);  // Ребро от вершины 1 к вершине 2 с весом 5
    graph.addEdge(1, 4, 4);  // Ребро от вершины 1 к вершине 4 с весом 4
    graph.addEdge(2, 4, 1);  // Ребро от вершины 2 к вершине 4 с весом 1
    graph.addEdge(3, 4, 5);  // Ребро от вершины 3 к вершине 4 с весом 5
    cout << "\n";
    // Вывод графа
    graph.printGraph();
    cout << "\n";
    // Алгоритм Дейкстры для всех вершин
    for (const auto& vertex : graph.vertices) {
        graph.dijkstra(vertex.first);  // Вызываем Дейкстру для каждой вершины
    }

    cout << "\n";
    // Алгоритм Флойда
    graph.floydWarshall();
    cout << "\n";
    // Поиск в ширину для ориентированного графа (с единичным весом рёбер)
    graph.bfs(0); // Запуск BFS от вершины 0

    Graph secondGraph(false); // Создаём ориентированный граф (false)
    cout << "\n";
    // Добавление вершин
    secondGraph.addVertex(Vertex(0));
    secondGraph.addVertex(Vertex(1));
    secondGraph.addVertex(Vertex(2));
    secondGraph.addVertex(Vertex(3));
    cout << "\n";
    // Добавление рёбер с весами
    secondGraph.addEdge(0, 1, 5);  // Ребро от 0 к 1 с весом 5
    secondGraph.addEdge(1, 3, 3);  // Ребро от 1 к 3 с весом 3
    secondGraph.addEdge(1, 2, 5);  // Ребро от 1 к 2 с весом 5
    secondGraph.addEdge(3, 2, -5); // Ребро от 3 к 2 с весом -5
    secondGraph.addEdge(3, 0, 2);  // Ребро от 3 к 0 с весом 2
    secondGraph.addEdge(2, 0, -3); // Ребро от 2 к 0 с весом -3
    cout << "\n";
    // Запуск алгоритма Флойда-Уоршелла
    secondGraph.floydWarshall();
    cout << "\n";
    secondGraph.fordBellman(0);

    // Извлекаем список рёбер для вершины 0
    vector<Edge> neighborsEdges = secondGraph.getAdjacencyList()[3];

    // Вывод соседей
    cout << "Соседи для вершины 3: ";
    for (const Edge& edge : neighborsEdges) {
        cout << edge.destination << " ";  // Печатаем только вершины
    }
    cout << endl;

    cout << "Проверка наличия ребра между вершинами 0 и 1: ";
    if (graph.hasEdge(0, 1)) {
        cout << "Ребро существует.\n";
    }
    else {
        cout << "Ребра нет.\n";
    }


    return 0;
}
