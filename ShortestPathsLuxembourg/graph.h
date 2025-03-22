#ifndef GRAPH_H
#define GRAPH_H

#include <QWidget>
#include <QVector>
#include <QList>
#include <QPair>
#include <QPointF>
#include <queue>

class Graph : public QWidget {
    Q_OBJECT

public:
    explicit Graph(QWidget *parent = nullptr);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;

private:
    QVector<QPointF> nodes;
    QVector<QPair<int, int>> edges;
    QVector<double> costs;
    QVector<QList<QPair<int, double>>> adjacencyList;
    QVector<int> selectedNodes;
    QVector<int> shortestPath;

    void drawEdges(QPainter &painter) const;
    void drawNodes(QPainter &painter) const;
    void drawSelectedNodes(QPainter &painter) const;
    void drawShortestPath(QPainter &painter) const;

    bool isValidNode(int index) const;
    bool isConex(const QVector<QPointF> &nodes, const QVector<QPair<int, int>> &edges);
    void readFromFile(QVector<QPointF> &nodes, QVector<QPair<int, int>> &edges, QVector<double> &costs);
    QVector<QList<QPair<int, double>>> adjList(const QVector<QPair<int, int>> &edges, const QVector<double> &costs, int numNodes);
    QVector<int> dijkstra(const QVector<QPair<int, int>> &edges, const QVector<double> &costs, int numNodes, int source, int target);
    void processNeighbors(int u, const QVector<QPair<int, int>> &edges, const QVector<double> &costs,QVector<double> &dist, QVector<int> &parent,std::priority_queue<QPair<double, int>, QVector<QPair<double, int>>, std::greater<QPair<double, int>>> &priorityQueue);
    QVector<int> reconstructPath(const QVector<double> &dist, const QVector<int> &parent, int target);
    int closestNode(const QVector<QPointF> &nodes, const QPointF &clickPosition);
};

#endif // GRAPH_H
