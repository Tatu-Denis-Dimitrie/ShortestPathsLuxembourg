#include "Graph.h"
#include <QPainter>
#include <QFile>
#include <QXmlStreamReader>
#include <QDebug>
#include <QSet>
#include <QMouseEvent>
#include <queue>

Graph::Graph(QWidget *parent) : QWidget(parent) {
    readFromFile(nodes, edges, costs);
}

void Graph::paintEvent(QPaintEvent *event) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    drawEdges(painter);
    drawNodes(painter);
    drawSelectedNodes(painter);
    drawShortestPath(painter);
}

void Graph::drawEdges(QPainter &painter) const {
    painter.setPen(Qt::darkCyan);
    for (const auto &edge : edges) {
        if (isValidNode(edge.first) && isValidNode(edge.second)) {
            painter.drawLine(nodes[edge.first], nodes[edge.second]);
        }
    }
}

void Graph::drawNodes(QPainter &painter) const {
    painter.setBrush(Qt::cyan);
    for (const auto &node : nodes) {
        painter.drawEllipse(node, 1, 1);
    }
}

void Graph::drawSelectedNodes(QPainter &painter) const {
    painter.setBrush(Qt::red);
    for (int nodeIndex : selectedNodes) {
        if (isValidNode(nodeIndex)) {
            painter.drawEllipse(nodes[nodeIndex], 2, 2);
        }
    }
}

void Graph::drawShortestPath(QPainter &painter) const {
    if (!shortestPath.isEmpty()) {
        painter.setPen(QPen(Qt::blue, 2));
        for (int i = 0; i < shortestPath.size() - 1; ++i) {
            if (isValidNode(shortestPath[i]) && isValidNode(shortestPath[i + 1])) {
                QPointF point1 = nodes[shortestPath[i]];
                QPointF point2 = nodes[shortestPath[i + 1]];
                painter.drawLine(point1, point2);
            }
        }
    }
}

bool Graph::isValidNode(int index) const {
    return index >= 0 && index < nodes.size();
}

void Graph::readFromFile(QVector<QPointF> &nodes, QVector<QPair<int, int>> &edges, QVector<double> &costs) {
    QFile file("Harta_Luxemburg.xml");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning("Nu s a putut deschide fisierul");
        return;
    }

    QXmlStreamReader xml(&file);
    double normalize = 100000.0;

    while (!xml.atEnd()) {
        xml.readNext();
        if (xml.isStartElement()) {
            if (xml.name() == QStringLiteral("node")) {
                double latitude = xml.attributes().value("latitude").toDouble();
                double longitude = xml.attributes().value("longitude").toDouble();

                latitude /= normalize;
                longitude /= normalize;

                nodes.append(QPointF(latitude, longitude));
            } else if (xml.name() == QStringLiteral("arc")) {
                int source = xml.attributes().value("from").toInt();
                int target = xml.attributes().value("to").toInt();

                double cost = xml.attributes().value("length").toDouble();
                edges.append(qMakePair(source, target));

                costs.append(cost);
            }
        }
    }

    if (!xml.hasError()) {
        qDebug() << "Fisierul xml a fost citit ";
    }

    double minLat = 999999;
    double minLon = 999999;
    double maxLat = -999999;
    double maxLon = -999999;
    for (const auto &node : nodes) {

        minLat = std::min(minLat, node.x());
        minLon = std::min(minLon, node.y());
        maxLat = std::max(maxLat, node.x());
        maxLon = std::max(maxLon, node.y());
    }

    int windowWidth = 1280;
    int windowHeight = 960;

    double scaleX = (windowWidth * 1.0) / (maxLon - minLon);
    double scaleY = (windowHeight * 1.0) / (maxLat - minLat);
    double scale = std::min(scaleX, scaleY);

    double mapCenterLat = (minLat + maxLat) / 2.0;
    double mapCenterLon = (minLon + maxLon) / 2.0;

    double offsetX = windowWidth / 2.0 - (mapCenterLon - minLon) * scale;
    double offsetY = windowHeight / 2.0 - (mapCenterLat - minLat) * scale;

    QVector<QPointF> tempnodes;
    for (const auto &node : nodes) {
        tempnodes.append(QPointF((node.y() - minLon) * scale + offsetX,(maxLat - node.x()) * scale + offsetY ));
    }
    nodes = tempnodes;
    file.close();

    adjacencyList = adjList(edges, costs, nodes.size());
}

bool Graph::isConex(const QVector<QPointF> &nodes, const QVector<QPair<int, int>> &edges) {
    int n = nodes.size();
    QVector<QPair<int, int>> allEdges = edges;
    for (const auto &edge : edges) {
        allEdges.append(qMakePair(edge.second, edge.first));
    }

    QSet<int> visited;
    QVector<int> container;
    container.append(0);

    while (!container.isEmpty()) {
        int current = container.back();
        container.pop_back();

        if (!visited.contains(current)) {
            visited.insert(current);

            for (const auto &edge : allEdges) {
                if (edge.first == current && !visited.contains(edge.second)) {
                    container.append(edge.second);
                }
            }
        }
    }

    return visited.size() == n;
}

QVector<int> Graph::dijkstra(const QVector<QPair<int, int>> &edges, const QVector<double> &costs, int numNodes, int source, int target) {
    QVector<double> distanceValue(numNodes, std::numeric_limits<double>::max());
    QVector<int> parent(numNodes, -1);
    distanceValue[source] = 0;

    std::priority_queue<QPair<double, int>, QVector<QPair<double, int>>, std::greater<QPair<double, int>>> priorityQueue;
    priorityQueue.push(qMakePair(0.0, source));

    while (!priorityQueue.empty()) {
        QPair<double, int> current = priorityQueue.top();
        priorityQueue.pop();
        double currentDist = current.first;
        int u = current.second;

        if (u == target) {
            break;
        }

        if (currentDist > distanceValue[u]) {
            continue;
        }

        processNeighbors(u, edges, costs, distanceValue, parent, priorityQueue);
    }

    return reconstructPath(distanceValue, parent, target);
}


void Graph::processNeighbors(int u, const QVector<QPair<int, int>> &edges, const QVector<double> &costs,
                             QVector<double> &distanceValue, QVector<int> &parent, std::priority_queue<QPair<double, int>, QVector<QPair<double, int>>, std::greater<QPair<double, int>>> &priorityQueue) {
    for (int i = 0; i < edges.size(); ++i) {
        if (edges[i].first == u) {
            int v = edges[i].second;
            double weight = costs[i];

            if (distanceValue[u] + weight < distanceValue[v]) {
                distanceValue[v] = distanceValue[u] + weight;
                parent[v] = u;
                priorityQueue.push(qMakePair(distanceValue[v], v));
            }
        }
    }
}

QVector<int> Graph::reconstructPath(const QVector<double> &distanceValue, const QVector<int> &parent, int target) {
    if (distanceValue[target] == std::numeric_limits<double>::max()) {
        return QVector<int>();
    }

    QVector<int> path;
    for (int v = target; v != -1; v = parent[v]) {
        path.prepend(v);
    }

    return path;
}

void Graph::mousePressEvent(QMouseEvent *event) {
    QPointF clickPosition = event->pos();
    int closestNodes = closestNode(nodes, clickPosition);

    if (closestNodes == -1) {
        qDebug() << "Nu exista niciun nod in apropiere";
        return;
    }

    selectedNodes.append(closestNodes);

    if (selectedNodes.size() == 2) {
        int source = selectedNodes[0];
        int target = selectedNodes[1];

        shortestPath = dijkstra(edges, costs, nodes.size(), source, target);

        if (shortestPath.isEmpty()) {
            qDebug() << "Nu exista nod intre cele 2 puncte selectate";
        }

        selectedNodes.clear();
    }

    update();
}

QVector<QList<QPair<int, double>>> Graph::adjList(const QVector<QPair<int, int>> &edges, const QVector<double> &costs, int numNodes) {
    QVector<QList<QPair<int, double>>> adjacencyList(numNodes);

    for (int i = 0; i < edges.size(); ++i) {
        int u = edges[i].first;
        int v = edges[i].second;
        double cost = costs[i];

        adjacencyList[u].append(qMakePair(v, cost));
    }

    return adjacencyList;
}
int Graph::closestNode(const QVector<QPointF> &nodes, const QPointF &clickPosition) {
    double minDistance = std::numeric_limits<double>::max();
    int closestNode = -1;

    for (int i = 0; i < nodes.size(); ++i) {
        double distance = QLineF(nodes[i], clickPosition).length();
        if (distance < minDistance) {
            minDistance = distance;
            closestNode = i;
        }
    }

    return closestNode;
}
