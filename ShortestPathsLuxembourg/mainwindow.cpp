#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "graph.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    Graph *graph = new Graph(this);
    setCentralWidget(graph);
    resize(1280, 960);
}

MainWindow::~MainWindow()
{
    delete ui;
}
