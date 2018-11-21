#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QDesktopWidget>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QStatusBar>
#include <QtGui/QIcon>
#include <QtCore/QDir>
#include <QtCore/QDebug>
#include <QtCore/QString>
#include <QtCore/QStringList>
#include <QtCore/QFileInfoList>
#include <include/MainWindow.h>

#include "Common.h"
#include "ROSNode.h"
#include "OSGWidget.h"
#include "MainWindow.h"
#include "NetworkManager.h"

bool g_is_debug_mode = false;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    osgwidget_(nullptr),
    dock_widget_(nullptr),
    tree_widget_(nullptr),
    open_file_action(nullptr),
    draw_line_action(nullptr),
    ros_node_(nullptr),
    network_manager_(nullptr) {

    this->setWindowTitle("PointCloudViewer");
    initUI();

    osgwidget_ = new OSGWidget(this);
    this->setCentralWidget(osgwidget_);
    osgwidget_->init();

    if(g_is_debug_mode)
        std::cout << "MainWindow Running on debug mode." << std::endl;
}

MainWindow::~MainWindow() = default;

void MainWindow::initUI() {
    createMenu();
    createToolBar();
    createDockWidget();
}

void MainWindow::createMenu() {
    open_file_action = new QAction("Open", this);
    open_file_action->setIcon(QIcon(":/images/file_open.png"));
    connect(open_file_action, SIGNAL(triggered()), this, SLOT(openFile()));
}

void MainWindow::createToolBar() {
    QToolBar *toolBar = addToolBar("Tools");
    toolBar->addAction(open_file_action);
    toolBar->addSeparator();

    //toolBar->addAction(draw_line_action);
}

void MainWindow::createDockWidget() {
    tree_widget_ = new QTreeWidget(this);
    tree_widget_->setColumnCount(1);
    tree_widget_->setHeaderHidden(true);
    //tree_widget_->setColumnWidth(0, 100);
    //tree_widget_->setStyleSheet("QTreeWidget::item {height:25px;");

    QTreeWidgetItem *multiDataItem = new QTreeWidgetItem(tree_widget_, QStringList(QStringLiteral("数据列表")));
    multiDataItem->setExpanded(true);
    //multiDataItem->setCheckState(0, Qt::CheckState::Checked);
    {

    }

    dock_widget_ = new QDockWidget(QStringLiteral("场景数据"), this);
    dock_widget_->setFixedWidth(200);
    dock_widget_->setFeatures(QDockWidget::AllDockWidgetFeatures);
    dock_widget_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    dock_widget_->setWidget(tree_widget_);
    this->addDockWidget(Qt::LeftDockWidgetArea, dock_widget_);

    //QTreeWidget connect
    //connect(edit_widget_, SIGNAL(itemClicked(QTreeWidgetItem *, int)), this, SLOT(TreeWidgetClicked(QTreeWidgetItem *, int)));
    //connect(edit_widget_, SIGNAL(itemDoubleClicked(QTreeWidgetItem *, int)), this, SLOT(TreeWidgetDoubleClicked(QTreeWidgetItem *, int)));
    //connect(edit_widget_, SIGNAL(itemPressed(QTreeWidgetItem *, int)), this, SLOT(TreeWidgetRightedClicked(QTreeWidgetItem *, int)));
}

void MainWindow::createConnect() {
    QObject::connect(ros_node_.data(), SIGNAL(emitUAVPos(Point)), osgwidget_, SLOT(updateUAVPose(Point)));
    QObject::connect(ros_node_.data(), SIGNAL(emitPointCloud(PointArray)), osgwidget_, SLOT(updatePointCloud(PointArray)));
    QObject::connect(network_manager_.data(), SIGNAL(emitPointCloud(PointArray)), osgwidget_, SLOT(updatePointCloud(PointArray)));
}

void MainWindow::openFile() {
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Model"),
            "/home/zhihui/workspace/data/osg_data", tr("Image Files (*.osg *.osgt *.osgb)"));
    if(fileName.isEmpty()) return;

    QFileInfo f(fileName);
    osgwidget_->readDataFromFile(f);
}

void MainWindow::setRosNode(ROSNode *ros_node) {
    ros_node_.reset(ros_node);
}

void MainWindow::setNetworkManager(NetworkManager *network_manager) {
    network_manager_.reset(network_manager);
}
