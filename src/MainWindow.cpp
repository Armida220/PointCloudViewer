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
#include <QtCore/QProcess>
#include <QtCore/QStringList>
#include <QtCore/QFileInfoList>
#include <include/MainWindow.h>

#include "Common.h"
#include "OSGWidget.h"
#include "MainWindow.h"
#include "NetworkManager.h"
extern "C"{
#include "SshControl.h"
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    osgwidget_(nullptr),
    dock_widget_(nullptr),
    tree_widget_(nullptr),
    open_file_action(nullptr),
    start_action(nullptr),
    end_action(nullptr),
    convert_action(nullptr),
    pointcloud_manager_(nullptr),
    statusinfo_manager_(nullptr) {

    this->setWindowTitle("PointCloudViewer");
    initUI();

    osgwidget_ = new OSGWidget(this);
    this->setCentralWidget(osgwidget_);
    osgwidget_->init();
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

    start_action = new QAction(QStringLiteral("开始"), this);
    start_action->setIcon(QIcon(":/images/start.png"));
    connect(start_action, SIGNAL(triggered()), this, SLOT(startTriggered()));

    end_action = new QAction(QStringLiteral("停止"), this);
    end_action->setIcon(QIcon(":/images/end.png"));
    connect(end_action, SIGNAL(triggered()), this, SLOT(endTriggered()));

    convert_action = new QAction(QStringLiteral("格式转换"), this);
    convert_action->setIcon(QIcon(":/images/convert.png"));
    connect(convert_action, SIGNAL(triggered()), this, SLOT(convertTriggered()));
}

void MainWindow::createToolBar() {
    QToolBar *toolBar = addToolBar("Tools");
    //toolBar->addAction(open_file_action);
    toolBar->addAction(start_action);
    toolBar->addSeparator();

    toolBar->addAction(end_action);
    toolBar->addSeparator();

    toolBar->addAction(convert_action);
    toolBar->addSeparator();

    //toolBar->addAction(draw_line_action);
}

void MainWindow::createDockWidget() {
    tree_widget_ = new QTreeWidget(this);
    tree_widget_->setColumnCount(1);
    tree_widget_->setHeaderHidden(true);
    //tree_widget_->setColumnWidth(0, 100);
    //tree_widget_->setStyleSheet("QTreeWidget::item {height:25px;");

    //GPS位置
    {
        QTreeWidgetItem *item = new QTreeWidgetItem(tree_widget_, QStringList(QStringLiteral("GPS位置")));
        item->setExpanded(true);
    }

    //卫星数
    {
        QTreeWidgetItem *item = new QTreeWidgetItem(tree_widget_, QStringList(QStringLiteral("卫星数")));
        item->setExpanded(true);
    }

    //RTK状态
    {
        QTreeWidgetItem *item = new QTreeWidgetItem(tree_widget_, QStringList(QStringLiteral("RTK状态")));
        item->setCheckState(0, Qt::CheckState::Unchecked);
    }


    dock_widget_ = new QDockWidget(QStringLiteral("无人机状态"), this);
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
    QObject::connect(statusinfo_manager_.data(), SIGNAL(emitUAVPos(Point)), osgwidget_, SLOT(updateUAVPose(Point)));
    QObject::connect(statusinfo_manager_.data(), SIGNAL(emitGPSLocation(Point)), this, SLOT(updateGPSLocation(Point)));
    QObject::connect(statusinfo_manager_.data(), SIGNAL(emitSatelliteNum(QString)), this, SLOT(updateSatelliteNum(QString)));
    QObject::connect(statusinfo_manager_.data(), SIGNAL(emitRTKStatus(bool)), this, SLOT(updateRTKStatus(bool)));

    QObject::connect(pointcloud_manager_.data(), SIGNAL(emitPointCloud(PointArray)), osgwidget_, SLOT(updatePointCloud(PointArray)));
}

void MainWindow::openFile() {
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Model"),
            "/home/zhihui/workspace/data/osg_data", tr("Image Files (*.osg *.osgt *.osgb)"));
    if(fileName.isEmpty()) return;

    QFileInfo f(fileName);
    osgwidget_->readDataFromFile(f);
}

void MainWindow::setPointCloudManager(NetworkManager *network_manager) {
    pointcloud_manager_.reset(network_manager);
}

void MainWindow::setStatusInfoManager(NetworkManager *network_manager) {
    statusinfo_manager_.reset(network_manager);
}

void MainWindow::updateGPSLocation(Point p) {
    QString location_str;
    location_str += "latitude:" + QString::number(p.x, 'f', 6) + "\n";
    location_str += "longitude:" + QString::number(p.y, 'f', 6) + "\n";
    location_str += "height:" + QString::number(p.z, 'f', 3);

    auto location_item = tree_widget_->topLevelItem(GPS_LOCATION);

    if(!location_item->childCount()) {
        auto item = new QTreeWidgetItem(location_item);
    }

    auto item = location_item->child(0);
    item->setText(0, location_str);
}

void MainWindow::updateSatelliteNum(QString num) {
    auto satellite_item = tree_widget_->topLevelItem(SATELLITE_NUM);

    if(!satellite_item->childCount()) {
        auto item = new QTreeWidgetItem(satellite_item);
    }

    auto item = satellite_item->child(0);
    item->setText(0, num);
}

void MainWindow::updateRTKStatus(bool is_valid) {
    auto rtk_item = tree_widget_->topLevelItem(RTK_STATUS);

    auto check_state = is_valid ? Qt::CheckState::Checked : Qt::CheckState::Unchecked;
    rtk_item->setCheckState(0, check_state);
}

void MainWindow::startTriggered() {
   executeSshCmd("ls");
   //executeSshCmd("./start.sh");
}

void MainWindow::endTriggered() {
    executeSshCmd("pwd");
    //executeSshCmd("./end.sh");
}

void MainWindow::convertTriggered() {
    QFileInfo file_info("./convert.sh");
    if(!file_info.exists()) {
        QMessageBox::information(this, "Open", "Can't find convert.sh");
        return;
    }

    QString program = file_info.absoluteFilePath();
    QStringList arguments;
    //arguments << "";

    QProcess *process = new QProcess;
    process->startDetached(program, arguments);
    if(!process->waitForStarted()){
        return;
    };

    std::cout << program.toStdString() << " is called!" << std::endl;
}
