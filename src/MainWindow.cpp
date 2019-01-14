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

#include <thread>
#include <functional>
#include <include/MainWindow.h>


#include "Common.h"
#include "Config.h"
#include "OSGWidget.h"
#include "MainWindow.h"
#include "NetworkManager.h"
extern "C"{
#include "SshControl.h"
}
#include "PointcloudConvert.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    osgwidget_(nullptr),
    dock_widget_(nullptr),
    tree_widget_(nullptr),
    open_file_action(nullptr),
    connect_action(nullptr),
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

    connect_action = new QAction(QStringLiteral("连接"), this);
    connect_action->setIcon(QIcon(":/images/connection.png"));
    connect(connect_action, SIGNAL(triggered()), this, SLOT(connectTriggered()));

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

    toolBar->addAction(connect_action);
    toolBar->addSeparator();

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

    //GPS lock
    {
        QTreeWidgetItem *item = new QTreeWidgetItem(tree_widget_, QStringList(QStringLiteral("GPS锁定")));
        item->setCheckState(0, Qt::CheckState::Unchecked);
    }

    //Kinetic Aligment
    {
        QTreeWidgetItem *item = new QTreeWidgetItem(tree_widget_, QStringList(QStringLiteral("Kinetic对齐")));
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
    QObject::connect(statusinfo_manager_.data(), SIGNAL(emitGPSLock(bool)), this, SLOT(updateGPSLock(bool)));
    QObject::connect(statusinfo_manager_.data(), SIGNAL(emitKineticAlignment(bool)), this, SLOT(updateKineticAligment(bool)));

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

void MainWindow::updateGPSLock(bool is_valid) {
    auto item = tree_widget_->topLevelItem(GPS_LOCK);

    auto check_state = is_valid ? Qt::CheckState::Checked : Qt::CheckState::Unchecked;
    item->setCheckState(0, check_state);
}

void MainWindow::updateKineticAligment(bool is_valid) {
    auto item = tree_widget_->topLevelItem(KINETIC);

    auto check_state = is_valid ? Qt::CheckState::Checked : Qt::CheckState::Unchecked;
    item->setCheckState(0, check_state);
}

void MainWindow::connectTriggered() {
    int result = execute_ssh_cmd(ip_address_.c_str(), usr_name_.c_str(), password_.c_str(), connect_cmd_.c_str());
    if (result == 0) {
        QMessageBox::information(NULL, "Info", QStringLiteral("连接成功"), QMessageBox::Yes);
    }
    //execute_ssh_cmd("./start.sh");
}
void MainWindow::startTriggered() {
	int result = execute_ssh_cmd(ip_address_.c_str(), usr_name_.c_str(), password_.c_str(), start_cmd_.c_str());
	if (result == 0) {
		QMessageBox::information(NULL, "Info", QStringLiteral("开启成功"), QMessageBox::Yes);
	}
	//execute_ssh_cmd("./start.sh");
}

void MainWindow::endTriggered() {
	int result = execute_ssh_cmd(ip_address_.c_str(), usr_name_.c_str(), password_.c_str(), end_cmd_.c_str());
	if (result == 0) {
		QMessageBox::information(NULL, "Info", QStringLiteral("关闭成功"), QMessageBox::Yes);
	}
    //execute_ssh_cmd("./end.sh");
}

void MainWindow::convertTriggered() {
    QUrl dir_path = QFileDialog::getExistingDirectoryUrl(nullptr, QStringLiteral("选择点云文件夹"));
    if(dir_path.isEmpty())
        return;

    convert_action->setEnabled(false);
    auto dir_path_str = dir_path.toLocalFile().toStdString();

    auto callback = [&]() {
        convert_action->setEnabled(true);
    };

    std::thread thread1(ConvertToLas, dir_path_str, callback);
    thread1.detach();
    std::cout << "get bag dir: " << dir_path_str << std::endl;
}

void MainWindow::initConfig() {
    auto init = [&](std::string& parm, const std::string& name) {
        parm = Config::get<std::string>(name);
    };

    init(ip_address_, "ip_address");
    init(usr_name_, "usr_name");
    init(password_, "password");
    init(connect_cmd_, "connect_bash_file_path");
    init(start_cmd_, "start_bash_file_path");
    init(end_cmd_, "end_bash_file_path");
}
