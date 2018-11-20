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

#include "MainWindow.h"
#include "OSGWidget.h"

MainWindow::MainWindow(bool debug_mode, QWidget *parent) :
    QMainWindow(parent),
    osgwidget_(nullptr),
    dock_widget_(nullptr),
    tree_widget_(nullptr),
    open_file_action(nullptr),
    draw_line_action(nullptr) {

    this->setWindowTitle("PointCloudViewer");
    initUI();

    osgwidget_ = new OSGWidget(this);
    this->setCentralWidget(osgwidget_);
    osgwidget_->init();
    createConnect();
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
    toolBar->addSeparator();
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
    //connect(osgwidget_->select_editor_, SIGNAL(selectItem(QStringList)), this, SLOT(receiveItem(QStringList)));
}

void MainWindow::openFile() {
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Model"),
            "/home/zhihui/workspace/data/osg_data", tr("Image Files (*.osg *.osgb)"));
    if(fileName.isEmpty()) return;

    QFileInfo f(fileName);
    osgwidget_->readDataFromFile(f);
}
