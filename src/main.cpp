#include <QtWidgets/QApplication>
#include <QtCore/QCommandLineOption>
#include <QtCore/QCommandLineParser>
#include <QtCore/QScopedPointer>

#include "Common.h"
#include "MainWindow.h"
#include "NetworkManager.h"

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    QApplication::setApplicationName("PointCloudViewer");
    QApplication::setApplicationVersion("1.0.0");

    qRegisterMetaType<Point>("Point");
    qRegisterMetaType<PointArray>("PointArray");

    QScopedPointer<NetworkManager> pointcloud_manager(new NetworkManager);
    pointcloud_manager->setObjectName("PointCloud Receiver");
    pointcloud_manager->setMode(NetworkManager::MODE::POINTCLOUD);
    pointcloud_manager->setPortNum(9696);

    QScopedPointer<NetworkManager> statusinfo_manager(new NetworkManager);
    statusinfo_manager->setObjectName("StatusInfo Receiver");
    statusinfo_manager->setMode(NetworkManager::MODE::STATUSINFO);
    statusinfo_manager->setPortNum(9697);

    MainWindow main_window;
    main_window.setGeometry(100, 100, 1500, 800);  //graphic_context bugs!
    main_window.setPointCloudManager(pointcloud_manager.take());
    main_window.setStatusInfoManager(statusinfo_manager.take());
    main_window.createConnect();
    main_window.show();

    return app.exec();
}