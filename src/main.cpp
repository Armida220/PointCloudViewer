#include <QtWidgets/QApplication>
#include <QtCore/QCommandLineOption>
#include <QtCore/QCommandLineParser>
#include <QtCore/QScopedPointer>

#include "Config.h"
#include "Common.h"
#include "MainWindow.h"
#include "NetworkManager.h"

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    QApplication::setApplicationName("PointCloudViewer");
    QApplication::setApplicationVersion("1.0.0");

#ifdef __GNUC__
    auto config_file_path = "../config/setting.yaml";
#else
    auto config_file_path = "./config/setting.yaml";
#endif
    Config::setParameterFile(config_file_path);

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
    main_window.initConfig();
    main_window.setMinimumSize(800, 600);  //graphic_context bugs!
    main_window.setPointCloudManager(pointcloud_manager.take());
    main_window.setStatusInfoManager(statusinfo_manager.take());
    main_window.createConnect();
    main_window.showMaximized();

    return app.exec();
}