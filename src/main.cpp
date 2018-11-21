#include <QtWidgets/QApplication>
#include <QtCore/QCommandLineOption>
#include <QtCore/QCommandLineParser>
#include <QtCore/QScopedPointer>

#include "Common.h"
#include "ROSNode.h"
#include "MainWindow.h"
#include "NetworkManager.h"

//extern bool g_is_debug_mode;

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    QApplication::setApplicationName("PointCloudViewer");
    QApplication::setApplicationVersion("1.0.0");

    qRegisterMetaType<Point>("Point");
    qRegisterMetaType<PointArray>("PointArray");

    QCommandLineParser parser;
    parser.setApplicationDescription("a PointCloud data viewer for ros message.");
    parser.addHelpOption();
    parser.addVersionOption();
    parser.addPositionalArgument("mode",
                                 "working mode: normal, debug. default: normal");

    parser.addOption({{"l", "listen"},
                       "listening tcp port number",
                       "portNo", "9696"});
    parser.addOption({{"t", "topic"},
                       "subscribe ros topic for UAV pose",
                       "name", "/gps_odom_filter"});
    parser.addOption({{"q", "queuesize"},
                       "ros topic queue size",
                       "num", "1"});
    parser.process(app);

    QStringList posArgs = parser.positionalArguments();
    if ( !posArgs.isEmpty() ) {
        const auto &mode = posArgs.at(0);
        if (mode == QLatin1Literal("debug")) {
            g_is_debug_mode = true;
        }
    }

    QScopedPointer<ROSNode> ros_node(new ROSNode(argc, argv));
    ros_node->init(parser.value("topic").toStdString(), parser.value("queuesize").toUInt());

    QScopedPointer<NetworkManager> network_manager(new NetworkManager);
    network_manager->setPortNum(parser.value("listen").toUShort());

    MainWindow main_window;
    main_window.setGeometry(100, 100, 1500, 800);  //graphic_context bugs!
    main_window.setRosNode(ros_node.take());
    main_window.setNetworkManager(network_manager.take());
    main_window.createConnect();
    main_window.show();

    return app.exec();
}