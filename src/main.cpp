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
    parser.addOption({{"p", "pose"},
                       "subscribe ros topic for UAV pose",
                       "topic", "/gps_odom_filter"});
    parser.addOption({{"g", "gps_info"},
                       "subscribe ros topic for GPS info",
                       "topic", "/GPS_odom"});
    parser.addOption({{"o", "origin point"},
                      "subscribe ros topic for origin of longitude and latitude",
                      "topic", "/uncertain"});
    parser.process(app);

    QStringList posArgs = parser.positionalArguments();
    if ( !posArgs.isEmpty() ) {
        const auto &mode = posArgs.at(0);
        if (mode == QLatin1Literal("debug")) {
            g_is_debug_mode = true;
        }
    }

    QScopedPointer<ROSNode> ros_node(new ROSNode(argc, argv));
    QVector<RosTopicPair> ros_topics;
    ros_topics.push_back(qMakePair(parser.value("pose"), 1));
    ros_topics.push_back(qMakePair(parser.value("gps_info"), 1));
    //ros_topics.push_back(qMakePair(parser.value("origin"), 1));
    ros_node->init(ros_topics);

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