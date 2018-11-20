#include <QtWidgets/QApplication>
#include <QtCore/QCommandLineOption>
#include <QtCore/QCommandLineParser>

#include "MainWindow.h"


int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    QApplication::setApplicationName("PointCloudViewer");
    QApplication::setApplicationVersion("1.0.0");

    QCommandLineParser parser;
    parser.setApplicationDescription("a PointCloud data viewer for ros message.");
    parser.addHelpOption();
    parser.addVersionOption();
    parser.addPositionalArgument("mode",
                                 "working mode: normal, debug. default: normal");

    parser.addOption({{"l", "listen"},
                       "listening tcp port number in server mode (default 8080)",
                       "portNo", "8080"});
    parser.addOption({{"u", "url"},
                       "fetch url data in client mode",
                       "address", "http://www.google.com"});
    parser.addOption({{"g", "geolocation"},
                       "a city name [,country name] in weather mode, default: Tehran",
                       "city", "Tehran"});
    parser.process(app);

    bool is_debug_mode = false;
    QStringList posArgs = parser.positionalArguments();
    if ( !posArgs.isEmpty() ) {
        const auto &mode = posArgs.at(0);
        if (mode == QLatin1Literal("debug")) {
            is_debug_mode = true;
        }
    }

    MainWindow main_window(is_debug_mode);
    main_window.setGeometry(100, 100, 1500, 800);  //graphic_context bugs!
    main_window.show();

    return app.exec();
}