/*
 * Copyright (c) 2018 Ally of Intelligence Technology Co., Ltd. All rights reserved.
 *
 * Created by WuKun on 11/19/18.
 * Contact with:wk707060335@gmail.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http: *www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#ifndef POINTCLOUDVIEWER_MAINWINDOW_H
#define POINTCLOUDVIEWER_MAINWINDOW_H

#include <string>

#include <QtWidgets/QMainWindow>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QAction>
#include <QtWidgets/QLabel>
#include <QtCore/QScopedPointer>

#include "Common.h"

//forward declaration
class OSGWidget;
class NetworkManager;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() final;

    void setPointCloudManager(NetworkManager* network_manager);
    void setStatusInfoManager(NetworkManager* network_manager);
    void createConnect();
    void initConfig();

    Q_DISABLE_COPY(MainWindow);
private:
    void initUI();
    void createMenu();
    void createToolBar();
    void createDockWidget();

    //core widget
    OSGWidget*      osgwidget_;

    //other widgets
    QDockWidget*  dock_widget_;
    QTreeWidget*  tree_widget_;
    enum TREE_WIDGET_ITEM {
        GPS_LOCATION,
        SATELLITE_NUM,
        RTK_STATUS,
        GPS_LOCK,
        KINETIC
    };

    //items
    QAction*   open_file_action;
    QAction*   connect_action;
    QAction*   start_action;
    QAction*   end_action;
    QAction*   convert_action;

    QScopedPointer<NetworkManager> pointcloud_manager_;
    QScopedPointer<NetworkManager> statusinfo_manager_;

    std::string ip_address_;
    std::string usr_name_;
    std::string password_;
    std::string connect_cmd_;
    std::string start_cmd_;
    std::string end_cmd_;

public Q_SLOTS:
    void openFile();
    void updateGPSLocation(Point);
    void updateSatelliteNum(QString);
    void updateRTKStatus(bool);
    void updateGPSLock(bool);
    void updateKineticAligment(bool);

    void connectTriggered();
    void startTriggered();
    void endTriggered();
    void convertTriggered();
};


#endif //POINTCLOUDVIEWER_MAINWINDOW_H
