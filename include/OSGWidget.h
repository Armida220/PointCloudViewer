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

#ifndef POINTCLOUDVIEWER_OSGWIDGET_H
#define POINTCLOUDVIEWER_OSGWIDGET_H

#include <QtWidgets/QWidget>
#include <QtCore/QFileInfo>
#include <QtCore/QString>
#include <QtCore/QTimer>
#include <QtCore/QScopedPointer>
#include <QtGui/QPaintEvent>

#include <osg/Vec3d>
#include <osg/Camera>
#include <osg/Switch>
#include <osg/Geode>
#include <osgViewer/View>
#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>
#include <osgGA/TrackballManipulator>
#include <osgGA/TerrainManipulator>
#include <osgQt/GraphicsWindowQt>

#include "Common.h"

class OSGWidget : public QWidget, public osgViewer::CompositeViewer {
    Q_OBJECT
public:
    explicit OSGWidget(QWidget* parent = nullptr);
    ~OSGWidget() final;

    Q_DISABLE_COPY(OSGWidget);
public:
    void init();
    void readDataFromFile(const QFileInfo& file_info);

private:
    void paintEvent(QPaintEvent*) final;

    void initSceneGraph();
    void initCamera();
    void initHelperNode();

    osgQt::GraphicsWindowQt* createGraphicsWindow(int x, int y, int w, int h, const std::string& name = "",
            bool windowDecoration = false) const;
    osg::Node* readPCLDataFromFile(const QFileInfo& file_info) const;
    osg::Geode* calculateBBoxForModel(osg::Node* node) const;

    osg::ref_ptr<osgViewer::View>  main_view_;
    osg::ref_ptr<osg::Switch>      root_node_;

    QScopedPointer<QTimer> update_timer_;

public Q_SLOTS:
    void updateUAVPose(Point);
    void updatePointCloud(PointArray);
};


#endif //POINTCLOUDVIEWER_OSGWIDGET_H
