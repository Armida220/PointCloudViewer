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

#ifndef POINTCLOUDVIEWER_COMMON_H
#define POINTCLOUDVIEWER_COMMON_H

#include <string>
#include <vector>
#include <iostream>

#include <osg/Vec3d>
#include <osg/Array>

const char root_node_name[] = "root_node";
const char point_cloud_node_name[] = "point_cloud_node";
const char helper_node_name[] = "helper_node";

struct Point {
    double x, y, z;
    Point() {
        x = y = z = 0;
    }
};

struct PointArray {
    std::vector<Point> data;
};

inline std::ostream &operator<<( std::ostream &output, const osg::Vec3d &p )
{
    output << p.x() << " " << p.y() << " " << p.z();
    return output;
}

#endif //POINTCLOUDVIEWER_COMMON_H
