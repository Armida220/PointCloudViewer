#include <QtWidgets/QGridLayout>
#include <QtCore/QDir>
#include <QtCore/QString>
#include <QtCore/QFileInfoList>
#include <QtCore/QTextStream>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonDocument>

#include <osg/Light>
#include <osg/Point>
#include <osg/Material>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/ValueObject>
#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>
#include <osg/ComputeBoundsVisitor>
#include <osg/PositionAttitudeTransform>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgGA/StateSetManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <include/OSGWidget.h>

#include "Tracer.h"
#include "OSGWidget.h"
#include "NodeCallback.h"
#include "CleanHandler.h"
#include "NodeTreeInfo.h"
#include "NodeTreeSearch.h"

OSGWidget::OSGWidget(QWidget* parent):
    QWidget(parent),
    main_view_(nullptr),
    root_node_(nullptr),
    text_node_(nullptr),
    update_timer_(new QTimer){
}

OSGWidget::~OSGWidget() = default;

void OSGWidget::init() {
    initSceneGraph();
    initHelperNode();
    initCamera();

    QObject::connect(update_timer_.data(), SIGNAL(timeout()), this, SLOT(update()));
    //QObject::connect(update_timer_.data(), &QTimer::timeout, this, &OSGWidget::update);
    update_timer_->start(15);
}

void OSGWidget::paintEvent(QPaintEvent *) {
    frame();
}


void OSGWidget::initSceneGraph() {
    root_node_ = new osg::Switch;
    root_node_->setName(root_node_name);

    osg::ref_ptr<osg::Switch> point_cloud_node = new osg::Switch;
    point_cloud_node->setName(point_cloud_node_name);
    root_node_->addChild(point_cloud_node);

    osg::ref_ptr<osg::PositionAttitudeTransform> uav_node = new osg::PositionAttitudeTransform;
    uav_node->setName(uav_node_name);
    {
        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->setName("UAV");

        osg::ref_ptr<osg::ShapeDrawable> point_sphere = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3d(), 0.5f));
        point_sphere->setColor(osg::Vec4(1.0, 1.0, 0.0, 1.0));
        geode->addDrawable(point_sphere);

        uav_node->addChild(geode);
    }
    root_node_->addChild(uav_node);

    osg::ref_ptr<osg::Camera> hud_node = createHUD();
    hud_node->setName(hud_node_name);
    {
        text_node_ = new osg::Switch;
        text_node_->setName(text_node_name);
        hud_node->addChild(text_node_);
    }
    root_node_->addChild(hud_node);

    osg::ref_ptr<osg::Switch> helper_node = new osg::Switch;
    helper_node->setName(helper_node_name);
    root_node_->addChild(helper_node);
}

void OSGWidget::initCamera() {
    osgViewer::ViewerBase::ThreadingModel threadingModel = osgViewer::ViewerBase::SingleThreaded;
    this->setThreadingModel(threadingModel);
    this->setKeyEventSetsDone(0);

    auto graphic_window = createGraphicsWindow(0, 0, 2000, 2000);
    auto traits = graphic_window->getTraits();

    main_view_ = new osgViewer::View;
    this->addView(main_view_.get());

    auto camera = main_view_->getCamera();
    camera->setGraphicsContext(graphic_window);
    camera->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
    camera->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
    camera->setProjectionMatrixAsPerspective(30.f, static_cast<double>(traits->width) / static_cast<double>(traits->height), 1.0, 1000.0);
    camera->setNearFarRatio(0.0000002);
    camera->setComputeNearFarMode(osg::CullSettings::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES);
//    camera->setClearColor(osg::Vec4(0.84313, 0.84313, 0.89804, 1.0));

    //for outline effects
    {
        osg::DisplaySettings::instance()->setMinimumNumStencilBits(1);
        unsigned int clearMask = camera->getClearMask();
        camera->setClearMask(clearMask | GL_STENCIL_BUFFER_BIT);
        camera->setClearStencil(0);
    }

    main_view_->addEventHandler(new osgViewer::StatsHandler);
    main_view_->addEventHandler(new NodeTreeHandler(root_node_));
    main_view_->addEventHandler(new CleanHandler(root_node_));
    main_view_->setSceneData(root_node_.get());
    main_view_->setCameraManipulator(new osgGA::TrackballManipulator);

    QWidget *widget = graphic_window->getGLWidget();
    auto grid = new QGridLayout;
    grid->addWidget(widget);
    this->setLayout(grid);
}

osgQt::GraphicsWindowQt* OSGWidget::createGraphicsWindow(int x, int y, int w, int h, const std::string& name, bool windowDecoration) const
{
    osg::DisplaySettings* ds = osg::DisplaySettings::instance().get();
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->windowName = name;
    traits->windowDecoration = windowDecoration;
    traits->x = x;
    traits->y = y;
    traits->width = w;
    traits->height = h;
    traits->doubleBuffer = true;
    traits->alpha = ds->getMinimumNumAlphaBits();
    traits->stencil = ds->getMinimumNumStencilBits();
    traits->sampleBuffers = ds->getMultiSamples();
    traits->samples = ds->getNumMultiSamples();

    return new osgQt::GraphicsWindowQt(traits.get());
}

osg::Node *OSGWidget::readPCLDataFromFile(const QFileInfo& file_info) const {
    osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(file_info.filePath().toStdString());
    return node.release();
}

void OSGWidget::readDataFromFile(const QFileInfo &file_info) {
    TRACER;
    static osg::ref_ptr<osg::Switch> point_cloud_node = dynamic_cast<osg::Switch*>(
            NodeTreeSearch::findNodeWithName(root_node_, point_cloud_node_name));

    osg::ref_ptr<osg::Node> node = readPCLDataFromFile(file_info);
    osg::ref_ptr<osg::Geode> bbox = calculateBBoxForModel(node);

    osg::ref_ptr<osg::MatrixTransform> matrix = new osg::MatrixTransform;
    matrix->addChild(node);
    matrix->addChild(bbox);
    matrix->setUpdateCallback(new NodeCallback());

    point_cloud_node->removeChildren(0, point_cloud_node->getNumChildren());
    point_cloud_node->addChild(matrix);
}

void OSGWidget::initHelperNode() {
    static osg::ref_ptr<osg::Switch> helper_node = dynamic_cast<osg::Switch*>(
            NodeTreeSearch::findNodeWithName(root_node_, helper_node_name));

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;

    float radius = 10.0;
    osg::Vec3d axis_center;
    vertices->push_back(axis_center);
    vertices->push_back(axis_center + osg::Vec3(radius, 0, 0));
    vertices->push_back(axis_center + osg::Vec3(0, radius, 0));
    vertices->push_back(axis_center + osg::Vec3(0, 0, radius));

    colors->push_back(osg::Vec3(1, 0, 0));
    colors->push_back(osg::Vec3(0, 1, 0));
    colors->push_back(osg::Vec3(0, 0, 1));

    geom->setVertexArray(vertices);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);

    osg::ref_ptr<osg::DrawElementsUInt> line1 = new osg::DrawElementsUInt(osg::DrawElementsUInt::LINES, 2);
    (*line1)[0] = 0; (*line1)[1] = 1;
    osg::ref_ptr<osg::DrawElementsUInt> line2 = new osg::DrawElementsUInt(osg::DrawElementsUInt::LINES, 2);
    (*line2)[0] = 0; (*line2)[1] = 2;
    osg::ref_ptr<osg::DrawElementsUInt> line3 = new osg::DrawElementsUInt(osg::DrawElementsUInt::LINES, 2);
    (*line3)[0] = 0; (*line3)[1] = 3;

    geom->addPrimitiveSet(line1);
    geom->addPrimitiveSet(line2);
    geom->addPrimitiveSet(line3);

    osg::ref_ptr<osg::StateSet> state_set = geode->getOrCreateStateSet();
    osg::ref_ptr<osg::LineWidth> line_width = new osg::LineWidth(3.0);
    state_set->setAttributeAndModes(line_width);
    geode->addDrawable(geom);

    helper_node->addChild(geode);
}

osg::Geode *OSGWidget::calculateBBoxForModel(osg::Node *node) const {
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;

    osg::ComputeBoundsVisitor bounds_visitor ;
    node->accept(bounds_visitor);
    osg::BoundingBox bb = bounds_visitor.getBoundingBox();
    float lengthX = bb.xMax()-bb.xMin();
    float lengthY = bb.yMax()-bb.yMin();
    float lengthZ = bb.zMax()-bb.zMin();
    osg::Vec3 center = bb.center();
    std::cout << "model center: " << center << std::endl;

    osg::ref_ptr<osg::ShapeDrawable> drawable = new osg::ShapeDrawable(new osg::Box(center,lengthX,lengthY,lengthZ));
    drawable->setColor(osg::Vec4(1.0,1.0,0.0,1.0));

    osg::ref_ptr<osg::StateSet> state_set = drawable->getOrCreateStateSet();
    osg::ref_ptr<osg::PolygonMode> polygon = new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::LINE);
    state_set->setAttributeAndModes(polygon);
    osg::ref_ptr<osg::LineWidth> line_width = new osg::LineWidth(3.0);
    state_set->setAttribute(line_width);
    geode->addDrawable(drawable);

    return geode.release();
}

void OSGWidget::updatePointCloud(PointArray points) {
    //TRACER;
    static osg::ref_ptr<osg::Switch> point_cloud_node = dynamic_cast<osg::Switch*>(
            NodeTreeSearch::findNodeWithName(root_node_, point_cloud_node_name));

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    osg::ref_ptr<osg::Vec3Array> colors = new osg::Vec3Array;

    for(const auto& point : points) {
        osg::Vec3d p(point.x, point.y, point.z);
        vertices->push_back(p);
        colors->push_back(calculateColorForPoint(p));
    }

    geom->setVertexArray(vertices);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->size()));

    geode->addDrawable(geom);
    point_cloud_node->addChild(geode);

    if(point_cloud_node->getNumChildren() >= 10000) {
        point_cloud_node->removeChildren(0, 1000);
    }
}

void OSGWidget::updateUAVPose(Point pos) {
    static osg::ref_ptr<osg::PositionAttitudeTransform> uav_node = dynamic_cast<osg::PositionAttitudeTransform*>(
            NodeTreeSearch::findNodeWithName(root_node_, uav_node_name));

    osg::Vec3d position;
    position.x() = pos.x;
    position.y() = pos.y;
    position.z() = pos.z;

    cur_position = position;
    uav_node->setPosition(position);

    QString position_str = QString::number(pos.x) + "," +
                          QString::number(pos.y) + "," +
                          QString::number(pos.z);
    //update text
    {
        text_node_->removeChildren(0, text_node_->getNumChildren());

        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->setName(positon_geode_name);
        // turn lighting off for the text and disable depth test to ensure it's always on top.
        osg::ref_ptr<osg::StateSet> state_set = geode->getOrCreateStateSet();
        state_set->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

        //position
        {
            osg::Vec3 position(20.0f,20.0f,0.0f);

            osg::ref_ptr<osgText::Text> text = new osgText::Text;
            geode->addDrawable( text );

            text->setPosition(position);
            text->setCharacterSize(20);
            text->setText("UAV Pos: " + position_str.toStdString());
        }
        text_node_->addChild(geode);
    }
}

osg::Camera *OSGWidget::createHUD() {
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;

    camera->setProjectionMatrix(osg::Matrix::ortho2D(0,1280,0,1024));
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setViewMatrix(osg::Matrix::identity());
    camera->setClearMask(GL_DEPTH_BUFFER_BIT);
    camera->setRenderOrder(osg::Camera::POST_RENDER);
    camera->setAllowEventFocus(false);

    return camera.release();
}

osg::Vec3d OSGWidget::calculateColorForPoint(const osg::Vec3d &point) const {
    static std::vector<osg::Vec3> Colors =
            { osg::Vec3(0.0, 0.0, 1.0), osg::Vec3(0.0, 0.2, 1.0), osg::Vec3(0.0, 0.4, 1.0), osg::Vec3(0.0, 0.6, 1.0), osg::Vec3(0.0, 0.8, 1.0), osg::Vec3(0.0, 1.0, 1.0),
              osg::Vec3(0.0, 1.0, 0.8), osg::Vec3(0.0, 1.0, 0.6), osg::Vec3(0.0, 1.0, 0.4), osg::Vec3(0.0, 1.0, 0.2), osg::Vec3(0.0, 1.0, 0.0), osg::Vec3(0.2, 1.0, 0.0),
              osg::Vec3(0.4, 1.0, 0.0), osg::Vec3(0.6, 1.0, 0.0), osg::Vec3(0.8, 1.0, 0.0), osg::Vec3(1.0, 1.0, 0.0), osg::Vec3(1.0, 0.8, 0.0), osg::Vec3(1.0, 0.6, 0.0),
              osg::Vec3(1.0, 0.4, 0.0), osg::Vec3(1.0, 0.2, 0.0), osg::Vec3(1.0, 0.0, 0.0)
            };
    const int height_range = 5;

    double height = std::abs(point.z() - cur_position.z());
    int range = static_cast<int>(height) / height_range;
    if(range >= Colors.size()) range = Colors.size() - 1;
    if(range == 0) range = 1;

    return Colors[Colors.size() - range];
}
