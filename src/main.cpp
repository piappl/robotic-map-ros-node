/*
 * A simple ros node for robotic map interface
 * Author: Adam Dąbrowski, PIAP (www.piap.pl), adabrowski@piap.pl
 */

#include <QQuickView>
#include <QQuickItem>
#include <QGuiApplication>
#include <QQmlEngine>
#include <QFileInfo>
#include <QtConcurrent/QtConcurrent>
#include <QMap>

#include <map/MapWrap.h>
#include <map/MapRobotObject.h>
#include <map/MapWaypointObject.h>
#include <map/GeoMapSender.h>

#include <std_msgs/String.h>
//#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include "rosmapinterface/RobotInformation.h"
#include "rosmapinterface/WaypointInformation.h"

//TODO - this is prototype code, no structure yet

using namespace MapAbstraction;

namespace
{
    MapAbstraction::MapObject::LocalizationType localization(int type)
    {
        switch (type)
        {
        case rosmapinterface::RobotInformation::TYPE_GLOBAL: return MapAbstraction::MapObject::Global;
        case rosmapinterface::RobotInformation::TYPE_LOCAL_ABSOLUTE: return MapAbstraction::MapObject::LocalAbsolute;
        case rosmapinterface::RobotInformation::TYPE_LOCAL_RELATIVE: return MapAbstraction::MapObject::LocalRelative;
        case rosmapinterface::RobotInformation::TYPE_NO_POSITION: return MapAbstraction::MapObject::None;
        default: return MapAbstraction::MapObject::None;
        }
    }

    MapAbstraction::MapObject::LocalizationMode mode(MapAbstraction::MapObject::LocalizationType type)
    {
        return type == MapAbstraction::MapObject::Global ? MapAbstraction::MapObject::Automatic
                                                         : MapAbstraction::MapObject::Assisted;
    }
}

//This interface is used to be noted of new robots in communication zone, which are broadcasting.
//Right now callback only used to advertise waypoint topic by the waypoint sender.
class RobotIDListener : public QObject
{
    Q_OBJECT
public:
    virtual ~RobotIDListener() {}
public slots:
    void robotDetected(int id) { onRobotDetected(id); }
private:
    virtual void onRobotDetected(int id) = 0;
};


class RosListener : public QObject
{
    Q_OBJECT
signals:
    void fileGeoUpdate(QString file);
    void fileGeoRemove(QString key);

    void robotUpdate(MapRobotObjectPtr newState);
    void receivingRobotID(int id);

public:
    static void start(MapWrapPtr mapWrap, QSharedPointer<ros::NodeHandle> handle, RobotIDListener *l)
    {
        QtConcurrent::run(&RosListener::startNewThread, mapWrap, handle, l);
    }

private:
    static void startNewThread(MapWrapPtr mapWrap, QSharedPointer<ros::NodeHandle> handle, RobotIDListener *l)
    {
        RosListener listener;
        listener.runListener(mapWrap, handle, l);
    }

    void fileAddCallback(const std_msgs::String::ConstPtr& msg)
    {
        std::string name = msg->data;
        emit fileGeoUpdate(QString(name.c_str()));
    }

    void fileRemoveCallback(const std_msgs::String::ConstPtr& msg)
    {
        std::string name = msg->data;
        emit fileGeoRemove(QString(name.c_str()));
    }

    void beaconCallback(const rosmapinterface::RobotInformation::ConstPtr& msg)
    {
        bool hasOrientation = true;
        double orientation = msg->theta;
        QString name(msg->robot_type.data());
        QString type(msg->robot_name.data());
        QString description(msg->description.data());
        RobotState state = RobotStateNormal;
        MapAbstraction::MapObject::LocalizationType lt = localization(msg->localization_type);
        MapAbstraction::MapObject::LocalizationMode lm = mode(lt);
        MapAbstraction::GeoCoords coords(msg->x, msg->y);
        int robotID = msg->robot_id;
        int consoleID = 0;
        MapAbstraction::MapRobotObjectPtr obj(
                    new MapAbstraction::MapRobotObject(
                        coords, orientation, state,
                        type, name, description,
                        robotID, consoleID,
                        lt, lm));
        obj->setOrientationAvailable(hasOrientation);
        emit robotUpdate(obj);
        emit receivingRobotID(robotID);

	/*
        //test: double add, double remove
        static int counter = 0;
        counter++;
        QString fileName("/share/maps/localmap/bucharest.kml");
        qDebug("Beacon callback");
        if (counter > 4)
        {
           qDebug("removing kml");
           emit fileGeoRemove(fileName);
           if (counter == 6)
               counter = 0;
        }
        else if (counter == 2 || counter == 3)
        {
            qDebug("adding kml");
            emit fileGeoUpdate(fileName);
        }
	*/
    }

    void runListener(MapWrapPtr mapWrap, QSharedPointer<ros::NodeHandle> handle, RobotIDListener *l)
    {
        connect(this, SIGNAL(receivingRobotID(int)), l, SLOT(robotDetected(int)));
        qDebug("RosListenerr running");
        qRegisterMetaType<MapRobotObjectPtr>("MapRobotObjectPtr");
        connect(this, SIGNAL(robotUpdate(MapRobotObjectPtr)),
                mapWrap.data(), SLOT(updateRobot(MapRobotObjectPtr)), Qt::QueuedConnection);
        connect(this, SIGNAL(fileGeoUpdate(QString)),
                mapWrap.data(), SLOT(updateFileGeometry(QString)));
        connect(this, SIGNAL(fileGeoRemove(QString)),
                mapWrap.data(), SLOT(removeGeometry(QString)));

        std::string subscriberTopicRI = "robot_introduction";
        ros::Subscriber subRI = handle->subscribe<rosmapinterface::RobotInformation>(
                    subscriberTopicRI, 100, &RosListener::beaconCallback, this);

        std::string subscriberTopicAdd = "file_geometry_add";
        std::string subscriberTopicRem = "file_geometry_remove";
        ros::Subscriber subAdd = handle->subscribe<std_msgs::String>(
                    subscriberTopicAdd, 100, &RosListener::fileAddCallback, this);
        ros::Subscriber subRem = handle->subscribe<std_msgs::String>(
                    subscriberTopicRem, 100, &RosListener::fileRemoveCallback, this);
        ros::spin();
    }
};

class RosWaypointSender : public RobotIDListener
{
    Q_OBJECT
    signals:
        void robotConnected(int robot, bool connected);

    public:
        RosWaypointSender(MapWrapPtr mapWrap, QSharedPointer<ros::NodeHandle> handle)
            : mRosHandle(handle), mMapWrap(mapWrap)
        {
            connectSignals();
        }

    public slots:
        void waypointsReceived(int robotID, MapAbstraction::MapPath waypoints)
        {
            if (waypoints.size() < 1)
                return;

            onRobotDetected(robotID);
            qDebug("Received %d waypoints for robot %d", waypoints.size(), robotID);
            rosmapinterface::WaypointInformation msg;
            MapWaypointObjectPtr waypoint = waypoints.first();
            msg.waypoint_name = ""; //"Go there now"
            msg.x = waypoint->coords().longitude();
            msg.y = waypoint->coords().latitude();
            msg.theta = 0;

            mPublishersForRobots.value(robotID)->publish(msg);
        }

        void onRobotDetected(int robotID)
        {
            if (!mPublishersForRobots.contains(robotID))
            {
                qDebug("Robot %d detected", robotID);
                QString topic = "/robot/" + QString::number(robotID) + "/waypoint_information";
                ros::Publisher pub = mRosHandle->advertise<rosmapinterface::WaypointInformation>(
                            topic.toStdString(), 5);
                qDebug("Advertising topic %s", qPrintable(topic));
                mPublishersForRobots.insert(robotID, QSharedPointer<ros::Publisher>(new ros::Publisher(pub)));
            }
        }

    private slots:
        void onConnectionRequest(int robotID, bool connected)
        {   //For connection oriented robots, some logic could be here (i.e negotiate connection,
            //disconnect from previous robot)
            emit robotConnected(robotID, connected);
        }

    private:
        void connectSignals()
        {
            qRegisterMetaType<MapWaypointObjectPtr>("MapWaypointObjectPtr");
            qRegisterMetaType<MapAbstraction::MapPath>("MapPath");
            connect(mMapWrap->sender().data(), SIGNAL(mapPathCreated(int,MapAbstraction::MapPath)),
                    this, SLOT(waypointsReceived(int,MapAbstraction::MapPath)),
                    Qt::QueuedConnection);

            connect(mMapWrap->sender().data(), SIGNAL(requestConnect(int,bool)),
                    this, SLOT(onConnectionRequest(int,bool)),
                    Qt::QueuedConnection);

            connect(this, SIGNAL(robotConnected(int, bool)),
                    mMapWrap.data(), SLOT(robotConnected(int, bool)),
                    Qt::QueuedConnection);
        }

        QMap<int, QSharedPointer<ros::Publisher> > mPublishersForRobots;
        QSharedPointer<ros::NodeHandle> mRosHandle;
        MapWrapPtr mMapWrap;
};

class MapQuickItem : public QQuickView
{
    Q_OBJECT
public:
    MapQuickItem(int argc, char *argv[])
    {
        ros::init(argc, argv, "rosmapinterface");
    }

    void start()
    {
        while (!ros::master::check())
        {
            qDebug("Waiting for ros master");
            sleep(1);
        }

        mHandle.reset(new ros::NodeHandle());

        setSource(QUrl("qrc:/wrap.qml"));
        QQuickItem *viewMapItem = rootObject();
        mMapWrap.reset(new MapWrap(viewMapItem, 0));
        if(status()!=QQuickView::Ready)
            qDebug("can't initialise view");

        QSurfaceFormat format;
        format.setAlphaBufferSize(8);
        setFormat(format);
        setClearBeforeRendering(true);
        setColor(QColor(Qt::transparent));
        setTitle("Ros map interface");

        show();

        mSender.reset(new RosWaypointSender(mMapWrap, mHandle));
        RosListener::start(mMapWrap, mHandle, mSender.data());

        QTimer *checker = new QTimer(this);
        connect(checker, SIGNAL(timeout()), this, SLOT(checkMaster()));
        checker->setSingleShot(false);
        checker->start(1000);
    }

private slots:
    void checkMaster()
    {
        if (!ros::ok())
        {
            qWarning("Ros not ok, shutting down (1)");
            exit(1);
        }
    }

private:
    bool event(QEvent *e)
    {
        if (e->type() == QEvent::Close)
        {
            qDebug("shutting down (0)");
            ros::shutdown();
            exit(0);
        }
        return QQuickView::event(e);
    }

    QSharedPointer<ros::NodeHandle> mHandle;
    MapWrapPtr mMapWrap;
    QSharedPointer<RosWaypointSender> mSender;
};

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);
    MapQuickItem node(argc, argv);
    node.start();
    return app.exec();
}

#include "main.moc"
