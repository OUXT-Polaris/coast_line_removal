#ifndef COAST_LINE_REMOVAL_COAST_LINE_REMOVAL_H_INCLUDED
#define COAST_LINE_REMOVAL_COAST_LINE_REMOVAL_H_INCLUDED

// Headers in ROS
#include <pcl_ros/pcl_nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nodelet_topic_tools/nodelet_lazy.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/pass_through.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace pcl_ros
{
    namespace sync_policies = message_filters::sync_policies;
    using namespace sensor_msgs;
    using namespace message_filters;
    class CoastLineRemoval : public nodelet_topic_tools::NodeletLazy
    {
    public:
        CoastLineRemoval();
    protected:
        void onInit();
        void subscribe();
        void unsubscribe();
    private:
        boost::shared_ptr<Synchronizer<sync_policies::ApproximateTime<PointCloud2,LaserScan> > > sync_ptr_;
        boost::shared_ptr<Subscriber<PointCloud2> > poincloud_filter_ptr_;
        boost::shared_ptr<Subscriber<LaserScan> > scan_filter_ptr_;
        void callback(PointCloud2::ConstPtr cloud_ptr,LaserScan::ConstPtr scan_ptr);
    };
}

#endif  //COAST_LINE_REMOVAL_COAST_LINE_REMOVAL_H_INCLUDED