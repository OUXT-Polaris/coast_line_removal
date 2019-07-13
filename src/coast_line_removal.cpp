#include <coast_line_removal/coast_line_removal.h>

namespace pcl_ros
{
    CoastLineRemoval::CoastLineRemoval()
    {

    }

    void CoastLineRemoval::onInit()
    {

    }

    void CoastLineRemoval::subscribe()
    {
        sync_ptr_.reset(new Synchronizer<sync_policies::ApproximateTime<PointCloud2,LaserScan> >(10));
        poincloud_filter_ptr_.reset(new message_filters::Subscriber<PointCloud2>());
        poincloud_filter_ptr_->subscribe(*pnh_,"points_raw",10);
        scan_filter_ptr_.reset(new message_filters::Subscriber<LaserScan>());
        scan_filter_ptr_->subscribe(*pnh_,"scan",10);
        sync_ptr_->registerCallback(boost::bind(&CoastLineRemoval::callback,this,_1,_2));
    }

    void CoastLineRemoval::unsubscribe()
    {
        poincloud_filter_ptr_->unsubscribe();
        scan_filter_ptr_->unsubscribe();
    }

    void CoastLineRemoval::callback(PointCloud2::ConstPtr cloud_ptr,LaserScan::ConstPtr scan_ptr)
    {

    }
}