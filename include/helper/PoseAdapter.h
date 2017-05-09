#pragma once

#include <ISM/common_type/Pose.hpp>
#include <Pose.h>
#include <geometry_msgs/Pose.h>

namespace ProbabilisticSceneRecognition {

class PoseAdapter {
public:
    static boost::shared_ptr<ResourcesForPsm::Pose> adapt(boost::shared_ptr<ISM::Pose> pPose)
    {
        geometry_msgs::Pose pose = adaptToGeometryMsg(pPose);
        boost::shared_ptr<ResourcesForPsm::Pose> result(new ResourcesForPsm::Pose(pose));
        return result;
    }

    static boost::shared_ptr<ISM::Pose> adapt(geometry_msgs::Pose pPose)
    {
        ISM::PointPtr point(new ISM::Point(pPose.position.x, pPose.position.y, pPose.position.z));
        ISM::QuaternionPtr quat(new ISM::Quaternion(pPose.orientation.w, pPose.orientation.x, pPose.orientation.y, pPose.orientation.z));
        boost::shared_ptr<ISM::Pose> result(new ISM::Pose(point, quat));
        return result;
    }

    static geometry_msgs::Pose adaptToGeometryMsg(boost::shared_ptr<ISM::Pose> pPose)
    {
        geometry_msgs::Pose pose;
        pose.position.x = pPose->point->getEigen().x();
        pose.position.y = pPose->point->getEigen().y();
        pose.position.z = pPose->point->getEigen().z();
        pose.orientation.w = pPose->quat->getEigen().w();
        pose.orientation.x = pPose->quat->getEigen().x();
        pose.orientation.y = pPose->quat->getEigen().y();
        pose.orientation.z = pPose->quat->getEigen().z();
        return pose;
    }

};

}
