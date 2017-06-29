
#include <Eigen/Geometry>

#include <ISM/utility/TableHelper.hpp>
#include <ISM/common_type/ObjectSet.hpp>

/**
 * Generates an ISM::PosePtr from the given coordinates.
 * If no orientation is given, a default of no rotation is used.
 * @param x     position
 * @param y     position
 * @param z     position, 0 by default (no z-axis)
 * @param w     orientation
 * @param qx    orientation
 * @param qy    orientation
 * @param qz    orientation
 * @return ISM::PosePtr with these coordinates
 */
ISM::PosePtr generatePose(double x, double y, double z = 0.0, double w = 1.0, double qx = 0.0, double qy = 0.0, double qz = 0.0)
{
    double s = 1.0; // scaling for position

    ISM::PointPtr point(new ISM::Point(s*x, s*y, s*z));
    ISM::QuaternionPtr quaternion(new ISM::Quaternion(w, qx, qy, qz));
    ISM::PosePtr pose(new ISM::Pose(point, quaternion));
    return pose;
}

int main(int argc, char* argv[]) {

    unsigned int stepsPerHalfTrajectory = 200;
    std::string sceneName = "falsepositive";
    std::string folder = "/home/SMBAD/students/nikolai/catkin_ws/src/psm_evaluation/" + sceneName + "/";

    std::string dbfilename = folder + sceneName + "_learningdata.sqlite";
    std::string evidencefilename = folder + sceneName + "_evidence.sqlite";

    // Three poses that form the basis for a trajectory
    struct Triplet
    {
        ISM::PosePtr start, middle, end;
    };

    // Hand-defined trajectory bases:
    Triplet basesA
    {
        generatePose(0.0, 0.0),
        generatePose(0.0, 1.0),
        generatePose(0.0, 2.0)
    };

    Triplet basesB
    {
        generatePose(1.0, 0.5),
        generatePose(1.0, 1.0),
        generatePose(1.0, 1.5)
    };

    Triplet basesC
    {
        generatePose(3.0, 0.0),
        generatePose(2.0, 1.0),
        generatePose(3.0, 2.0)
    };

    // Basic object information
    struct ObjectInformation
    {
        std::string type, meshResourcePath;
        Triplet bases;
    };

    std::vector<ObjectInformation> objectBases;
    objectBases.push_back(ObjectInformation{ "CupRoundHandle",
                                             "package://asr_object_database/rsc/databases/segmentable_objects/CupRoundHandle/object.dae",
                                             basesA});
    objectBases.push_back(ObjectInformation{ "BowlSmall",
                                             "package://asr_object_database/rsc/databases/segmentable_objects/BowlSmall/object.dae",
                                             basesB});
    objectBases.push_back(ObjectInformation{ "Smacks",
                                             "package://asr_object_database/rsc/databases/textured_objects/Smacks/Smacks.dae",
                                             basesC});

    std::vector<ISM::ObjectSet> objectSetList(stepsPerHalfTrajectory * 2);
    for (unsigned int id = 0; id < objectBases.size(); id++)
    {
        ObjectInformation o = objectBases[id];
        Eigen::Vector3d stepFirstHalf = (o.bases.middle->point->getEigen() - o.bases.start->point->getEigen()) / ((double) stepsPerHalfTrajectory);
        Eigen::Vector3d stepSecondHalf = (o.bases.end->point->getEigen() - o.bases.middle->point->getEigen()) / ((double) stepsPerHalfTrajectory);
        for (unsigned int i = 0; i < stepsPerHalfTrajectory; i++)
        {
            Eigen::Vector3d positionFirstHalf = o.bases.start->point->getEigen() + (stepFirstHalf * i);
            Eigen::Vector3d positionSecondHalf = o.bases.middle->point->getEigen() + (stepSecondHalf * i);
            ISM::PosePtr poseFirstHalf = generatePose(positionFirstHalf.x(), positionFirstHalf.y());
            ISM::PosePtr poseSecondHalf = generatePose(positionSecondHalf.x(), positionSecondHalf.y());
            ISM::ObjectPtr objectFirstHalf(new ISM::Object(o.type, poseFirstHalf, std::to_string(id), o.meshResourcePath));
            ISM::ObjectPtr objectSecondHalf(new ISM::Object(o.type, poseSecondHalf, std::to_string(id), o.meshResourcePath));

            objectSetList[i].insert(objectFirstHalf);
            objectSetList[stepsPerHalfTrajectory + i].insert(objectSecondHalf);
        }
    }

    ISM::TableHelperPtr tableHelper(new ISM::TableHelper(dbfilename));
    tableHelper->dropRecordTables();
    tableHelper->createTablesIfNecessary();
    tableHelper->createColumnsIfNecessary();

    for (ISM::ObjectSet& os: objectSetList)
        tableHelper->insertRecordedObjectSet(ISM::ObjectSetPtr(new ISM::ObjectSet(os)), sceneName);

    std::cout << "Generated " << objectSetList.size() << " object sets and wrote them to " << dbfilename << std::endl;

    // Create evidence across the observations:
    ISM::ObjectSet evidence;
    ObjectInformation& o = objectBases[0];
    evidence.insert(ISM::ObjectPtr(new ISM::Object(o.type, o.bases.end, "0", o.meshResourcePath)));

    o = objectBases[objectBases.size() / 2];
    evidence.insert(ISM::ObjectPtr(new ISM::Object(o.type, o.bases.middle, "1", o.meshResourcePath)));

    o = objectBases.back();
    evidence.insert(ISM::ObjectPtr(new ISM::Object(o.type, o.bases.start, "2", o.meshResourcePath)));

    tableHelper.reset(new ISM::TableHelper(evidencefilename));
    tableHelper->dropRecordTables();
    tableHelper->createTablesIfNecessary();
    tableHelper->createColumnsIfNecessary();

    tableHelper->insertRecordedObjectSet(ISM::ObjectSetPtr(new ISM::ObjectSet(evidence)), sceneName);

    std::cout << "Generated evidence and wrote it to " << evidencefilename << std::endl;

    return EXIT_SUCCESS;
}
