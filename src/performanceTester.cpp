
#include <boost/filesystem/path.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <ISM/utility/TableHelper.hpp>
//#include "randomDemoRecorder.hpp"

#include "learner/SceneLearningEngine.h"
#include "inference/model/SceneModelDescription.h"

#include <sys/time.h>
#include <string>
#include <chrono>
#include <vector>
#include <string>

#include <random>
#include <algorithm>

using boost::posix_time::ptime;
using boost::posix_time::time_duration;
using boost::filesystem::path;
using namespace ISM;
using namespace ProbabilisticSceneRecognition;

std::vector<unsigned> objectCounts = {
    5, 6, 7 //, 10, 15
};

std::vector<unsigned> timestepCounts = {
    100, 200, 300, 400
};

std::string testSetFolder = "TestSets";
std::string testSetFolderNoRefs = "TestSetsNoRefs";

std::string validTestSetPref = "validTestSets_";
std::string invalidTestSetPref = "invalidTestSets_";
std::string outputPath;

std::vector<std::string> testSets;
std::vector<std::string> testSetsNoRefs;

std::vector<std::string> demoRecordings;
std::vector<std::string> demoRecordingsNames;

double recognitionThreshold = 1.0e-04;
std::string inferenceAlgorithm = "maximum";
std::string treeAlgorithm = "combinatorial_optimization";
std::string optimizationAlgorithm = "HillClimbing";

bool train = true;

double scale, sigma;
std::string frameId;

void setDefaultParameters()
{
    ros::NodeHandle nh("~");

    //nh.setParam("input_db_file", "/home/SMBAD/students/nikolai/catkin_ws/src/psm_evaluation/testsets/Scenes/experiment_2_part_1_and_2.sqlite");

    //nh.setParam("scene_model_name", "advertisement");
    nh.setParam("scene_model_type", "ocm");
    //nh.setParam("scene_model_directory", "$(find asr_psm)/data");
    nh.setParam("workspace_volume", 8.0);
    nh.setParam("static_break_ratio", 1.01);
    nh.setParam("together_ratio", 0.90);
    nh.setParam("intermediate_results", false);
    nh.setParam("timestamps", false);
    nh.setParam("kernels_min", 1);
    nh.setParam("kernels_max", 5);
    nh.setParam("runs_per_kernel", 3);
    nh.setParam("synthetic_samples", 1);
    nh.setParam("interval_position", 0.25);
    nh.setParam("interval_orientation", 10);
    nh.setParam("orientation_plot_path", "UNDEFINED");
    nh.setParam("max_angle_deviation", 45);
    frameId = "/PTU";
    nh.setParam("base_frame_id", frameId);
    scale = 1.0;
    nh.setParam("scale_factor", scale);
    sigma = 3.0;
    nh.setParam("sigma_multiplicator", sigma);
    nh.setParam("attempts_per_run", 100);

    //SHARED PARAMETERS:
    nh.setParam("targeting_help", false);
    nh.setParam("inference_algorithm", inferenceAlgorithm);
    //nh.setParam("runtime_log_path", "/media/data_ext/data/gehrung/evaluation");

    nh.setParam("relation_tree_trainer_type", treeAlgorithm);

    // COMBINATORIAL OPTIMIZATION PARAMETERS:
    nh.setParam("optimization_algorithm", optimizationAlgorithm);
    nh.setParam("starting_topologies_type", "Random");
    nh.setParam("number_of_starting_topologies", 2);
    nh.setParam("neighbourhood_function", "TopologyManager");
    nh.setParam("remove_relations", true);
    nh.setParam("swap_relations", true);
    nh.setParam("maximum_neighbour_count", 100);

    nh.setParam("cost_function", "WeightedSum");
    nh.setParam("false_positives_weight", 5);
    nh.setParam("avg_recognition_time_weight", 1);
    nh.setParam("false_negatives_weight", 5);

    nh.setParam("test_set_generator_type", "absolute");
    nh.setParam("test_set_count", 1000);
    nh.setParam("object_missing_in_test_set_probability", 0);
    nh.setParam("recognition_threshold", recognitionThreshold);
    //nh.setParam("valid_test_set_db_filename", "");
    //nh.setParam("invalid_test_set_db_filename", "");
    nh.setParam("loaded_test_set_count", 1000);
    nh.setParam("write_valid_test_sets_filename", "");
    nh.setParam("write_invalid_test_sets_filename", "");

    nh.setParam("conditional_probability_algorithm", "minimum");
    nh.setParam("revisit_topologies", true);
    nh.setParam("flexible_recognition_threshold", false);
    nh.setParam("quit_after_test_set_evaluation", false);
    nh.setParam("optimize_star_topologies", false);

    nh.setParam("hill_climbing_random_walk_probability", 0);
    nh.setParam("hill_climbing_random_restart_probability", 0);

    nh.setParam("record_hunt_initial_acceptable_cost_delta", 0.02);
    nh.setParam("record_hunt_cost_delta_decrease_factor", 0.01);

    nh.setParam("simulated_annealing_start_temperature", 1);
    nh.setParam("simulated_annealing_end_temperature", 0.005);
    nh.setParam("simulated_annealing_repetitions_before_update", 8);
    nh.setParam("simulated_annealing_temperature_factor", 0.85);

    nh.setParam("xml_output", "file");
    //nh.setParam("xml_file_path", "");
    nh.setParam("optimization_history_output", "txt");
    //nh.setParam("optimization_history_file_path", "");
    nh.setParam("create_runtime_log", false);
    nh.setParam("log_file_name", "");
    nh.setParam("visualize_inference", false);

    //INFERENCE PARAMETERS:
   nh.setParam("plot", false);
   nh.setParam("object_topic", "/stereo/objects");
   nh.setParam("scene_graph_topic", "/scene_graphs");
   //nh.setParam("scene_model_filename", "$(find asr_psm)/data/advertisement.xml");
   nh.setParam("/js_probabilistic_scene_inference_engine/evidence_timeout", 100);
}

void writeFile(const std::string & directoryPath, const std::string & filenName, std::ostringstream & content)
{
    std::string filePath = directoryPath + "/" + filenName;
    std::ofstream file;
    std::ios_base::iostate exceptionMask = file.exceptions() | std::ios::failbit | std::ios::badbit;
    file.exceptions(exceptionMask);
    try
    {
        file.open(filePath);
        file << content.str();
        file.flush();
        file.close();
    }
    catch (std::ios_base::failure& e)
    {
        std::cerr << e.what() << "\n";
    }
}

double calculateStandardDeviation(const std::vector<double>& values, const double& average)
{
    double sumSquares = 0;
    for (double value : values)
    {
        sumSquares += (value - average) * (value - average);
    }

    double variance = sumSquares / values.size();

    return std::sqrt(variance);
}

void createHistogram(const std::vector<double>& values, const unsigned int intervals, const std::string & fileName)
{
    double minValue = std::numeric_limits<double>::max();
    double maxValue = std::numeric_limits<double>::min();

    for (double value : values)
    {
        minValue = std::min(minValue, value);
        maxValue = std::max(maxValue, value);
    }

    double diff = maxValue - minValue;
    double factor = intervals / diff;

    std::vector<unsigned int> histogram(intervals, 0);

    for (double value : values)
    {
        histogram[std::min((unsigned int) std::floor((value - minValue) * factor), intervals)]++;
    }

    std::ostringstream oss;
    oss << "Intervall in seconds, Number Testsets" << std::endl;
    for (unsigned int i = 0; i < intervals; ++i)
    {
        double lower = minValue + (i * 1 / factor);
        double upper = lower + 1 / factor;

        oss << lower << "-" << upper << ", " << histogram[i] << std::endl;
    }

    writeFile(outputPath, fileName, oss);
}

void runOptimization(std::ostringstream & os)
{
    time_duration td;
    try
    {
        ptime t1(boost::posix_time::microsec_clock::local_time());

        boost::shared_ptr<SceneLearningEngine> sceneLearningEngine(new SceneLearningEngine());
        sceneLearningEngine->readLearnerInput();
        sceneLearningEngine->generateSceneModel();
        sceneLearningEngine->saveSceneModel();

        ptime t2(boost::posix_time::microsec_clock::local_time());
        td = t2 - t1;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        throw e;
    }

    long secs = td.total_seconds() % 60;
    long mins = std::floor(td.total_seconds() / 60.);

    os << ", " << mins << "." << secs << std::endl;
}

void initRandomScenes()
{
    for (unsigned int i = 0; i < objectCounts.size(); ++i)
    {
        for (unsigned int j = 0; j < timestepCounts.size(); ++j)
        {
            std::string demoName = "demoRecording_" + std::to_string(objectCounts[i]) + "_" + std::to_string(timestepCounts[j]) + ".sqlite";
            std::string demoRecordingPath = outputPath + "/" + demoName;

            demoRecordings.push_back(demoRecordingPath);
            demoRecordingsNames.push_back(demoName);
        }
    }
}

void initTestSets()
{
    std::string testSetFolderPath = outputPath + "/" + testSetFolder + "1";

    if(!boost::filesystem::exists(testSetFolderPath))
        boost::filesystem::create_directories(testSetFolderPath);

    for (unsigned int i = 0; i < demoRecordings.size(); ++i) {

        path validTestSetsPath(testSetFolderPath + "/" + validTestSetPref + demoRecordingsNames[i]);
        path invalidTestSetsPath(testSetFolderPath + "/" + invalidTestSetPref + demoRecordingsNames[i]);

        testSets.push_back(validTestSetsPath.string());
        testSets.push_back(invalidTestSetsPath.string());
    }
}

void trainScenes()
{
    std::ostringstream os;
    os << "Scene, Training duration" << std::endl;
    for (unsigned int i = 0; i < demoRecordings.size(); ++i) {
        ros::NodeHandle nh("~");

        std::cout << "Learner input: " << demoRecordings[i] << std::endl;
        nh.setParam("input_db_file", demoRecordings[i]);
        std::string additionalInfo = "";
        if (treeAlgorithm == "combinatorial_optimization") additionalInfo = "_" + optimizationAlgorithm;
        std::string demoName = demoRecordingsNames[i].substr(0, demoRecordingsNames[i].find("."));
        std::string sceneModelName = demoName + "_" + treeAlgorithm + additionalInfo;
        std::cout << "Learning model " << outputPath << "/ProbabilisticSceneModels/" << sceneModelName << ".xml" << std::endl;
        nh.setParam("scene_model_name", sceneModelName);
        nh.setParam("scene_model_directory", outputPath + "/ProbabilisticSceneModels");

        nh.setParam("valid_test_set_db_filename", testSets[i * 2]);
        nh.setParam("invalid_test_set_db_filename", testSets[i * 2 + 1]);

        nh.setParam("xml_file_path", outputPath + "/" + demoName + "/");
        nh.setParam("optimization_history_file_path", outputPath + "/" + demoName + "/");
        std::cout << "Writing partial models and optimization histories into folder " << outputPath << "/" << demoName << std::endl;

        os << demoRecordings[i] << ", ";
        runOptimization(os);
    }
    writeFile(outputPath, "trainingTime.csv", os);

}

PatternNameToObjectSet loadTestSetsFromDB(std::string fileName, std::string patternName)
{
    TableHelperPtr localTableHelper(new TableHelper(fileName));
    PatternNameToObjectSet testSet;
    testSet[patternName] = localTableHelper->getRecordedPattern(patternName)->objectSets;
    return testSet;
}

struct PerformanceEvaluationResult
{
    unsigned int falsePositives;
    unsigned int falseNegatives;
    double averageRecognitionRuntime;
    std::vector<std::pair<std::string, double>> recognitionRuntimes;
};

PerformanceEvaluationResult evaluate(PatternNameToObjectSet ts, bool valid, boost::shared_ptr<SceneModelDescription> model)
{
    PerformanceEvaluationResult er;
    er.falseNegatives = 0;
    er.falsePositives = 0;
    er.recognitionRuntimes = std::vector<std::pair<std::string, double>>();

    double fullRecognitionRuntime = 0;

    for (std::pair<std::string, std::vector<ObjectSetPtr>> testSetsByPattern: ts)
    {
        std::string in = "";
        if (!valid) in = "in";
        std::cout << "Evaluating " << testSetsByPattern.second.size() << " " << in << "valid test sets for pattern " << testSetsByPattern.first << std::endl;

        for (unsigned int testSetNumber = 0; testSetNumber < testSetsByPattern.second.size(); testSetNumber++)
        {
            ObjectSetPtr testSet = testSetsByPattern.second[testSetNumber];
            std::vector<SceneIdentifier> sceneList;
            double actualRecognitionThreshold = std::pow(recognitionThreshold, testSet->objects.size() - 1);

            struct timeval start;
            struct timeval end;

            gettimeofday(&start, NULL);

            for (unsigned int i = 0; i < testSet->objects.size(); i++)
            {
                boost::shared_ptr<Object> object(new Object(*(testSet->objects[i])));
                model->integrateEvidence(object);
            }

            model->updateModel();
            model->getSceneListWithProbabilities(sceneList);

            gettimeofday(&end, NULL);

            double recognitionRuntime, seconds, useconds;
            seconds = end.tv_sec - start.tv_sec;
            useconds = end.tv_usec - start.tv_usec;
            recognitionRuntime = seconds + useconds / 1000000;

            std::string validity = "v";
            if (!valid) validity = "i";
            std::string testSetName = testSetsByPattern.first + "_" + validity + "_" + std::to_string(testSetNumber);

            er.recognitionRuntimes.push_back(std::pair<std::string, double>(testSetName, recognitionRuntime));
            fullRecognitionRuntime += recognitionRuntime;

            double probability = -1;
            for (SceneIdentifier sceneIdentifier: sceneList)
            {
                if (sceneIdentifier.mType == "ocm")
                {
                    probability = sceneIdentifier.mLikelihood;
                    break;
                }
            }
            if (probability == -1)
                throw std::runtime_error("Could not find probability for scene ocm in list of scene probabilities.");

            if (valid && probability <= actualRecognitionThreshold)
                er.falseNegatives++;
            if (!valid && probability > actualRecognitionThreshold)
                er.falseNegatives++;
        }
    }
    er.averageRecognitionRuntime = fullRecognitionRuntime / ((double) er.recognitionRuntimes.size());

    return er;
}

void testPerformance()
{

    std::ostringstream os;
    os << "Scene, Average Recognition Runtime, Standard Deviation of Recognition Runtime" << std::endl;

    boost::shared_ptr<SceneModelDescription> model(new SceneModelDescription());
    boost::shared_ptr<Visualization::ProbabilisticSceneModelVisualization> mVisualizer(new Visualization::ProbabilisticSceneModelVisualization());

    unsigned index = 0;
    for (unsigned int i = 0; i < objectCounts.size(); ++i)
    {
        for (unsigned int j = 0; j < timestepCounts.size(); ++j)
        {
            path demoRecording(demoRecordings[index]);

            std::string sceneModelNameSuffix = "_" + treeAlgorithm;
            if (treeAlgorithm == "combinatorial_optimization") sceneModelNameSuffix += "_" + optimizationAlgorithm;
            std::string trainedScenePath = outputPath + "/ProbabilisticSceneModels/" + demoRecording.stem().string() + sceneModelNameSuffix + ".xml" ;

            std::cout << "Using model " << trainedScenePath << std::endl;
            std::string sceneName = "demo_recording_" + std::to_string(objectCounts[i]) + "_" + std::to_string(timestepCounts[j]);

            std::cout << sceneName << std::endl;

            std::pair<PatternNameToObjectSet, PatternNameToObjectSet> ts;

            std::string validTestSets = testSets[index * 2];
            std::string invalidTestSets = testSets[index * 2 + 1];

            ts.first = loadTestSetsFromDB(validTestSets, sceneName);
            ts.second = loadTestSetsFromDB(invalidTestSets, sceneName);

            model->loadModelFromFile(trainedScenePath, inferenceAlgorithm);
            model->initializeVisualizer(mVisualizer);
            mVisualizer->setDrawingParameters(scale, sigma, frameId);

            PerformanceEvaluationResult validER = evaluate(ts.first, true, model);
            PerformanceEvaluationResult invalidER = evaluate(ts.second, false, model);
            EvaluationResult er;
            er.falseNegatives = validER.falseNegatives + invalidER.falseNegatives;
            er.falsePositives = validER.falsePositives + invalidER.falsePositives;

            double fullRecognitionRuntime = 0;
            for (std::pair<std::string, double> validRt: validER.recognitionRuntimes)
            {
                er.recognitionRuntimes.push_back(validRt);
                fullRecognitionRuntime += validRt.second;
            }
            for (std::pair<std::string, double> invalidRt: invalidER.recognitionRuntimes)
            {
                er.recognitionRuntimes.push_back(invalidRt);
                fullRecognitionRuntime += invalidRt.second;
            }
            er.averageRecognitionRuntime = fullRecognitionRuntime / ((double) (validER.recognitionRuntimes.size() + invalidER.recognitionRuntimes.size()));

            std::vector<std::pair<std::string, double>> recognitionRuntimes = er.recognitionRuntimes;
            std::vector<double> runtimes;

            std::ostringstream runtimesPerSetStream;
            runtimesPerSetStream << "Testset name, Average recognition runtime" << std::endl;
            for (std::pair<std::string, double> valuePair : recognitionRuntimes)
            {
                runtimes.push_back(valuePair.second);
                runtimesPerSetStream << valuePair.first << ", " << valuePair.second << std::endl;
            }

            writeFile(outputPath, sceneName + "_runtimes_per_testset.csv", runtimesPerSetStream);

            std::cout << er.getDescription() << std::endl;

            os << objectCounts[i] << "-" << timestepCounts[j] << ", " << er.averageRecognitionRuntime <<
                ", " << calculateStandardDeviation(runtimes, er.averageRecognitionRuntime) << std::endl;

            createHistogram(runtimes, 10, sceneName + "_histogram.csv");

            index++;
        }
    }
    writeFile(outputPath, "runtimes.csv", os);
}

int main(int argc, char *argv[])
{

  outputPath = "/home/SMBAD/students/nikolai/catkin_ws/src/psm_evaluation/performanceTestRessources";

  if(!boost::filesystem::exists(outputPath))
    boost::filesystem::create_directories(outputPath);

  ros::init(argc, argv, "performance_test" + treeAlgorithm);
  ros::NodeHandle nh;   // necessary to show ros streams

  setDefaultParameters();

  initRandomScenes();
  initTestSets();
  if (train) trainScenes();
//  clearRefs();
  testPerformance();

  // testMemoryLeak();

}


