
#include <ros/package.h>

#include <boost/filesystem/path.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <ISM/utility/TableHelper.hpp>

#include "learner/SceneLearningEngine.h"
#include "inference/model/SceneModelDescription.h"

#include <sys/time.h>
#include <string>
#include <chrono>
#include <vector>
#include <string>

#include <random>
#include <algorithm>

/* This program serves to test training and recognition performance of asr_psm.
 * Note that it does not generate test sets (for learning) and validation sets (for recognition)
 * on its own. Generate them using the standard combinatorial optimization launch file with quit_after_test_set_generation set to true
 * and the same parameters as below. Write the databases to asr_psm/data/test/performanceTestRessources/[ValidationSets/]demoRecording_X_Y00.sqlite
 * with X from objectCounts, Y from timestepCounts below.
 * Also note that training will, unless stopped earlier, perform 1000 runs while cycling through the different algorithms, so the comparably long
 * tests can be run mostly automated. Errors are tolerated and marked in the results, again to allow it to run without needing constant supervision.
 * The recognition tests have to be run one each, but they are also usually much quicker. */

using boost::posix_time::ptime;
using boost::posix_time::time_duration;
using boost::filesystem::path;
using namespace ISM;
using namespace ProbabilisticSceneRecognition;
using namespace boost::posix_time;

std::vector<unsigned> objectCounts = {
    3, 4, 5
};

std::vector<unsigned> timestepCounts = {
    100, 200, 300, 400
};

std::string testSetFolder = "ValidationSets";
std::string testSetFolderNoRefs = "TestSetsNoRefs";

std::string validTestSetPref = "validValidationSets_";
std::string invalidTestSetPref = "invalidValidationSets_";
std::string outputPath;

std::vector<std::string> testSets;
std::vector<std::string> testSetsNoRefs;

std::vector<std::string> demoRecordings;
std::vector<std::string> demoRecordingsNames;

double recognitionThreshold = 1.0e-04;
// overwritten in training, used for performance tests:
std::string inferenceAlgorithm = "maximum";
std::string treeAlgorithm = "fullymeshed";
std::string optimizationAlgorithm = "RecordHunt";

bool train = true;
bool test  = true; // recognition runtime

double scale, sigma;
std::string frameId;

void setDefaultParameters()
{
    ros::NodeHandle nh("~");

    nh.setParam("scene_model_type", "ocm");

    nh.setParam("workspace_volume", 8.0);
    nh.setParam("static_break_ratio", 1.01);
    nh.setParam("together_ratio", 0.90);
    nh.setParam("intermediate_results", false);
    nh.setParam("timestamps", false);
    nh.setParam("kernels_min", 1);
    nh.setParam("kernels_max", 5);
    nh.setParam("runs_per_kernel", 3);
    nh.setParam("synthetic_samples", 10);
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
    nh.setParam("attempts_per_run", 10);

    //SHARED PARAMETERS:
    nh.setParam("targeting_help", false);
    nh.setParam("inference_algorithm", inferenceAlgorithm);

    // COMBINATORIAL OPTIMIZATION PARAMETERS:
    nh.setParam("starting_topologies_type", "BestStarOrFullyMeshed");
    nh.setParam("number_of_starting_topologies", 1);
    nh.setParam("neighbourhood_function", "TopologyManager");
    nh.setParam("remove_relations", true);
    nh.setParam("swap_relations", true);
    nh.setParam("maximum_neighbour_count", 100);

    nh.setParam("cost_function", "WeightedSum");
    nh.setParam("false_positives_weight", 5);
    nh.setParam("avg_recognition_time_weight", 1);
    nh.setParam("false_negatives_weight", 5);

    nh.setParam("test_set_generator_type", "relative");
    nh.setParam("test_set_count", 1000);
    nh.setParam("object_missing_in_test_set_probability", 0);
    nh.setParam("recognition_threshold", recognitionThreshold);
    nh.setParam("loaded_test_set_count", 1000);
    nh.setParam("write_valid_test_sets_filename", "");
    nh.setParam("write_invalid_test_sets_filename", "");

    nh.setParam("conditional_probability_algorithm", "minimum");
    nh.setParam("revisit_topologies", true);
    nh.setParam("flexible_recognition_threshold", false);
    nh.setParam("quit_after_test_set_evaluation", false);
    nh.setParam("get_worst_star_topology", false);

    nh.setParam("hill_climbing_random_walk_probability", 0);
    nh.setParam("hill_climbing_random_restart_probability", 0.2);

    nh.setParam("record_hunt_initial_acceptable_cost_delta", 0.02);
    nh.setParam("record_hunt_cost_delta_decrease_factor", 0.01);

    nh.setParam("simulated_annealing_start_temperature", 1);
    nh.setParam("simulated_annealing_end_temperature", 0.005);
    nh.setParam("simulated_annealing_repetitions_before_update", 8);
    nh.setParam("simulated_annealing_temperature_factor", 0.9);

    nh.setParam("xml_output", "file");
    nh.setParam("optimization_history_output", "txt");
    nh.setParam("create_runtime_log", false);
    nh.setParam("log_file_name", "");
    nh.setParam("visualize_inference", false);

    //INFERENCE PARAMETERS:
   nh.setParam("plot", false);
   nh.setParam("object_topic", "/stereo/objects");
   nh.setParam("scene_graph_topic", "/scene_graphs");
   nh.setParam("evidence_timeout", 100);
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
        std::cerr << "std::ios_base::failure" << std::endl;
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
    std::stringstream error;
    error << "";
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
        error << ", Error: " << e.what();
        td = minutes(0);
    }

    long secs = td.total_seconds() % 60;
    long mins = std::floor(td.total_seconds() / 60.);

    os << ", " << mins << "." << secs << error.str() << std::endl;
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
    std::string testSetFolderPath = outputPath + "/" + testSetFolder;

    if(!boost::filesystem::exists(testSetFolderPath))
        boost::filesystem::create_directories(testSetFolderPath);

    for (unsigned int i = 0; i < demoRecordings.size(); ++i) {

        path validTestSetsPath(testSetFolderPath + "/" + validTestSetPref + demoRecordingsNames[i]);
        path invalidTestSetsPath(testSetFolderPath + "/" + invalidTestSetPref + demoRecordingsNames[i]);

        testSets.push_back(validTestSetsPath.string());
        testSets.push_back(invalidTestSetsPath.string());
    }
}

void trainScenes(unsigned int run)
{
    std::ostringstream os;
    os << "Scene, Training duration" << std::endl;
    for (unsigned int i = 0; i < demoRecordings.size(); ++i) {
        ros::NodeHandle nh("~");
        nh.setParam("relation_tree_trainer_type", treeAlgorithm);
        nh.setParam("optimization_algorithm", optimizationAlgorithm);

        std::cout << "Learner input: " << demoRecordings[i] << std::endl;
        nh.setParam("input_db_file", demoRecordings[i]);
        std::string additionalInfo = "";
        if (treeAlgorithm == "combinatorial_optimization") additionalInfo = "_" + optimizationAlgorithm;
        std::string demoName = demoRecordingsNames[i].substr(0, demoRecordingsNames[i].find("."));
        std::string sceneModelName = demoName + "_" + treeAlgorithm + additionalInfo;
        std::cout << "Learning model " << outputPath << "/" << run << "/" << sceneModelName << ".xml" << std::endl;
        nh.setParam("scene_model_name", sceneModelName);
        nh.setParam("scene_model_directory", outputPath + "/" + std::to_string(run));

        nh.setParam("valid_test_set_db_filename", testSets[i * 2]);
        nh.setParam("invalid_test_set_db_filename", testSets[i * 2 + 1]);

        std::string subfolder = outputPath + "/" + std::to_string(run) + "/" + demoName + "/";
        nh.setParam("xml_file_path", subfolder);
        nh.setParam("optimization_history_file_path", subfolder);
        std::cout << "Writing partial models and optimization histories into folder " << subfolder << "/" << demoName << std::endl;

        os << demoRecordings[i] << ", ";
        runOptimization(os);
    }
    writeFile(outputPath + "/" + std::to_string(run), treeAlgorithm + "_" + optimizationAlgorithm + "_d" + std::to_string(run) + "_" + "trainingTime.csv", os);

}

std::vector<ObjectSetPtr> loadTestSetsFromDB(std::string fileName, std::string patternName)
{
    try
    {
    TableHelperPtr localTableHelper(new TableHelper(fileName));
    std::vector<ObjectSetPtr> testSet;
    if (!localTableHelper)
        throw std::runtime_error("No local table helper!");
    if (!localTableHelper->getRecordedPattern(patternName))
        throw std::runtime_error("No recorded pattern for pattern name " + patternName);
    testSet = localTableHelper->getRecordedPattern(patternName)->objectSets;
    std::cout << "Read test sets for pattern " << patternName << std::endl;
    return testSet;
    }
    catch (soci::soci_error& se)
    {
        std::cerr << "Soci error " << se.what() << std::endl;
        throw se;
    }
}

struct PerformanceEvaluationResult
{
    unsigned int falsePositives;
    unsigned int falseNegatives;
    double averageRecognitionRuntime;
    std::vector<std::pair<std::string, double>> recognitionRuntimes;
};

PerformanceEvaluationResult evaluate(const std::vector<ObjectSetPtr>& ts, bool valid, const std::string& patternName, const std::string& trainedScenePath)
{
    PerformanceEvaluationResult er;
    er.falseNegatives = 0;
    er.falsePositives = 0;
    er.recognitionRuntimes = std::vector<std::pair<std::string, double>>();

    double fullRecognitionRuntime = 0;

    std::string in = "";
    if (!valid) in = "in";
    std::cout << "Evaluating " << ts.size() << " " << in << "valid test sets for pattern " << patternName << std::endl;

    boost::shared_ptr<SceneModelDescription> model(new SceneModelDescription());
    boost::shared_ptr<Visualization::ProbabilisticSceneModelVisualization> mVisualizer(new Visualization::ProbabilisticSceneModelVisualization());
    model->loadModelFromFile(trainedScenePath, inferenceAlgorithm);
    model->initializeVisualizer(mVisualizer);
    mVisualizer->setDrawingParameters(scale, sigma, frameId);

    for (unsigned int testSetNumber = 0; testSetNumber < ts.size(); testSetNumber++)
    {
        ObjectSetPtr testSet = ts[testSetNumber];
        if (!testSet)
            throw std::runtime_error("Test set pointer invalid");
        std::vector<SceneIdentifier> sceneList;
        double actualRecognitionThreshold = 0.5;

        time_duration td;

        ros::Time start = ros::Time::now();

        for (unsigned int i = 0; i < testSet->objects.size(); i++)
        {
            boost::shared_ptr<Object> object(new Object(*(testSet->objects[i])));
            model->integrateEvidence(object);
        }
        model->updateModel();
        model->getSceneListWithProbabilities(sceneList);

        ros::Time end = ros::Time::now();
        double recognitionRuntime = end.toSec() - start.toSec();

        std::string validity = "v";
        if (!valid) validity = "i";
        std::string testSetName = patternName + "_" + validity + "_" + std::to_string(testSetNumber);

        std::pair<std::string, double> result(testSetName, -1);
        er.recognitionRuntimes.push_back(result);
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
            er.falsePositives++;
    }

    er.averageRecognitionRuntime = fullRecognitionRuntime / ((double) er.recognitionRuntimes.size());

    return er;
}

// recognition performance
void testPerformance()
{

    std::ostringstream os;
    os << "Scene, Average Recognition Runtime, Standard Deviation of Recognition Runtime" << std::endl;

    unsigned index = 0;
    for (unsigned int i = 0; i < objectCounts.size(); ++i)
    {
        for (unsigned int j = 0; j < timestepCounts.size(); ++j)
        {
            std::cout << "index = " << index << std::endl;
            path demoRecording(demoRecordings[index]);
            std::cout << "demo recording: " << demoRecording.string() << std::endl;

            std::string sceneModelNameSuffix = "_" + treeAlgorithm;
            if (treeAlgorithm == "combinatorial_optimization") sceneModelNameSuffix += "_" + optimizationAlgorithm;
            std::string trainedScenePath = outputPath + "/ProbabilisticSceneModels/" + demoRecording.stem().string() + sceneModelNameSuffix + ".xml" ;

            std::cout << "Using model " << trainedScenePath << std::endl;
            std::string sceneName = "demo_recording_" + std::to_string(objectCounts[i]) + "_" + std::to_string(timestepCounts[j]);

            std::cout << sceneName << std::endl;

            std::string validTestSets = testSets[index * 2];
            std::string invalidTestSets = testSets[index * 2 + 1];

            std::cout << "Loading test sets from " << validTestSets << ", " << invalidTestSets << std::endl;

            std::vector<ObjectSetPtr> validSets = loadTestSetsFromDB(validTestSets, sceneName);
            std::vector<ObjectSetPtr> invalidSets = loadTestSetsFromDB(invalidTestSets, sceneName);

            std::cout << "Test set loading complete." << std::endl;
            std::cout << "Model initialized." << std::endl;

            PerformanceEvaluationResult validER = evaluate(validSets, true, sceneName, trainedScenePath);
            PerformanceEvaluationResult invalidER = evaluate(invalidSets, false, sceneName, trainedScenePath);
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

            // To manuever around a weird issue where, if the actual recognition runtime gets assigned to the pair, there is an munmap_chunk() invalid pointer error:
            er.averageRecognitionRuntime = (validER.averageRecognitionRuntime * validER.recognitionRuntimes.size()
                        + invalidER.averageRecognitionRuntime * invalidER.recognitionRuntimes.size())
                        / (validER.recognitionRuntimes.size() + invalidER.recognitionRuntimes.size());

            std::cout << "Evaluation complete." << std::endl;

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

            std::ostringstream falserec;
            falserec << "scene; false positives; false negatives; sum; average recognition runtime" << std::endl;
            falserec << sceneName << "; " << er.falsePositives << "; " << er.falseNegatives << "; " << er.falsePositives + er.falseNegatives << "; " << er.averageRecognitionRuntime << std::endl;
            writeFile(outputPath, sceneName + "_false_recognitions.csv", falserec);

            os << objectCounts[i] << "-" << timestepCounts[j] << ", " << er.averageRecognitionRuntime <<
                ", " << calculateStandardDeviation(runtimes, er.averageRecognitionRuntime) << std::endl;

            createHistogram(runtimes, 10, sceneName + "_histogram.csv");
            std::cout << "Histogram created." << std::endl;

            index++;
            std::cout << "timestepCounts.size() = " << timestepCounts.size() << ", j = " << j << std::endl;
        }
    }
    writeFile(outputPath, "runtimes.csv", os);
}

int main(int argc, char *argv[])
{
    std::string packagePath = ros::package::getPath("asr_psm");
  outputPath = packagePath + "/data/test/performanceTestRessources";

  if(!boost::filesystem::exists(outputPath))
    boost::filesystem::create_directories(outputPath);

  ros::init(argc, argv, "performance_test" + treeAlgorithm + "_" + optimizationAlgorithm);
  ros::NodeHandle nh;   // necessary to show ros streams

  setDefaultParameters();

  initRandomScenes();
  initTestSets();

  std::vector<std::string> methods = {
      "tree",
      "fullymeshed",
      "combinatorial_optimization",
      "combinatorial_optimization",
      "combinatorial_optimization"
  };
  std::vector<std::string> algos = {
      "HillClimbing",
      "HillClimbing",
      "HillClimbing",
      "RecordHunt",
      "SimulatedAnnealing"
  };
  unsigned int i = 0;
  while (train && i < 1000)
  {
      treeAlgorithm = methods[i % methods.size()];
      optimizationAlgorithm = algos[i % algos.size()];

      std::string subfolder = outputPath + "/" + std::to_string(i);
      if (!boost::filesystem::exists(subfolder))
        boost::filesystem::create_directories(subfolder);
      for (path demoRecording: demoRecordingsNames)
      {
          std::string demofolder = subfolder + "/" + demoRecording.stem().string();
          if (!boost::filesystem::exists(demofolder))
              boost::filesystem::create_directories(demofolder);
      }
      try
      {
        trainScenes(i);
      }
      catch (std::exception& e)
      {
          std::ofstream errorfile;
          errorfile.open(subfolder + "/error.txt");
          if (errorfile.is_open())
          {
              errorfile << e.what();
              errorfile.close();
          }
          std::cerr << e.what();
      }
      i++;
  }

  //Set up LogHelper
  std::string logFileName = "Log_x.txt";
  path logFilePath = outputPath + "/Logfile/" + logFileName;
  LogHelper::init(logFilePath, LOG_INFO);

  if (test) testPerformance();
}


