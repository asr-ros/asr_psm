#include <ISM/utility/TableHelper.hpp>
#include "inference/model/SceneModelDescription.h"

using namespace ISM;
using namespace ProbabilisticSceneRecognition;

typedef std::map<std::string, std::vector<ObjectSetPtr> > PatternNameToObjectSet;

struct TestSetSingleResult
{
    std::string sceneName;
    double probability;
    double correct;
};

struct TestSetCompleteResult
{
    std::string testSetName;
    double recognitionRuntime;
    bool testSetValid;
    std::vector<TestSetSingleResult> singleResults;
};

struct EvaluationResult
{
    double averageRecognitionRuntime;
    std::vector<TestSetCompleteResult> testSetResults;
};

struct ModelParams
{
    double recognitionThreshold, scale, sigma;
    std::string validfile, invalidfile, sceneModel, outputFile, frameId;
};

PatternNameToObjectSet loadTestSetsFromDB(std::string fileName)
{
    try {
        TableHelperPtr localTableHelper(new TableHelper(fileName));
        PatternNameToObjectSet testSet;
        for (auto patternName: localTableHelper->getRecordedPatternNames())
            testSet[patternName] = localTableHelper->getRecordedPattern(patternName)->objectSets;
        return testSet;
    } catch (soci::soci_error& se) {
        std::cerr << "Soci error " << se.what() << std::endl;
        throw se;
    }
}

EvaluationResult evaluate(PatternNameToObjectSet ts, bool valid, const ModelParams& params)
{
    EvaluationResult er;

    double fullRecognitionRuntime = 0;

    for (std::pair<std::string, std::vector<ObjectSetPtr>> testSetsByPattern: ts)
    {
        std::string in = "";
        if (!valid) in = "in";
        std::cout << "Evaluating " << testSetsByPattern.second.size() << " " << in << "valid test sets for pattern " << testSetsByPattern.first << std::endl;

        boost::shared_ptr<SceneModelDescription> model;
        boost::shared_ptr<Visualization::ProbabilisticSceneModelVisualization> mVisualizer(new Visualization::ProbabilisticSceneModelVisualization());

        for (unsigned int testSetNumber = 0; testSetNumber < testSetsByPattern.second.size(); testSetNumber++)
        {
            ObjectSetPtr testSet = testSetsByPattern.second[testSetNumber];
            std::vector<SceneIdentifier> sceneList;

            struct timeval start;
            struct timeval end;

            model.reset(new SceneModelDescription());
            model->loadModelFromFile(params.sceneModel, "maximum");
            model->initializeVisualizer(mVisualizer);
            mVisualizer->setDrawingParameters(params.scale, params.sigma, params.frameId);

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
            fullRecognitionRuntime += recognitionRuntime;

            std::string validity = "v";
            if (!valid) validity = "i";
            std::string testSetName = testSetsByPattern.first + "_" + validity + "_" + std::to_string(testSetNumber);

            TestSetCompleteResult tscr;
            tscr.testSetValid = valid;
            tscr.testSetName = testSetName;
            tscr.recognitionRuntime = recognitionRuntime;

            for (SceneIdentifier sceneIdentifier: sceneList)
            {
                double probability = sceneIdentifier.mLikelihood;
                TestSetSingleResult tssr;
                tssr.sceneName = sceneIdentifier.mDescription;
                tssr.probability = probability;
                tssr.correct = true;
                if (valid && probability <= params.recognitionThreshold)
                    tssr.correct = false;
                if (!valid && probability > params.recognitionThreshold)
                    tssr.correct = false;

                tscr.singleResults.push_back(tssr);
            }
            er.testSetResults.push_back(tscr);
        }
    }
    er.averageRecognitionRuntime = fullRecognitionRuntime / ((double) er.testSetResults.size());

    return er;
}

bool find(const std::vector<double>& in, double toFind)
{
    for (double i: in)
    {
        if (std::abs(i - toFind) < std::numeric_limits<double>::epsilon())
            return true;
    }
    return false;
}

/**
 * The validator, which tests the given model against the given test sets and counts false recognitions and runtime.
 * Calculates some statistical values (average of runtime and false recognitions, ratio of false recognition against number of sets).
 * Writes the results as csv to the given file -->
 */
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "js_validator");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    ModelParams mp;

    if (!nh.getParam("recognition_threshold", mp.recognitionThreshold))
        throw std::runtime_error("Please specifiy parameter recognition_threshold when starting this node.");

    if (!nh.getParam("valid_testset_db_filename", mp.validfile))
        throw std::runtime_error("Please specifiy parameter valid_testset_db_filename when starting this node.");
    if (!nh.getParam("invalid_testset_db_filename", mp.invalidfile))
        throw std::runtime_error("Please specifiy parameter invalid_testset_db_filename when starting this node.");
    if (!nh.getParam("scene_model", mp.sceneModel))
        throw std::runtime_error("Please specifiy parameter scene_model when starting this node.");
    if (!nh.getParam("output_file", mp.outputFile))
        throw std::runtime_error("Please specifiy parameter output_file when starting this node.");
    if (!nh.getParam("base_frame_id", mp.frameId))
        throw std::runtime_error("Please specifiy parameter base_frame_id when starting this node.");
    if (!nh.getParam("scale_factor", mp.scale))
        throw std::runtime_error("Please specifiy parameter scale_factor when starting this node.");
    if (!nh.getParam("sigma_multiplicator", mp.sigma))
        throw std::runtime_error("Please specifiy parameter sigma_multiplicator when starting this node.");

    PatternNameToObjectSet vts;
    PatternNameToObjectSet its;

    vts = loadTestSetsFromDB(mp.validfile);
    its = loadTestSetsFromDB(mp.invalidfile);

    EvaluationResult validER = evaluate(vts, true,  mp);
    EvaluationResult invalidER = evaluate(its, false, mp);

    EvaluationResult er;

    er.testSetResults = validER.testSetResults;
    er.testSetResults.insert(er.testSetResults.end(), invalidER.testSetResults.begin(), invalidER.testSetResults.end());

    // Make sure all lists have the same order:
    TestSetCompleteResult ref = er.testSetResults[0];
    for (TestSetCompleteResult tscr: er.testSetResults)
    {
        if (tscr.singleResults.size() != ref.singleResults.size())
            throw std::runtime_error("List size not equal");
        for (unsigned int i = 0; i < ref.singleResults.size(); i++)
            if (tscr.singleResults[i].sceneName != ref.singleResults[i].sceneName)
                throw std::runtime_error("Order in list not equal.");
    }

    std::stringstream csv;
    csv << mp.sceneModel << std::endl;
    csv << "testset; recognition runtime;";
    for (TestSetSingleResult tssr: er.testSetResults[0].singleResults)
        csv << "P(" << tssr.sceneName << "); ";
    csv << std::endl;

    struct ProbSum
    {
        double sum = 0;
        double amount = 0;
    };

    struct FalseRecognition
    {
        unsigned int number = 0;
    };

    std::map<std::string, ProbSum> allProbs;
    std::map<std::string, ProbSum> valProbs;
    std::map<std::string, ProbSum> invProbs;
    std::map<std::string, FalseRecognition> falsePositives;
    std::map<std::string, FalseRecognition> falseNegatives;

    double recognitionRuntimeSum = 0;

    for (TestSetCompleteResult tscr: er.testSetResults)
    {
        csv << tscr.testSetName << "; ";
        recognitionRuntimeSum += tscr.recognitionRuntime;
        csv << tscr.recognitionRuntime << "; ";
        for (TestSetSingleResult tssr: tscr.singleResults)
        {
            csv << tssr.probability << "; ";
            allProbs[tssr.sceneName].sum += tssr.probability;
            allProbs[tssr.sceneName].amount++;

            if (tscr.testSetValid)
            {
                valProbs[tssr.sceneName].sum += tssr.probability;
                valProbs[tssr.sceneName].amount++;
                if (!tssr.correct)
                    falseNegatives[tssr.sceneName].number++;
            }
            else
            {
                invProbs[tssr.sceneName].sum += tssr.probability;
                invProbs[tssr.sceneName].amount++;
                if (!tssr.correct)
                    falsePositives[tssr.sceneName].number++;
            }
        }
        csv << std::endl;
    }
    csv << std::endl;

    er.averageRecognitionRuntime = recognitionRuntimeSum / ((double) er.testSetResults.size());

    unsigned int numberOfValidTestSets = validER.testSetResults.size();
    unsigned int numberOfInvalidTestSets = invalidER.testSetResults.size();

    std::stringstream avgprobs, avgvalp, avginvp, avgfp, avgfpi, avgfpr, avgfn, avgfnv, avgfnr, avgfr, ts, avgfrr;
    for (TestSetSingleResult tssr: ref.singleResults)
    {
        avgprobs << allProbs[tssr.sceneName].sum / ((double) allProbs[tssr.sceneName].amount) << "; ";
        avgvalp << valProbs[tssr.sceneName].sum / ((double) valProbs[tssr.sceneName].amount) << "; ";
        avginvp << invProbs[tssr.sceneName].sum / ((double) invProbs[tssr.sceneName].amount) << "; ";
        avgfp << falsePositives[tssr.sceneName].number << "; ";
        avgfpi << numberOfInvalidTestSets << "; ";
        avgfpr << ((double) falsePositives[tssr.sceneName].number) / ((double) numberOfInvalidTestSets) << "; ";
        avgfn << falseNegatives[tssr.sceneName].number << "; ";
        avgfnv << numberOfValidTestSets << "; ";
        avgfnr << ((double) falseNegatives[tssr.sceneName].number) / ((double) numberOfValidTestSets) << "; ";
        avgfr << falsePositives[tssr.sceneName].number + falseNegatives[tssr.sceneName].number << "; ";
        ts << numberOfInvalidTestSets + numberOfValidTestSets << "; ";
        avgfrr << ((double) (falsePositives[tssr.sceneName].number + falseNegatives[tssr.sceneName].number)) / ((double) (numberOfInvalidTestSets + numberOfValidTestSets)) << "; ";
    }

    csv << "average; " << er.averageRecognitionRuntime << "; " << avgprobs.str() << std::endl;
    csv << "average of valid sets; " << validER.averageRecognitionRuntime << "; " << avgvalp.str() << std::endl;
    csv << "average of invalid sets; " << invalidER.averageRecognitionRuntime << "; " << avginvp.str() << std::endl;
    csv << "false positives; ; " << avgfp.str() << std::endl;
    csv << "invalid sets; ; " << avgfpi.str() << std::endl;
    csv << "ratio; ; " << avgfpr.str() << std::endl;
    csv << "false negatives; ; " << avgfn.str() << std::endl;
    csv << "valid sets; ; " << avgfnv.str() << std::endl;
    csv << "ratio; ; " << avgfnr.str() << std::endl;
    csv << "false recognitions; ; " << avgfr.str() << std::endl;
    csv << "test sets; ; " << ts.str() << std::endl;
    csv << "ratio; ; " << avgfrr.str() << std::endl;

    std::ofstream csvfile;
    csvfile.open(mp.outputFile);
    if (!csvfile.is_open())
            throw std::invalid_argument("Could not open file " + mp.outputFile);
    csvfile << csv.str();
    csvfile.close();

    std::cout << "Wrote evaluation result to file " << mp.outputFile << std::endl;
}

