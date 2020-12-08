#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <robosherlock/types/all_types.h>
//RS
#include <robosherlock/scene_cas.h>
#include <robosherlock/utils/time.h>

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>

#include <stdlib.h>
#include <string>

using namespace uima;


class classificationEvaluationAnnotator : public Annotator
{
private:
  cv::Mat current_image;
  std::string label_file_path;
  //rapidjson document holding the json Object
  rapidjson::Document document;

  int framecounter = 0;
  int correct_samples = 0;
  int total_samples = 0;
  float accuracy;

public:

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    ctx.extractValue("label_file_path", label_file_path);
    //loading the json file, saving it into a string and parsing it into json object with rapidjson
    std::ifstream file(label_file_path);
    std::string content(std::istreambuf_iterator<char>(file), (std::istreambuf_iterator<char>()));
    document.Parse(content.c_str());

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
      //Initializing the CAS
      outInfo("Starting classification Evaluation");
      rs::SceneCas cas(tcas);
      rs::Scene scene = cas.getScene();

      //Get the current image from the cas
      cas.get(VIEW_COLOR_IMAGE, current_image);

      //Get the clusters in the image found by previous annotators
      std::vector<rs::ObjectHypothesis> clusters;
      scene.identifiables.filter(clusters);
      outInfo("Found " << clusters.size() << " clusters.");

      outInfo("Frame: " << framecounter);
      //only use frame for accuracy calculation if clusters is not empty. This enables leaving frames out.
      if (!document["clusterLabeling"][framecounter]["clusters"].Empty()) {

          //Iterating though the found clusters and getting the classification.
          for (int i = 0; i < clusters.size(); i++) {
              std::vector<rs::Classification> classResult;
              clusters[i].annotations.filter(classResult);
              outInfo("Class result size is: " << classResult.size());

              /*Check if there is a classification result. classResult.size() == 0 happens if the classifier filters by
               * confidence.
               */
              if (classResult.size() > 0) {
                  outInfo("Comparing " << classResult[0].classname.get() << " and "
                                       << document["clusterLabeling"][framecounter]["clusters"][i]["label"].GetString());
                  //count correctly classified samples.
                  if (classResult[0].classname.get().compare(
                          document["clusterLabeling"][framecounter]["clusters"][i]["label"].GetString()) == 0) {
                      outInfo("Correct classification.");
                      correct_samples++;
                      total_samples++;
                  } else {
                      outInfo("Incorrect classification.");
                      total_samples++;
                  }
              } else {  //consider sample as wrongly classified, if confidence was not high enough in KNN.
                  total_samples++;
              }

          }
          //calculate accuracy.
          accuracy = ((float) correct_samples) / ((float) total_samples);
          outInfo("Current accuracy is " << (accuracy * 100) << "%.");
      }
      framecounter++;

      return UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotators.
MAKE_AE(classificationEvaluationAnnotator)
