# Feature Extraction with the BVLC Reference Caffenet

## Prerequisites

To extract features with caffe [^1] make sure you have caffe correctly installed and install the BVLC Reference Caffenet model. A tutorial of how to do this is on the caffe homepage.

Also make sure you have the repos rs_addons[^2] and rs_resources [^3] ready to go in your workspace.

## Setup

To set everything up for feature extraction with the BVLC Reference Caffenet, you have to perform the following steps:

* copy the the **bvlc_reference_caffenet.caffemodel**  modelfile from *your_caffe_source*/models/bvlc_reference_caffenet to *your_rs_resources_source*/caffe/models/bvlc_reference_caffene
* copy the folders with your training data to *your_rs_resources_source*/objects_dataset/object_data
  * as training data you could for example use images from the odu_iai repo[^4] 
  * as a minimal example you could take two folders out of the odu_iai/object/data/partial_views folder
* create a split file and add your classes
  * this gives the feature extractor the information to map the extracted features of your images to their classes
  * you can find an example split file in the folder of this .md file
* create a folder to hold your extracted features
  * you can use the extractedFeats folder in the directory of this .md file

## Feature Extraction

Now you can just type:

 **rosrun rs_addons featureExtractor -s your_split_file -f BVLC_REF -o extractedFeats/**

and the feature extractor will extract your features to your specified folder.

Type **rosrun rs_addons featureExtractor -h** to display information about the featureExtractor.







[^1]: https://caffe.berkeleyvision.org/
[^2]: https://github.com/bbferka/rs_addons
[^3]: https://github.com/RoboSherlock/rs_resources
[^4]: https://github.com/bbferka/odu_iai



