# Perception Demo 

## Prerequisite Files  

* Bag file: https://seafile.zfn.uni-bremen.de/f/271defcfe3e04dbc89ea/?dl=1 

## Running the demo 

* This tutorial expects that the perception installation is complete: https://github.com/SUTURO/suturo_docs/tree/master/bachelor_19_20/installation_guide
* First start the recording by running the following command in the same folder as the bag file: rosbag play -l 2020-12-07-13-47-24.bag
* Then Start the demo pipeline with the following command from the perception workspace: rosrun robosherlock run _ae:=suturo_demo _vis:=true
* If there are some problems with this command maybe the workspace in not sourced, in this case run : source devel/setup.bash and after that run the last command again. 
* This will start the pipeline and after some time two displays will pop up, the Image Viewer for 2D information and Cloud Viewer for depth information. 
* The user can navigate between the pipeline result by clicking on the Image Viewer and then pressing the keys "N" or "O". 
 
# Recording and replaying files from MongoDB in robosherlock on hsrb

## Recording

* choose the name of your new database to record in and set it in storage_suturo.yaml and StorageWriterSuturo.yaml at the field storagedb
* execute *rosrun robosherlock run _ae:=storage_suturo

## Replaying

* Run your Pipeline with CollectionReaderSuturo as the first annotator in the chain
* set the name of the database you want to replay out of in config_mongo_hsrb.ini at the field db
* you can look up the names of your available databases in your MongoDB directory 
  * you can find out where it is stored by looking at dbpath in /etc/mongodb.conf 

# Save Clusters for Klassifikation 

## Record Pictures 

* First set the Recording Plane with : 
* rosrun robosherlock run _ae:=estimate_plane _vis:=true 

* Then place the Objekt in the Turntable and run : 
* rosrun robosherlock run _ae:=save_images _vis:=true 
* All Pictures are saved in the folder "data"

# Feature Extraction 

## Extract Feaures from Picture set 

* Run 
* rosrun rs_addons featureExtractor -s your_split_file -f BVLC_REF -o your_extracted_features_folder

## Region Setup (Simulation)
Execute simulation_region_filter_setup.py from scripts/RegionFilter to generate a new semantic map. Copy this semantic map into the config folder. In „suturo_perception/descriptors/analysis_engines/hsrb.yaml“ you need to insert the names from regions under SuturoRegionFilter. The names can be copied from the semantic map.


## RoboSherlock Caffe Setup

* Download model file with the python script „scripts/download_model_binary.py“ with the parameter „models/bvlc_reference_caffenet “. The script can be found in Caffe directory.

Official explanation of the reference models: https://caffe.berkeleyvision.org/model_zoo.html

* Copy the files from step 1 into your rs_ressources directory to „rs_resources/caffe/models/bvlc_reference_caffenet“. Make sure you use the newest version of the rs_ressources repository from:
https://github.com/Paniago82/rs_resources

* In „rs_addons“ change the file „rs_addons/descriptors/annotators/KnnAnnotator.yaml“
Change the following parameters:
  feature_descriptor_type: BVLC_REF  
  class_label_mapping: extracted_feats/BVLC_REF_ClassLabel_BA.txt
  training_data: extracted_feats/BVLC_REF_data_BA.yaml 




