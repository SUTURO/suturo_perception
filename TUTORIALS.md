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



