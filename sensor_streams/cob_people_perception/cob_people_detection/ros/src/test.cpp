// substribe face images
// save new database status
face_recognizer_trainer_.saveTrainingData(face_images_,face_depthmaps_);


if (face_recognizer_trainer_.addFace(img_color,img_depth,face_bounding_box,head_bounding_box , current_label_, face_images_,face_depthmaps_)==ipa_Utils::RET_FAILED)
		{
			ROS_WARN("Normalizing failed");
			return;
		}

// label for the images
current_label = goal->label
