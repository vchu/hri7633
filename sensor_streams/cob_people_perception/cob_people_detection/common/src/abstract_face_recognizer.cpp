/*!
*****************************************************************
* \file
*
* \note
* Copyright (c) 2012 \n
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA) \n\n
*
*****************************************************************
*
* \note
* Project name: Care-O-bot
* \note
* ROS stack name: cob_people_perception
* \note
* ROS package name: cob_people_detection
*
* \author
* Author: Richard Bormann
* \author
* Supervised by:
*
* \date Date of creation: 07.08.2012
*
* \brief
* abstract class with common functions for recognizing a face within a color image (patch)
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************/


#ifdef __LINUX__
	#include "cob_people_detection/abstract_face_recognizer.h"
	#include "cob_vision_utils/GlobalDefines.h"
#else
#include "cob_vision/cob_people_detection/common/include/cob_people_detection/PeopleDetector.h"
#include "cob_common/cob_vision_utils/common/include/cob_vision_utils/GlobalDefines.h"
#endif

// stream
#include <fstream>

// opencv
#include <opencv/cv.h>
#include <opencv/cvaux.h>


using namespace ipa_PeopleDetector;

AbstractFaceRecognizer::AbstractFaceRecognizer(void)
{
}

AbstractFaceRecognizer::~AbstractFaceRecognizer(void)
{
}



unsigned long AbstractFaceRecognizer::recognizeFaces(std::vector<cv::Mat>& color_images, std::vector< std::vector<cv::Rect> >& face_coordinates, std::vector< std::vector<std::string> >& identification_labels)
{
	// prepare index list
	identification_labels.clear();
	identification_labels.resize(face_coordinates.size());

	// find identification indices
	for (unsigned int i=0; i<color_images.size(); i++)
	{
		identification_labels[i].resize(face_coordinates[i].size());
		unsigned long result_state = recognizeFace(color_images[i], face_coordinates[i], identification_labels[i]);
		if (result_state == ipa_Utils::RET_FAILED)
			return ipa_Utils::RET_FAILED;
	}
	return ipa_Utils::RET_OK;
}

unsigned long AbstractFaceRecognizer::recognizeFaces(std::vector<cv::Mat>& color_images,std::vector<cv::Mat>& depth_images, std::vector< std::vector<cv::Rect> >& face_coordinates, std::vector< std::vector<std::string> >& identification_labels)
{
  std::vector< std::vector<cv::Mat> > classification_probabilities_vv;
  return recognizeFaces(color_images, depth_images, face_coordinates, identification_labels, classification_probabilities_vv);
}


unsigned long AbstractFaceRecognizer::recognizeFaces(std::vector<cv::Mat>& color_images,std::vector<cv::Mat>& depth_images, std::vector< std::vector<cv::Rect> >& face_coordinates, std::vector< std::vector<std::string> >& identification_labels, std::vector< std::vector<cv::Mat> >& classification_probabilities_vv)
{
	// prepare index list
	identification_labels.clear();
	identification_labels.resize(face_coordinates.size());

	// prepare probabilities
        classification_probabilities_vv.clear();
	classification_probabilities_vv.resize(face_coordinates.size());
	std::cout << "Resizing classification_probabilities" << std::endl;

	// find identification indices
	for (unsigned int i=0; i<color_images.size(); i++)
	{
		identification_labels[i].resize(face_coordinates[i].size());
		classification_probabilities_vv[i].resize(face_coordinates[i].size());		
		unsigned long result_state = recognizeFace(color_images[i],depth_images[i], face_coordinates[i], identification_labels[i], classification_probabilities_vv[i]);
		//		unsigned long result_state = recognizeFace(color_images[i],depth_images[i], face_coordinates[i], identification_labels[i]);		
		if (result_state == ipa_Utils::RET_FAILED)
			return ipa_Utils::RET_FAILED;
	}

	return ipa_Utils::RET_OK;
}
