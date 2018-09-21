#ifndef OBJECT_DETECTOR_H_INCLUDED
#define OBJECT_DETECTOR_H_INCLUDED

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml/ml.hpp>

#include <algorithm>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <string>

#include "Utils.h"

using namespace std;
using namespace cv;

class Object
{
	private:
		string name;
		vector<string> image_filenames;
		
		Mat main_image;
		vector<KeyPoint> keypoints;
		Mat descriptors;
		
	public:
		Object(string name, vector<string> image_filenames, int descriptor_extractor)
		{ 
			this->name = name; 
			this->image_filenames = image_filenames;
			this->main_image = imread(image_filenames[0], CV_LOAD_IMAGE_GRAYSCALE);
			
			if(descriptor_extractor == 0)
			{
				SiftDescriptorExtractor detector_sift;
				detector_sift.detect(main_image, keypoints);
	  			detector_sift.compute(main_image, keypoints, descriptors);
	  		}
  			
  			else
  			{
  				SurfDescriptorExtractor detector_surf;
				detector_surf.detect(main_image, keypoints);
	  			detector_surf.compute(main_image, keypoints, descriptors);
  			}
		}
		 
		void InsertImageFilename(string image_filename) { image_filenames.push_back(image_filename); }
		string GetName() { return name; }
		vector<string> GetFilenames() { return image_filenames; }
		string GetFilename(int i) { return image_filenames[i]; }
		int GetNumImages() { return (int) image_filenames.size(); }
		Mat GetMainImage() { return main_image; }
		vector<KeyPoint> GetKeypoints() { return keypoints; }
		Mat GetDescriptors() { return descriptors; }
};

class ObjectDetector
{
	private:
		vector<Object> objects;
		Mat dictionary;
		Mat labels;
		CvSVM svm;
		vector<KeyPoint> keypoints;
		
		//Parâmetros de aprendizado
		double confidence_threshold;
		int descriptor_extractor;
		int dictionary_size;
		int blur_size;
		int svm_kernel_type;
		int svm_degree;
		int svm_gamma;
		
		//Funções auxiliares
		Mat ComputeHistogram(Mat image);

	public:
		ObjectDetector();
		bool LoadParams(const char* FILE_PARAMS);
		bool LoadObjects(const char* FILE_DATABASE);
		bool LoadDictionary(const char* FILE_DICTIONARY);
		bool LoadSVM(const char* FILE_SVM);
		void Train();
		void SaveDictionary(const char* FILE_DICTIONARY);
		void SaveSVM(const char* FILE_SVM);
		void Detect(Mat image, char* objName);
};

#endif
