#include <string>
#include <cstdio>
#include <cstdlib>

#include "ObjectDetector.h"

//Arquivos utilizados pelo programa
#define FILE_DICTIONARY "ini/dictionary.yml"
#define FILE_DATABASE "ini/objects.ini"
#define FILE_PARAMS "ini/detector.ini"
#define FILE_SVM "ini/svm.ini" 

int main(int argc, char** argv)
{
	ObjectDetector objectDetector;
	objectDetector.LoadParams(FILE_PARAMS);
	objectDetector.LoadObjects(FILE_DATABASE);
	objectDetector.Train();
	objectDetector.SaveDictionary(FILE_DICTIONARY);
	objectDetector.SaveSVM(FILE_SVM);
	printf("\r");

	return 0;
}
