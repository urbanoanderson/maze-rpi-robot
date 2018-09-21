#include "ObjectDetector.h"

ObjectDetector::ObjectDetector()
{

}

bool ObjectDetector::LoadParams(const char* FILE_PARAMS)
{
	char aux[50];

	printf("\rLoading Detector Params...\n");
	FILE* file_params = fopen(FILE_PARAMS, "r");

	if(file_params != NULL)
	{
		fscanf(file_params, "%*[^:] %*c %lf", &confidence_threshold);
		
		fscanf(file_params, "%*[^:] %*c %[^\n]", aux);
		if(strcmp(aux, "SIFT") == 0)
			descriptor_extractor = 0;
		else
			descriptor_extractor = 1;
		
		fscanf(file_params, "%*[^:] %*c %d", &dictionary_size);
		fscanf(file_params, "%*[^:] %*c %d", &blur_size);
		
		fscanf(file_params, "%*[^:] %*c %[^\n]", aux);
		if(strcmp(aux, "LINEAR") == 0)
			svm_kernel_type = CvSVM::LINEAR;
		
		else if(strcmp(aux, "POLY") == 0)
			svm_kernel_type = CvSVM::POLY;	
		else if(strcmp(aux, "SIGMOID") == 0)
			svm_kernel_type = CvSVM::SIGMOID;
		else
			svm_kernel_type = CvSVM::RBF;
			
		fscanf(file_params, "%*[^:] %*c %d", &svm_degree);
		fscanf(file_params, "%*[^:] %*c %d", &svm_gamma);
		
		printf("\r\tCONFIDENCE THRESHOLD: %.2lf\n", confidence_threshold);
		
		if(descriptor_extractor == 0)
			printf("\r\tDESCRIPTOR EXTRACTOR: SIFT\n");
		else
			printf("\r\tDESCRIPTOR EXTRACTOR: SURF\n");
		
		printf("\r\tDICTIONARY SIZE: %d\n", dictionary_size);
		printf("\r\tBLUR WINDOW SIZE: %d\n", blur_size);
		printf("\r\tSVM KERNEL TYPE: %s\n", aux);
		printf("\r\tSVM DEGREE: %d\n", svm_degree);
		printf("\r\tSVM GAMMA: %d\n", svm_gamma);

		fclose(file_params);
		return true;
	}

	else
	{
		printf("\rError. Could not open ´%s´.\n", FILE_PARAMS);
		return false;
	}
}

bool ObjectDetector::LoadObjects(const char* FILE_DATABASE)
{
	printf("\rLoading Detector Objects...\n");

	//Abre arquivo com informações das imagens dos objetos
	FILE* file_database = fopen(FILE_DATABASE, "r");

	if(file_database != NULL)
	{
		int num_objects;
		
		//Verifica quantos objetos diferentes existem
		fscanf(file_database, "%*[^:] %*c %d", &num_objects);
		char aux[100];
		
		for(int i = 0; i < num_objects; i++)
		{
			//Pega o nome do objeto
			fscanf(file_database, "%*[^:] %*c %[^\n]", aux);
			string name(aux);
			
			printf("\r\tObject: %s\n", name.c_str());
		
			//Pega o número de imagens do objeto
			int num_images;
			fscanf(file_database, "%*[^:] %*c %d", &num_images);
			
			//Pega o nome com path de cada imagem
			vector<string> image_filenames;
			for(int j = 0; j < num_images; j++)
			{
				fscanf(file_database, "%*[^:] %*c %[^\n]", aux);
				string filename(aux);

				if(file_exists(filename.c_str()))
				{
					image_filenames.push_back(filename);
				
					//Insere label no vetor de labels
					Mat label(1, 1, CV_32FC1, cv::Scalar(i));
					labels.push_back(label);
				}

				else 
					printf("\r\t\tError. Could not find object image ´%s´.\n", filename.c_str());

			}
			
			//Insere objeto na lista de objetos do detector
			Object new_object(name, image_filenames, descriptor_extractor);
			objects.push_back(new_object);
		}
		
		fclose(file_database);
		return true;
	}

	else
	{
		printf("\rError. Could not open ´%s´.\n", FILE_DATABASE);
		return false;
	}
}

bool ObjectDetector::LoadDictionary(const char* FILE_DICTIONARY)
{
	//Verifica se existe um dicionário
	FileStorage file_dictionary(FILE_DICTIONARY, FileStorage::READ);
	
	//Se existe o dicionário carrega ele e carrega o svm
	if(file_dictionary.isOpened())
	{
		printf("\rLoading Detector Dictionary...\n");
		file_dictionary["vocabulary"] >> dictionary;
    	file_dictionary.release();
    	return true;
	}

	else
	{
		printf("\rError. Could not open ´%s´.\n", FILE_DICTIONARY);
		return false;
	}
}

bool ObjectDetector::LoadSVM(const char* FILE_SVM)
{
	FILE* file_svm = fopen(FILE_SVM, "r");
	
	if(file_svm != NULL)
	{
		fclose(file_svm); //só pra checar se o arquivo existe
		printf("\rLoading Detector SVM...\n");
    	svm.load(FILE_SVM);
    	return true;
	}

	else
	{
		printf("\rError. Could not open ´%s´.\n", FILE_SVM);
		return false;
	}	
}

void ObjectDetector::Train()
{   
	printf("\rStarting Training...\n");
	
	SiftDescriptorExtractor detector_sift;
	SurfDescriptorExtractor detector_surf;
	Mat featuresUnclustered;
	
	printf("\r\tCreating Dictionary...\n");
	
	//Lê cada imagem do banco para extrair descritores
	for(unsigned int i = 0; i < objects.size(); i++)
	{
		vector<string> image_filenames = objects[i].GetFilenames();
		
		for(unsigned int j = 0; j < image_filenames.size(); j++)
		{
			//Abre a imagem em escala de cinza
			Mat image = imread(image_filenames[j], CV_LOAD_IMAGE_GRAYSCALE);

			//Passa filtro gaussiano na imagem
			if(blur_size > 0)
				GaussianBlur(image, image, Size(blur_size, blur_size), 0, 0);
			
			//Detecta os pontos de interesse
			vector<KeyPoint> keypoints;
			
			if(descriptor_extractor == 0)
				detector_sift.detect(image, keypoints);
			else
				detector_surf.detect(image, keypoints);
		
			//Computa os descritores para cada ponto de interesse
			Mat descriptor;
			
			if(descriptor_extractor == 0)
				detector_sift.compute(image, keypoints, descriptor);
			else
				detector_surf.compute(image, keypoints, descriptor);
			
			//Guarda os descritores encontrados
			featuresUnclustered.push_back(descriptor);   
		}
	}

	//Constroi o dicionário para o BoF
	TermCriteria tc(CV_TERMCRIT_ITER, 100, 0.001);
	int retries = 1;
	int flags = KMEANS_PP_CENTERS;
	BOWKMeansTrainer bowTrainer(dictionary_size, tc, retries, flags);
	dictionary = bowTrainer.cluster(featuresUnclustered);

	//Prepara matriz de treinamento do SVM
	printf("\r\tTraining SVM...\n");
	Mat training_matrix;
	
	for(unsigned int i = 0; i < objects.size(); i++)
	{
		vector<string> image_filenames = objects[i].GetFilenames();
		
		for(unsigned int j = 0; j < image_filenames.size(); j++)
		{
			//Abre a imagem em escala de cinza
			Mat image = imread(image_filenames[j], CV_LOAD_IMAGE_GRAYSCALE);
			
			//Passa filtro gaussiano na imagem
			if(blur_size > 0)
				GaussianBlur(image, image, Size(blur_size, blur_size), 0, 0);
		
			//Computa o histograma segundo o BoF
			Mat image_histogram = ComputeHistogram(image);
			
			//Adiciona à matrix de treinamento
			training_matrix.push_back(image_histogram);  
		}
	}

	//Treina SVM
	CvSVMParams params;
	params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
	params.svm_type = CvSVM::C_SVC;
	params.kernel_type = svm_kernel_type; //LINEAR / POLY / RBF / SIGMOID
	params.degree = svm_degree;	//influencia kernel POLY
	params.gamma = svm_gamma;	//influencia kernels  POLY / RBF / SIGMOID
	svm.train(training_matrix, labels, Mat(), Mat(), params);
}

void ObjectDetector::SaveDictionary(const char* FILE_DICTIONARY)
{
	printf("\rSaving dictionary to ´%s´...\n", FILE_DICTIONARY);
	FileStorage file_dictionary(FILE_DICTIONARY, FileStorage::WRITE);
	file_dictionary << "vocabulary" << dictionary;
	file_dictionary.release();
}

void ObjectDetector::SaveSVM(const char* FILE_SVM)
{
	printf("\rSaving SVM to ´%s´...\n", FILE_SVM);
	svm.save(FILE_SVM);
}

Mat ObjectDetector::ComputeHistogram(Mat image)
{
	Ptr<DescriptorMatcher> matcher(new FlannBasedMatcher);
	Ptr<FeatureDetector> detector_sift(new SiftFeatureDetector());
	Ptr<FeatureDetector> detector_surf(new SurfFeatureDetector());
	Ptr<DescriptorExtractor> extractor_sift(new SiftDescriptorExtractor); 
    Ptr<DescriptorExtractor> extractor_surf(new SurfDescriptorExtractor); 
	BOWImgDescriptorExtractor bowDE_sift(extractor_sift, matcher);
	BOWImgDescriptorExtractor bowDE_surf(extractor_surf, matcher);
	Mat image_histogram; 
	
	if(descriptor_extractor == 0)
	{
		detector_sift->detect(image, keypoints);
		bowDE_sift.setVocabulary(dictionary);
    	bowDE_sift.compute(image, keypoints, image_histogram);
    }
    
    else
    {
		detector_surf->detect(image, keypoints);
		bowDE_surf.setVocabulary(dictionary);
    	bowDE_surf.compute(image, keypoints, image_histogram);
    }
    
    return image_histogram;
}

void ObjectDetector::Detect(Mat image, char* objName)
{	
	//Converte para preto e branco
	Mat frame;
	cvtColor(image, frame, CV_BGR2GRAY);

	//Passa filtro gaussiano
	if(blur_size > 0)
		GaussianBlur(frame, frame, Size(blur_size, blur_size), 0, 0);

	//Computa histograma do frame segundo BoF
	Mat frame_histogram = ComputeHistogram(frame);

	//Inpede crash de histogramas zerados
	if(frame_histogram.rows == 0 || frame_histogram.cols == 0)
	{
		Mat noncrash(1, dictionary_size, CV_32FC1, Scalar::all(0));
		frame_histogram = noncrash;
	}
	
	//Classifica frame
	int prediction = svm.predict(frame_histogram);

	//Verifica a confiança da classificação
	double confidence = 1.0 / (1.0 + exp(-(svm.predict(frame_histogram, true))));

	//Retorna o nome do objeto
	if(confidence >= confidence_threshold)
		strcpy(objName, objects[prediction].GetName().c_str());
	else
		strcpy(objName, "Not Found");
}
