#include "ColorDetector.h"

std::string ColorDetector::Detect(cv::Mat image)
{
    cv::Mat img = image.clone();
    cv::Mat red, blue, white, yellow, skin, green, brown, background;
    cv::Rect rectangle(0,100,640,380);
 
    int qtdRed = 0;
    int qtdBlue = 0;
    int qtdWhite = 0;
    int qtdYellow = 0;
    int qtdSkin = 0;
    int qtdGreen = 0;
    int qtdBrown = 0;
    int qtdBackground = 0;
    int qtdSnoop = 0;
    int qtdCharlieBrown = 0;
    int qtdMario = 0;
    int qtdYoshi = 0;
    int qtdParede = 0;
    int qtdNaoDetectado = 0;
 
    img = img(rectangle); 
    cv::cvtColor(img,img, CV_BGR2HSV);
    inRange(img, cv::Scalar(150, 85, 90), cv::Scalar(179, 255, 207), red);
    inRange(img, cv::Scalar(56, 110, 65), cv::Scalar(160, 255, 167), blue);
    inRange(img, cv::Scalar(40, 0, 148), cv::Scalar(105, 62, 248), white);
    inRange(img, cv::Scalar(14, 103, 85), cv::Scalar(26, 255, 239), yellow);
    inRange(img, cv::Scalar(6, 19, 119), cv::Scalar(22, 100, 214), skin);
    inRange(img, cv::Scalar(26, 80, 60), cv::Scalar(73, 250, 175), green);
    inRange(img, cv::Scalar(0,51,41), cv::Scalar(18,255,164), brown);
    inRange(img, cv::Scalar(0, 0, 56), cv::Scalar(104, 102, 173), background);
 
    qtdRed = cv::countNonZero(red);
    qtdBlue = cv::countNonZero(blue);
    qtdWhite = cv::countNonZero(white);
    qtdYellow = cv::countNonZero(yellow);
    qtdSkin = cv::countNonZero(skin);
    qtdGreen = cv::countNonZero(green);
    qtdBrown = cv::countNonZero(brown);
    qtdBackground = cv::countNonZero(background);

    if(qtdYellow >= 350 && (qtdYellow <= 15000 || qtdGreen <= 3000))
    {
        if(qtdRed <= 50)
            return std::string("Charlie Brown");
        else
            return std::string("Mario");
    }
    else if (qtdRed >= 4000)
        return std::string("Snoopie");
    else if ((qtdBackground >= 209000 && qtdRed <= 10 && qtdYellow <= 5
        && qtdGreen <= 10 ) || qtdBackground >= 230000)
        return std::string("Não detectado");
    else
        return std::string("Yoshi");
}