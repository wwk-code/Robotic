#ifndef CAMERACALIBRATOR_H
#define CAMERACALIBRATOR_H
/*
height	rows Y j
width	cols X i
*/
#include "Head.h"

class CameraCalibrator
{
public:
	bool checkChessBoard(Mat temp);

	CameraCalibrator() {};
	~CameraCalibrator() {};
	void InitCameraCalibrator(int i);
	cv::Mat getIntrinsicMatrix() const {return intrinsicMatrix;}
	cv::Mat getDistortionCoeffs() const {return distortionCoeffs;}
	std::vector<std::vector<cv::Point2f>> getImagePoint() const {return imagePoints;};
	int addChessboardPoints(const std::vector<std::string> &fileList,cv::Size &boardSize);	//���ǵ�
	double calibrate();	//����궨
	void CalcUndistort();//��������������
	cv::Mat undistorted(const cv::Mat &image);	//ȥ����
	void SaveIntrinsicDistortion();
	void ReadIntrinsicDistortion();

private:
	std::vector<std::vector<cv::Point3f>> objectPoints;	//��������
	std::vector<std::vector<cv::Point2f>> imagePoints;	//ͼ������
	cv::Mat intrinsicMatrix;	//�ڲ�������
	cv::Mat distortionCoeffs;	//����ϵ��
	cv::Mat mapX,mapY;			//ȥ�����������
	void addPoint(const std::vector<cv::Point2f> &imageCorners, const std::vector<cv::Point3f> &objectCorners)
	{
		imagePoints.push_back(imageCorners);
		objectPoints.push_back(objectCorners);
	}
	int device;
};
void CameraCalibrator::InitCameraCalibrator(int i)
{
	device = i;
}

bool CameraCalibrator::checkChessBoard(Mat temp)
{
	bool flag = false;
	std::vector<cv::Point2f> imageCorners;
	flag = cv::findChessboardCorners(temp,boardSize,imageCorners);	//8bit�Ҷ�or��ɫͼ
	if(flag && imageCorners.size() == boardSize.area())
	{
		flag = true;
	}
	return flag;
}

int CameraCalibrator::addChessboardPoints(const std::vector<std::string> &filelist,cv::Size &boardSize)
{
	std::vector<cv::Point2f> imageCorners;
	std::vector<cv::Point3f> objectCorners;
	for(int i = 0;i < boardSize.height;i++)
	{
		for(int j = 0;j < boardSize.width;j++)
		{
			objectCorners.push_back(cv::Point3f(i,j,0.0f));
		}
	}
	cv::Mat temp;
	int success = 0;
	for(int i = 0;i < filelist.size();i++)
	{
		temp = cv::imread(filelist[i],0);
		if(temp.empty())
		{
			return -1;
		}
		bool found = cv::findChessboardCorners(temp,boardSize,imageCorners);	//8bit�Ҷ�or��ɫͼ
		//��������(5*2+1)*(5*2+1),zeroZone��������(-1,-1)��ʾ��,�ǵ㾫׼��ֹ��������30��or eps<0.01
		cv::cornerSubPix(temp,imageCorners,cv::Size(5,5),cv::Size(-1,-1),cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,30,0.1));	
		if(found && imageCorners.size() == boardSize.area())
		{
			addPoint(imageCorners,objectCorners);
			success++;
			std::cout<<success<<std::endl;
			cv::drawChessboardCorners(temp,boardSize,imageCorners,found);
			cv::imshow("Corners on Chessboard",temp);
			cv::waitKey(100);
		}
		else
		{
			std::cout<<"Can't find "<<i <<"-th chessImage corner."<<std::endl;
		}
	}
	cv::destroyWindow("Corners on Chessboard");
	return success;
}

double CameraCalibrator::calibrate()
{
	std::vector<cv::Mat> Rvecs,Tvecs;
	return cv::calibrateCamera(objectPoints,imagePoints,RESOLUTION,intrinsicMatrix,distortionCoeffs,Rvecs,Tvecs);
}

void CameraCalibrator::CalcUndistort()
{
	cv::initUndistortRectifyMap(intrinsicMatrix,distortionCoeffs,cv::Mat(),cv::Mat(),RESOLUTION,CV_32FC1,mapX,mapY);
}

cv::Mat CameraCalibrator::undistorted(const cv::Mat &image)
{
	cv::Mat undistorted;
	//Ӧ��ӳ�����˫���Բ�ֵ��
	cv::remap(image,undistorted,mapX,mapY,cv::INTER_LINEAR);
	return undistorted;
}

void CameraCalibrator::SaveIntrinsicDistortion()
{
	std::stringstream filename;
	filename<<"cap"<<device<<"\\"<<"Matrix.xml";
	cv::FileStorage temp(filename.str(),cv::FileStorage::WRITE);  
	temp << "Intrinsic" << intrinsicMatrix;
	temp << "Distortion" << distortionCoeffs;
	temp.release();
}

void CameraCalibrator::ReadIntrinsicDistortion()
{
	std::stringstream filename;
	filename<<"cap"<<device<<"\\"<<"Matrix.xml";
	cv::FileStorage temp(filename.str(),cv::FileStorage::READ);  
	if(temp.isOpened())
	{
		temp["Intrinsic"] >> intrinsicMatrix;
		temp["Distortion"] >> distortionCoeffs;
	}
	else
	{
		std::cout<<"Can't read "<<device<<"-th intrinsic and distortion matrix."<<std::endl;
	}
	temp.release();
}

#endif