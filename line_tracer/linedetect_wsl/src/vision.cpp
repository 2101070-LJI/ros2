#include "subsub/vision.hpp"

CamSubNode::CamSubNode() 
: Node("camsub_wsl7"), 
  previousCenter_(-1, -1), 
  firstFrame_(true), 
  previousArea_(0) // 면적 0으로 시작
{
    namedWindow("Input Source", WINDOW_AUTOSIZE);
    namedWindow("Binary Tracking", WINDOW_AUTOSIZE);
}

CamSubNode::~CamSubNode()
{
    destroyAllWindows();
}

void CamSubNode::preprocess(const Mat& source)
{
    Rect roi(0, source.rows * 3 / 4, source.cols, source.rows / 4);
    Mat color_roi = source(roi);
    cvtColor(color_roi, gray_, COLOR_BGR2GRAY);

    cv::Scalar m1 = cv::mean(gray_);
    gray_ = gray_ + (100 - m1[0]);

    threshold(gray_, binary_roi_, 0, 255, THRESH_BINARY | THRESH_OTSU);
    cvtColor(binary_roi_, view_roi_, COLOR_GRAY2BGR);
}

double CamSubNode::process_tracking()
{
    int nLabels = connectedComponentsWithStats(binary_roi_, labelImage_, stats_, centroids_, 8, CV_32S);

    vector<Point> lineCenters;
    vector<int> lineAreas; // 면적 정보도 같이 저장

    for (int i = 1; i < nLabels; i++) {
        int area = stats_.at<int>(i, CC_STAT_AREA); // 면적 가져오기
        
        // 작은 노이즈 무시
        if (area < 50) continue; 

        int x = stats_.at<int>(i, CC_STAT_LEFT);
        int y = stats_.at<int>(i, CC_STAT_TOP);
        int width = stats_.at<int>(i, CC_STAT_WIDTH);
        int height = stats_.at<int>(i, CC_STAT_HEIGHT);

        Point center(centroids_.at<double>(i, 0), centroids_.at<double>(i, 1));
        
        // 후보군 등록
        lineCenters.push_back(center);
        lineAreas.push_back(area);

        // 시각화
        rectangle(view_roi_, Rect(x, y, width, height), Scalar(255, 0, 0), 2);
        circle(view_roi_, center, 5, Scalar(255, 0, 0), -1);
    }

    const double MAX_DISTANCE = 100.0;
    double error = 0.0;
    Point centerOfImage(binary_roi_.cols / 2, binary_roi_.rows / 2);

    // 처음
    if (firstFrame_ && !lineCenters.empty()) {
        double minDistance = DBL_MAX;
        int bestIdx = -1;

        for (size_t i = 0; i < lineCenters.size(); i++) {
            double distance = norm(lineCenters[i] - centerOfImage);
            if (distance < minDistance) {
                minDistance = distance;
                bestIdx = i;
            }
        }
        
        if (bestIdx != -1) {
            previousCenter_ = lineCenters[bestIdx];
            previousArea_ = lineAreas[bestIdx]; // 찾은 라인의 면적 기억
            firstFrame_ = false;
            
            error = centerOfImage.x - previousCenter_.x;
            rectangle(view_roi_, Rect(previousCenter_.x - 10, previousCenter_.y - 10, 20, 20), Scalar(0, 0, 255), 2);
        }
    }
    // 두번째
    else if (previousCenter_.x != -1 && !lineCenters.empty()) {
        double minDistance = DBL_MAX;
        int bestIdx = -1;
        bool found = false;

        for (size_t i = 0; i < lineCenters.size(); i++) {
            double distance = norm(lineCenters[i] - previousCenter_);
            
            // 이전 크기의 0.5배 ~ 5배 사이여야 인정
            bool isSizeSimilar = (lineAreas[i] > previousArea_ * 0.5) && (lineAreas[i] < previousArea_ * 5);

            if (distance < minDistance && distance < MAX_DISTANCE && isSizeSimilar) {
                minDistance = distance;
                bestIdx = i;
                found = true;
            }
        }

        if (found) {
            previousCenter_ = lineCenters[bestIdx];
            previousArea_ = lineAreas[bestIdx]; // 현재 라인 면적으로 업데이트
            error = centerOfImage.x - previousCenter_.x;
            
            rectangle(view_roi_, Rect(previousCenter_.x - 10, previousCenter_.y - 10, 20, 20), Scalar(0, 0, 255), 2);
        } else {
            // 못 찾았으면 이전 위치 유지
            rectangle(view_roi_, Rect(previousCenter_.x - 10, previousCenter_.y - 10, 20, 20), Scalar(0, 0, 255), 2);
        }
    }
    
    return error;
}

void CamSubNode::display_result(const Mat& source)
{
    imshow("Input Source", source);
    imshow("Binary Tracking", view_roi_);
    waitKey(1);
}
