#include "subsub/vision.hpp"

CamSubNode::CamSubNode() 
: Node("camsub_wsl7"), 
  previousCenter_(-1, -1),
  firstFrame_(true)
{
    // 생성자에서는 이제 OpenCV 창만 띄웁니다.
    // 구독(Subscribe)은 여기서 하지 않습니다.
    namedWindow("Original", WINDOW_AUTOSIZE);
    namedWindow("Binary Tracking", WINDOW_AUTOSIZE);
}

CamSubNode::~CamSubNode()
{
    destroyAllWindows();
}

// --------------------------------------------------------------------------
// 아래 함수들은 로직은 그대로지만, mysub_callback 없이 독립적으로 존재합니다.
// --------------------------------------------------------------------------

void CamSubNode::preprocess(const Mat& source, Mat& binary_out, Mat& view_out)
{
    Mat gray;
    cvtColor(source, gray, COLOR_BGR2GRAY);

    Rect roi(0, gray.rows * 3 / 4, gray.cols, gray.rows / 4);
    Mat resizedGray = gray(roi);

    threshold(resizedGray, binary_out, 0, 255, THRESH_BINARY | THRESH_OTSU);
    cvtColor(binary_out, view_out, COLOR_GRAY2BGR);
}

double CamSubNode::process_tracking(const Mat& binary_roi, Mat& view_roi)
{
    Mat labelImage, stats, centroids;
    int nLabels = connectedComponentsWithStats(binary_roi, labelImage, stats, centroids, 8, CV_32S);

    vector<Point> lineCenters;
    for (int i = 1; i < nLabels; i++) {
        int x = stats.at<int>(i, CC_STAT_LEFT);
        int y = stats.at<int>(i, CC_STAT_TOP);
        int width = stats.at<int>(i, CC_STAT_WIDTH);
        int height = stats.at<int>(i, CC_STAT_HEIGHT);

        Point center(centroids.at<double>(i, 0), centroids.at<double>(i, 1));
        lineCenters.push_back(center);

        rectangle(view_roi, Rect(x, y, width, height), Scalar(255, 0, 0), 2);
        circle(view_roi, center, 5, Scalar(255, 0, 0), -1);
    }

    const double MAX_DISTANCE = 50.0;
    double error = 0.0;
    Point centerOfImage(binary_roi.cols / 2, binary_roi.rows / 2);

    if (firstFrame_ && !lineCenters.empty()) {
        double minDistance = DBL_MAX;
        Point closestCenter;
        for (const auto& center : lineCenters) {
            double distance = norm(center - centerOfImage);
            if (distance < minDistance) {
                minDistance = distance;
                closestCenter = center;
            }
        }
        previousCenter_ = closestCenter;
        firstFrame_ = false;
        
        rectangle(view_roi, Rect(closestCenter.x - 10, closestCenter.y - 10, 20, 20), Scalar(0, 0, 255), 2);
        error = centerOfImage.x - closestCenter.x;
    }
    else if (previousCenter_.x != -1 && !lineCenters.empty()) {
        double minDistance = DBL_MAX;
        Point closestCenter;
        bool found = false;
        for (const auto& center : lineCenters) {
            double distance = norm(center - previousCenter_);
            if (distance < minDistance && distance < MAX_DISTANCE) {
                minDistance = distance;
                closestCenter = center;
                found = true;
            }
        }
        if (found) {
            previousCenter_ = closestCenter;
            error = centerOfImage.x - closestCenter.x;
            rectangle(view_roi, Rect(closestCenter.x - 10, closestCenter.y - 10, 20, 20), Scalar(0, 0, 255), 2);
        } else {
            rectangle(view_roi, Rect(previousCenter_.x - 10, previousCenter_.y - 10, 20, 20), Scalar(0, 0, 255), 2);
        }
    }
    return error;
}

void CamSubNode::display_result(const Mat& original, const Mat& result_view)
{
    imshow("Original", original);
    imshow("Binary Tracking", result_view);
    waitKey(1);
}