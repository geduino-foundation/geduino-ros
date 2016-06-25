#ifndef CV_CONVERSION_H_
#define CV_CONVERSION_H_

#include <opencv2/core/core.hpp>
#include <zxing/ResultPoint.h>
#include <zxing/common/Array.h>

cv::Point toCvPoint(const zxing::Ref<zxing::ResultPoint> & resultPoint) {
    return cv::Point(resultPoint->getX(), resultPoint->getY());
}

cv::Point2f toCvPoint2f(const zxing::Ref<zxing::ResultPoint> & resultPoint) {
    return cv::Point2f(resultPoint->getX(), resultPoint->getY());
}

cv::Mat toCvMat3x3(const boost::array<double, 9> & array9) {

    // Create cv mat 3x3
    cv::Mat mat3x3(3, 3, cv::DataType<double>::type);

    for (int i = 0; i < 3; i++) {

        for (int j = 0; j < 3; j++) {

            // Set value
            mat3x3.at<double>(j, i) = array9[i * 3 + j];

        }

    }

    return mat3x3;

}

cv::Mat toCvMat5x1(const std::vector<double> & vector5) {

    // Create cv mat5x1
    cv::Mat mat5x1(5, 1, cv::DataType<double>::type);

    for (int i = 0; i < 5; i++) {

        // Set value
        mat5x1.at<double>(i) = vector5.at(i);

    }


    return mat5x1;

}

std::vector<cv::Point3f> toCvPoint3fVector(const std::vector<double> & doubleVector) {

    // Get number of points
    int pointCount = doubleVector.size() / 3;

    // Create point3f vector
    std::vector<cv::Point3f> point3fVector;

    for (int index = 0; index < pointCount; index++) {

        // Create point3f
        cv::Point3f point3f(doubleVector.at(index * 3), doubleVector.at(index * 3 + 1), doubleVector.at(index * 3 + 2));

        // Add point3f to point3f vector
        point3fVector.push_back(point3f);

    }

    return point3fVector;

}

std::vector<cv::Point2f> toCvPoint2fVector(const zxing::ArrayRef<zxing::Ref<zxing::ResultPoint> > & resultPoints, int limit) {

    // Create point2f vector
    std::vector<cv::Point2f> point2fVector;

    for (int index = 0; index < std::min(resultPoints->size(), limit); index++) {

        // Convert result point to point2
        cv::Point2f point2f = toCvPoint2f(resultPoints[index]);

        // Add to point2f vector
        point2fVector.push_back(point2f);

    }

    return point2fVector;

}

#endif
