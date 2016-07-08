#ifndef RIGHT_HAND_RULE_H_INCLUDED
#define RIGHT_HAND_RULE_H_INCLUDED

#include <opencv2/opencv.hpp>
#include "calibrate.h"

/**<  */
/** \brief adjust vanishing point such that z-axis is upward
 *
 * \param _v1 is input of vanishing point v1
 * \param _v2 is input of vanishing point v2
 * \param origin pixel position in image correspond to object space.
 * \param v1 is output of vanishing point v1
 * \param v2 is output of vanishing point v2
 * \return void
 *
 */

bool adjust_vanishing_point(const cv::Point2f  _v1, const cv::Point2f _v2, const cv::Point2f origin, cv::Point2f &v1, cv::Point2f &v2);
void adjustInteraction(cv::Point2f src_point[11], cv::Point2f dst_point[11]);
#endif // RIGHT_HAND_RULE_H_INCLUDED
