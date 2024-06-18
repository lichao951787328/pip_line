/*
 * @description: 
 * @param : 
 * @return: 
 */
#ifndef _POINT_TYPE_H_
#define _POINT_TYPE_H_
#include <vector>
#include <Eigen/Core>
typedef std::pair< std::pair<size_t, size_t>, Eigen::Vector3f>  IndexPoint;

typedef std::vector<IndexPoint> IndexPoints;

#endif