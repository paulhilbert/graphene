/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


inline std::vector<Eigen::Vector3f> boundingBoxLinePoints(const Eigen::AlignedBox<float, 3>& box) {
	Eigen::Vector3f min = box.min();
	Eigen::Vector3f max = box.max();
	std::vector<Eigen::Vector3f> c(8);
	c[0] << min[0], min[1], min[2];
	c[1] << max[0], min[1], min[2];
	c[2] << max[0], max[1], min[2];
	c[3] << min[0], max[1], min[2];
	c[4] << min[0], min[1], max[2];
	c[5] << max[0], min[1], max[2];
	c[6] << max[0], max[1], max[2];
	c[7] << min[0], max[1], max[2];
	return { c[0], c[1], c[1], c[2], c[2], c[3], c[3], c[0],
	         c[4], c[5], c[5], c[6], c[6], c[7], c[7], c[4],
				c[0], c[4], c[3], c[7],
				c[1], c[5], c[2], c[6]
	       };
	/*
	std::vector<Eigen::Vector3f> result;
	result.push_back( Eigen::Vector3f( c0 ) );
	result.push_back( Eigen::Vector3f( c1 ) );
	result.push_back( Eigen::Vector3f( c1 ) );
	result.push_back( Eigen::Vector3f( c2 ) );
	result.push_back( Eigen::Vector3f( c2 ) );
	result.push_back( Eigen::Vector3f( c3 ) );
	result.push_back( Eigen::Vector3f( c3 ) );
	result.push_back( Eigen::Vector3f( c0 ) );
	*/
}
