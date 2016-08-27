#include <iostream>
#include "nanoflann.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <chrono>



template <typename T>
struct PointCloud
{

	std::vector<pcl::PointXYZ>  pts;

	// Must return the number of data points
	inline size_t kdtree_get_point_count() const { return pts.size(); }

	// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
	inline T kdtree_distance(const T *p1, const size_t idx_p2,size_t /*size*/) const
	{
		const T d0=p1[0]-pts[idx_p2].x;
		const T d1=p1[1]-pts[idx_p2].y;
		const T d2=p1[2]-pts[idx_p2].z;
		return d0*d0+d1*d1+d2*d2;
	}

	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate value, the
	//  "if/else's" are actually solved at compile time.
	inline T kdtree_get_pt(const size_t idx, int dim) const
	{
		if (dim==0) return pts[idx].x;
		else if (dim==1) return pts[idx].y;
		else return pts[idx].z;
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }

};

template <typename num_t>
class KDTree {
public:
	typedef nanoflann::KDTreeSingleIndexAdaptor<
	nanoflann::L2_Simple_Adaptor<num_t, PointCloud<num_t> > ,
	PointCloud<num_t>,
	3 /* dim */
	> my_kd_tree_t;

	KDTree() : cloud(), index(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */)) { };

	void Initialize(pcl::PointCloud<pcl::PointXYZ>::Ptr const& xyz_cloud_new) {
		using namespace std;
		using namespace nanoflann;
		cloud.pts.clear();

	  // Make a PointCloud from a pcl pointcloud
		size_t num_points = xyz_cloud_new->points.size();
		for (size_t i = 0; i < num_points; i++) {
			if ( !(xyz_cloud_new->points[i].x != xyz_cloud_new->points[i].x) ) {
				cloud.pts.push_back(xyz_cloud_new->points[i]);
			}
		}

	  // cloud.pts = xyz_cloud_new;
	  //std::cout << cloud.pts.size() << " is how many pts I ended up with" << std::endl;

	  // construct a kd-tree index:

	// index = my_kd_tree_t(3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );
	index.buildIndex();

	  //   std::cout << "knnSearch(nn="<<num_results<<"): \n";
	  //   std::cout << "ret_index=" << ret_index << " out_dist_sqr=" << out_dist_sqr << endl;
	  // }
	  // {
	  //   // Unsorted radius search:
	  //   const num_t radius = 1;
	  //   std::vector<std::pair<size_t,num_t> > indices_dists;
	  //   RadiusResultSet<num_t,size_t> resultSet(radius,indices_dists);

	  //   index.findNeighbors(resultSet, query_pt, nanoflann::SearchParams());

	  //   // Get worst (furthest) point, without sorting:
	  //   std::pair<size_t,num_t> worst_pair = resultSet.worst_item();
	  //   cout << "Worst pair: idx=" << worst_pair.first << " dist=" << worst_pair.second << endl;
	  // }

	}

template <int n>
void SearchForNearest(num_t x, num_t y, num_t z, std::vector<pcl::PointXYZ>& closest_pts, std::vector<num_t>& squared_distances) {
	if (cloud.pts.size() > 0) {

		closest_pts.clear();
		squared_distances.clear();

		num_t query_pt[3] = { x, y, z};

	    size_t ret_index[n];
	    num_t out_dist_sqr[n];
	    nanoflann::KNNResultSet<num_t> resultSet(n);
	    resultSet.init(&ret_index[0], &out_dist_sqr[0] );
	    nanoflann::SearchParams params(10);
	    index.findNeighbors(resultSet, &query_pt[0], params);

	    for (size_t i = 0; i < n; i++) {
	    	closest_pts.push_back(cloud.pts[ret_index[i]]);
	    	squared_distances.push_back(out_dist_sqr[i]);
	    }
	}

}

private:
	PointCloud<num_t> cloud;
	my_kd_tree_t index;
};