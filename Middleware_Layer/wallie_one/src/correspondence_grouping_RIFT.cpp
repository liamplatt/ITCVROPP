#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/point_types_conversion.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/rift.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/shot.h>
#include <pcl/features/board.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/rsd.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>

typedef pcl::PointXYZI PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::Histogram<32> DescriptorType;

std::string model_filename_;
std::string scene_filename_;

//Algorithm params
bool show_keypoints_ (false);
bool show_correspondences_ (false);
bool use_cloud_resolution_ (false);
bool use_hough_ (true);
float model_ss_ (0.01f);
float scene_ss_ (0.03f);
float rf_rad_ (0.015f);
float descr_rad_ (0.02f);
float cg_size_ (0.01f);
float cg_thresh_ (5.0f);
float descr_thresh_ (0.25f);
int descr_type_ (5);
int image_width_(20);

void
showHelp (char *filename)
{
  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*             Correspondence Grouping Tutorial - Usage Guide              *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd [Options]" << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     -h:                     Show this help." << std::endl;
  std::cout << "     -k:                     Show used keypoints." << std::endl;
  std::cout << "     -c:                     Show used correspondences." << std::endl;
  std::cout << "     -r:                     Compute the model cloud resolution and multiply" << std::endl;
  std::cout << "                             each radius given by that value." << std::endl;
  std::cout << "     --algorithm (Hough|GC): Clustering algorithm used (default Hough)." << std::endl;
  std::cout << "     --model_ss val:         Model uniform sampling radius (default 0.01)" << std::endl;
  std::cout << "     --scene_ss val:         Scene uniform sampling radius (default 0.03)" << std::endl;
  std::cout << "     --rf_rad val:           Reference frame radius (default 0.015)" << std::endl;
  std::cout << "     --descr_rad val:        Descriptor radius (default 0.02)" << std::endl;
  std::cout << "     --cg_size val:          Cluster size (default 0.01)" << std::endl;
  std::cout << "     --cg_thresh val:        Clustering threshold (default 5)"  << std::endl;
  std::cout << "     --descr_type val:        Descriptor type (default SHOT)" << std::endl;
  std::cout << "     --descr_thresh val:        Descriptor threshold (default .25)" << std::endl << std::endl;
}

void
parseCommandLine (int argc, char *argv[])
{
  //Show help
  if (pcl::console::find_switch (argc, argv, "-h"))
  {
    showHelp (argv[0]);
    exit (0);
  }

  //Model & scene filenames
  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  if (filenames.size () != 2)
  {
    std::cout << "Filenames missing.\n";
    showHelp (argv[0]);
    exit (-1);
  }

  model_filename_ = argv[filenames[0]];
  scene_filename_ = argv[filenames[1]];

  //Program behavior
  if (pcl::console::find_switch (argc, argv, "-k"))
  {
    show_keypoints_ = true;
  }
  if (pcl::console::find_switch (argc, argv, "-c"))
  {
    show_correspondences_ = true;
  }
  if (pcl::console::find_switch (argc, argv, "-r"))
  {
    use_cloud_resolution_ = true;
  }

  std::string used_algorithm;
  if (pcl::console::parse_argument (argc, argv, "--algorithm", used_algorithm) != -1)
  {
    if (used_algorithm.compare ("Hough") == 0)
    {
      use_hough_ = true;
    }else if (used_algorithm.compare ("GC") == 0)
    {
      use_hough_ = false;
    }
    else
    {
      std::cout << "Wrong algorithm name.\n";
      showHelp (argv[0]);
      exit (-1);
    }
  }

  //General parameters
  pcl::console::parse_argument (argc, argv, "--descr_type", descr_type_);
  pcl::console::parse_argument (argc, argv, "--model_ss", model_ss_);
  pcl::console::parse_argument (argc, argv, "--scene_ss", scene_ss_);
  pcl::console::parse_argument (argc, argv, "--rf_rad", rf_rad_);
  pcl::console::parse_argument (argc, argv, "--descr_rad", descr_rad_);
  pcl::console::parse_argument (argc, argv, "--cg_size", cg_size_);
  pcl::console::parse_argument (argc, argv, "--cg_thresh", cg_thresh_);
  pcl::console::parse_argument (argc, argv, "--descr_thresh", descr_thresh_);
  pcl::console::parse_argument (argc, argv, "--image_width", image_width_);
}

double
computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType> tree;
  tree.setInputCloud (cloud);

  for (std::size_t i = 0; i < cloud->size (); ++i)
  {
    if (! std::isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

int
main (int argc, char *argv[])
{
ros::Time::init();
  parseCommandLine (argc, argv);
	

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_in(new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_in(new pcl::PointCloud<pcl::PointXYZRGB> ());

  pcl::PointCloud<pcl::PointXYZI>::Ptr model (new pcl::PointCloud<pcl::PointXYZI> ());
  pcl::PointCloud<pcl::PointXYZI>::Ptr model_keypoints (new pcl::PointCloud<pcl::PointXYZI> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_keypoints_RGB(new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZI>::Ptr scene (new pcl::PointCloud<pcl::PointXYZI> ());
  pcl::PointCloud<pcl::PointXYZI>::Ptr scene_keypoints (new pcl::PointCloud<pcl::PointXYZI> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_keypoints_RGB(new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
// Object for storing the intensity gradients.
pcl::PointCloud<pcl::IntensityGradient>::Ptr model_gradients(new pcl::PointCloud<pcl::IntensityGradient>);
pcl::PointCloud<pcl::IntensityGradient>::Ptr scene_gradients(new pcl::PointCloud<pcl::IntensityGradient>);

  //
  //  Load clouds
  //
  if (pcl::io::loadPCDFile (model_filename_, *model_in) < 0)
  {
    std::cout << "Error loading model cloud." << std::endl;
    showHelp (argv[0]);
    return (-1);
  }
  if (pcl::io::loadPCDFile (scene_filename_, *scene_in) < 0)
  {
    std::cout << "Error loading scene cloud." << std::endl;
    showHelp (argv[0]);
    return (-1);
  }

  //
  //  Set up resolution invariance
  //
  if (use_cloud_resolution_)
  {
    float resolution = static_cast<float> (computeCloudResolution (model));
    if (resolution != 0.0f)
    {
      model_ss_   *= resolution;
      scene_ss_   *= resolution;
      rf_rad_     *= resolution;
      descr_rad_  *= resolution;
      cg_size_    *= resolution;
    }

    std::cout << "Model resolution:       " << resolution << std::endl;
    std::cout << "Model sampling size:    " << model_ss_ << std::endl;
    std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
    std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
    std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
    std::cout << "SHOT descriptor threshold: " << descr_thresh_ << std::endl;
    std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
  }

		double start = ros::Time::now().toSec();
 
	// Convert to XYZI from XYZRGB
	pcl::PointCloudXYZRGBtoXYZI(*model_in, *model);
	pcl::PointCloudXYZRGBtoXYZI(*scene_in, *scene);
  //
  //  Downsample Clouds to Extract keypoints
  //

  pcl::UniformSampling<PointType> uniform_sampling;
  uniform_sampling.setInputCloud (model);
  uniform_sampling.setRadiusSearch (model_ss_);
  uniform_sampling.filter (*model_keypoints);
  std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

  uniform_sampling.setInputCloud (scene);
  uniform_sampling.setRadiusSearch (scene_ss_);
  uniform_sampling.filter (*scene_keypoints);
  std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;


  //  Compute Normals
  //
  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
 norm_est.setKSearch (10);
  norm_est.setInputCloud (model_keypoints);
  norm_est.compute (*model_normals);

  norm_est.setInputCloud (scene_keypoints);
  norm_est.compute (*scene_normals);
  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
 // Compute intensity gradient

pcl::IntensityGradientEstimation <PointType, pcl::Normal, pcl::IntensityGradient, pcl::common::IntensityFieldAccessor<PointType>> grad_est;

grad_est.setInputCloud(model_keypoints);
grad_est.setInputNormals(model_normals);
grad_est.setKSearch(10);
grad_est.compute(*model_gradients);

grad_est.setInputCloud(scene_keypoints);
grad_est.setInputNormals(scene_normals);
grad_est.setKSearch(10);
grad_est.compute(*scene_gradients);

  //
  //  Compute Descriptor for keypoints
  //	
	pcl::RIFTEstimation<PointType, pcl::IntensityGradient, DescriptorType> rift;

	pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
  	pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
  	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());


	rift.setInputCloud(model_keypoints);
	rift.setInputGradient(model_gradients);
	rift.setSearchMethod(tree);
	// Radius, to get all neighbors within.
	rift.setRadiusSearch(descr_rad_);
	// Set the number of bins to use in the distance dimension.
	rift.setNrDistanceBins(4);
	// Set the number of bins to use in the gradient orientation dimension.
	rift.setNrGradientBins(8);
	rift.compute(*model_descriptors);


	rift.setInputCloud(scene_keypoints);
	rift.setInputGradient(scene_gradients);
	rift.setSearchMethod(tree);
	// Radius, to get all neighbors within.
	rift.setRadiusSearch(descr_rad_);
	// Set the number of bins to use in the distance dimension.
	rift.setNrDistanceBins(4);
	// Set the number of bins to use in the gradient orientation dimension.
	rift.setNrGradientBins(8);
	rift.compute(*scene_descriptors);



  //
  //  Find Model-Scene Correspondences with KdTree
  //

  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud (model_descriptors);
  std::cout << "Scene Descriptors found: " << scene_descriptors->size () << std::endl;
  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
  for (std::size_t i = 0; i < scene_descriptors->size (); ++i)
  {
    std::vector<int> neigh_indices (1);
    std::vector<float> neigh_sqr_dists (1);
    int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
  	std::cout << "Processing Scene Descriptor " << i << "  found " << found_neighs << "neighbors @ dist " << neigh_sqr_dists[0] << std::endl;
    if(found_neighs >= 1 && neigh_sqr_dists[0] <= descr_thresh_) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    {
      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      model_scene_corrs->push_back (corr);
    }
  }
  std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;


  //
  //  Actual Clustering
  //
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector<pcl::Correspondences> clustered_corrs;

  //  Using GC
    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    gc_clusterer.setGCSize (cg_size_);
    gc_clusterer.setGCThreshold (cg_thresh_);

    gc_clusterer.setInputCloud (model_keypoints);
    gc_clusterer.setSceneCloud (scene_keypoints);
    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

    gc_clusterer.cluster (clustered_corrs);
    gc_clusterer.recognize (rototranslations, clustered_corrs);


  //  
  //  Output results
  //  
  std::cout << "Model instances found: " << rototranslations.size () << std::endl;
  for (std::size_t i = 0; i < rototranslations.size (); ++i)
  {
    std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
    std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
    Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

    printf ("\n");
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    printf ("\n");
    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  }

  //
  //  Visualization
  //
double end = ros::Time::now().toSec();
ROS_INFO("Correspondence grouping processed %d correspondences in: %f sec.\n",clustered_corrs.size (), end-start);
  pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
  viewer.addPointCloud (scene_in, "scene_cloud");

  pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

  if (show_correspondences_ || show_keypoints_)
  {
    //  We are translating the model so that it doesn't end in the middle of the scene representation
    pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
    pcl::transformPointCloud (*model, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
    viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
  }

  if (show_keypoints_)
  {
    pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
    viewer.addPointCloud (scene, scene_keypoints_color_handler, "scene_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
    viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
  }

  for (std::size_t i = 0; i < rototranslations.size (); ++i)
  {
    pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
    pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

    std::stringstream ss_cloud;
    ss_cloud << "instance" << i;

    pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
    viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

    if (show_correspondences_)
    {
      for (std::size_t j = 0; j < clustered_corrs[i].size (); ++j)
      {
        std::stringstream ss_line;
        ss_line << "correspondence_line" << i << "_" << j;
        PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
        PointType& scene_point = scene->at (clustered_corrs[i][j].index_match);

        //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
        viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
      }
    }
  }

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }

  return (0);
}
