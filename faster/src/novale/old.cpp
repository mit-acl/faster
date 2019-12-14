/*    mtx_X_U.lock();
    int my_length = std::min(deltaT, (int)(U_.rows() - (k_ + 1)));
    my_length = (my_length < 1) ? 1 : my_length;
    Eigen::MatrixXd jerk_simulate = U_.block(k_ + 1, 0, my_length, U_.cols());
    Eigen::Vector3d vicon_accel = X_.block(k_, 6, 1, 3).transpose();  // TODO: Take this from the IMU in the real HW

    std::vector<Eigen::Vector3d> simulated;

    mtx_X_U.unlock();

    std::cout << "****STARTING FROM:" << std::endl;
    std::cout << "pos:" << state_pos.transpose() << std::endl;
    std::cout << "vel:" << state_vel.transpose() << std::endl;
    std::cout << "accel:" << vicon_accel.transpose() << std::endl;

    std::cout << "Y aplicando el jerk:" << std::endl;
    std::cout << "jerk:" << jerk_simulate << std::endl;

    simulated = simulateForward(state_pos, state_vel, vicon_accel,
                                jerk_simulate);  // produces the simulated state deltaT + 1*/

/*
mtx_X_U.lock();

Eigen::Vector3d novale_pos = (X_.block(k_, 0, 1, 3)).transpose();
Eigen::Vector3d novale_vel = (X_.block(k_, 3, 1, 3)).transpose();
Eigen::Vector3d novale_accel = (X_.block(k_, 6, 1, 3)).transpose();
mtx_X_U.unlock();

 std::cout << "STARTING FROM:" << std::endl;
    std::cout << "pos:" << novale_pos.transpose() << std::endl;
    std::cout << "vel:" << novale_vel.transpose() << std::endl;
    std::cout << "accel:" << novale_accel.transpose() << std::endl;

    std::cout << "Y aplicando el jerk:" << std::endl;
    std::cout << "jerk:" << jerk_simulate << std::endl;

    simulated = simulateForward(novale_pos, novale_vel, novale_accel,
                                jerk_simulate);  // produces the simulated state deltaT + 1

    // simulated = simulateForward(state_pos, state_vel, vicon_accel, jerk_simulate);
    std::cout << "SIMULATED:" << std::endl;
    std::cout << simulated[0].transpose() << " " << simulated[1].transpose() << " " << simulated[2].transpose()
              << std::endl;

    mtx_X_U.lock();
    std::cout << "ME DEBERIA DAR:" << std::endl;
    std::cout << X_.block(k_, 0, deltaT + 2, X_.cols()) << std::endl;
    mtx_X_U.unlock();

    mtx_initial_cond.lock();
    std::cout << "IVE TAKEN AS INITIAL CONDITION:" << std::endl;
    std::cout << stateA_.pos.x << ", " << stateA_.pos.y << ", " << stateA_.pos.z << ", "
              << stateA_.vel.x << ", " << stateA_.vel.y << ", " << stateA_.vel.z << ", "
              << stateA_.accel.x << ", " << stateA_.accel.y << ", " << stateA_.accel.z << std::endl;
    mtx_initial_cond.unlock();*/
//     double x0[9] = { (simulated[0])(0), (simulated[0])(1), (simulated[0])(2),  ////////
//                  (simulated[1])(0), (simulated[1])(1), (simulated[1])(2),  ////////
//                  (simulated[2])(0), (simulated[2])(1), (simulated[2])(2) };

// mtx_k.unlock();
/* std::cout << "SIMULATED:" << std::endl;
std::cout << simulated[0].transpose() << " " << simulated[1].transpose() << " " << simulated[2].transpose()
          << std::endl;
x0[0] = (simulated[0])(0);  // Pos
x0[1] = (simulated[0])(1);  // Pos
x0[2] = (simulated[0])(2);  // Pos
x0[3] = (simulated[1])(0);  // Vel
x0[4] = (simulated[1])(1);  // Vel
x0[5] = (simulated[1])(2);  // Vel
x0[6] = (simulated[2])(0);  // Accel
x0[7] = (simulated[2])(1);  // Accel
x0[8] = (simulated[2])(2);  // Accel

std::cout << "here4" << std::endl;*/

/*void CVX::pubPlanningVisual(Eigen::Vector3d center, double ra, double rb, Eigen::Vector3d B1, Eigen::Vector3d C1)
{
  visualization_msgs::MarkerArray tmp;

  int start = 2200;  // Large enough to prevent conflict with other markers
  visualization_msgs::Marker sphere_Sa;
  sphere_Sa.header.frame_id = "world";
  sphere_Sa.id = start;
  sphere_Sa.type = visualization_msgs::Marker::SPHERE;
  sphere_Sa.scale = vectorUniform(2 * ra);
  sphere_Sa.color = color(BLUE_TRANS);
  sphere_Sa.pose.position = eigen2point(center);
  tmp.markers.push_back(sphere_Sa);

  visualization_msgs::Marker sphere_Sb;
  sphere_Sb.header.frame_id = "world";
  sphere_Sb.id = start + 1;
  sphere_Sb.type = visualization_msgs::Marker::SPHERE;
  sphere_Sb.scale = vectorUniform(2 * rb);
  sphere_Sb.color = color(RED_TRANS_TRANS);
  sphere_Sb.pose.position = eigen2point(center);
  tmp.markers.push_back(sphere_Sb);

  visualization_msgs::Marker B1_marker;
  B1_marker.header.frame_id = "world";
  B1_marker.id = start + 2;
  B1_marker.type = visualization_msgs::Marker::SPHERE;
  B1_marker.scale = vectorUniform(0.1);
  B1_marker.color = color(BLUE_LIGHT);
  B1_marker.pose.position = eigen2point(B1);
  tmp.markers.push_back(B1_marker);

  visualization_msgs::Marker C1_marker;
  C1_marker.header.frame_id = "world";
  C1_marker.id = start + 3;
  C1_marker.type = visualization_msgs::Marker::SPHERE;
  C1_marker.scale = vectorUniform(0.1);
  C1_marker.color = color(BLUE_LIGHT);
  C1_marker.pose.position = eigen2point(C1);
  tmp.markers.push_back(C1_marker);

  pub_planning_vis_.publish(tmp);
}*/

/*void CVX::frontierCB(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg)
{
  // printf("****In FrontierCB\n");
  if (pcl2ptr_msg->width == 0 || pcl2ptr_msg->height == 0)  // Point Cloud is empty (this happens at the beginning)
  {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr_frontier(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pcl2ptr_msg, *pclptr_frontier);

  mtx_frontier.lock();
  kdtree_frontier_.setInputCloud(pclptr_frontier);
  mtx_frontier.unlock();
  kdtree_frontier_initialized_ = 1;
}*/

/*void CVX::pclCB(const sensor_msgs::PointCloud2ConstPtr& pcl2ptr_msg)
{
  // printf("In pclCB\n");

  if (pcl2ptr_msg->width == 0 || pcl2ptr_msg->height == 0)  // Point Cloud is empty
  {
    return;
  }
  geometry_msgs::TransformStamped transformStamped;
  sensor_msgs::PointCloud2Ptr pcl2ptr_msg_transformed(new sensor_msgs::PointCloud2());
  try
  {
    transformStamped = tf_buffer_.lookupTransform("world", name_drone_ + "/camera", ros::Time(0));
    tf2::doTransform(*pcl2ptr_msg, *pcl2ptr_msg_transformed, transformStamped);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pclptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pcl2ptr_msg_transformed, *pclptr);

    std::vector<int> index;
    // TODO: there must be a better way to check this. It's here because (in the simulation) sometimes all the points
    // are NaN (when the drone is on the ground and stuck moving randomly). If this is not done, the program breaks. I
    // think it won't be needed in the real drone
    pcl::removeNaNFromPointCloud(*pclptr, *pclptr, index);
    if (pclptr->size() == 0)
    {
      return;
    }

    kdTreeStamped my_kdTreeStamped;
    my_kdTreeStamped.kdTree.setInputCloud(pclptr);
    my_kdTreeStamped.time = pcl2ptr_msg->header.stamp;
    mtx_inst.lock();
    // printf("pclCB: MTX is locked\n");
    v_kdtree_new_pcls_.push_back(my_kdTreeStamped);
    // printf("pclCB: MTX is unlocked\n");
    mtx_inst.unlock();
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
}*/

/*void CVX::cvxSeedDecompUnkOcc(Vecf<3>& seed)
{
  double before = ros::Time::now().toSec();

  // std::cout << "In cvxDecomp 0!" << std::endl;
  if (kdtree_map_initialized_ == false)
  {
    return;
  }
  // std::cout << "In cvxDecomp 1!" << std::endl;

  // vec_Vec3f obs;

  // std::cout << "Type Obstacles==UNKOWN_AND_OCCUPIED_SPACE**************" << std::endl;

  // pcl::KdTreeFLANN<pcl::PointXYZ>::PointCloudConstPtr ptr_cloud_map = kdtree_map_.getInputCloud();
  // pcl::KdTreeFLANN<pcl::PointXYZ>::PointCloudConstPtr ptr_cloud_unkown = kdtree_unk_.getInputCloud();

  // std::cout << "Number of elements in unkCloud" < < < < std::endl;

 // obs = pclptr_to_vec(pclptr_map_, pclptr_unk_);
 //   std::cout << "Points in mapCloud=" << (*pclptr_map_).points.size() << std::endl;
 //   std::cout << "Points in unkCloud=" << (*pclptr_unk_).points.size() << std::endl;

  // Initialize SeedDecomp3D

  seed_decomp_util_.set_seed(seed);
  seed_decomp_util_.set_obs(vec_uo_);
  seed_decomp_util_.set_local_bbox(Vec3f(4, 4, 1));
  // std::cout << "In cvxDecomp before dilate!" << std::endl;
  seed_decomp_util_.dilate(0.1);
  // std::cout << "In cvxDecomp after dilate!" << std::endl;
  seed_decomp_util_.shrink_polyhedron(par_.drone_radius);
  // std::cout << "In cvxDecomp after shrink!" << std::endl;

  vec_E<Polyhedron<3>> polyhedron_as_array;  // This vector will contain only one element
  polyhedron_as_array.push_back(seed_decomp_util_.get_polyhedron());
  decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polyhedron_as_array);
  poly_msg.header.frame_id = "world";

  // Publish visualization msgs
  cvx_decomp_poly_uo_pub_.publish(poly_msg);
  auto poly = seed_decomp_util_.get_polyhedron();
  // std::cout << "Incluso antes A es\n" << poly.hyperplanes()[0].n_.transpose() << std::endl;
  l_constraints_uo_.clear();
  LinearConstraint3D cs(seed, poly.hyperplanes());

  MatDNf<3> A = cs.A();
  // std::cout << "Incluso antes A es\n" << A << std::endl;

  l_constraints_uo_.push_back(cs);
  // ROS_WARN("SeedDecomp takes: %0.2f ms", 1000 * (ros::Time::now().toSec() - before));
}*/

void CVX::pubintersecPoint(Eigen::Vector3d p, bool add)
{
  static int i = 0;
  static int last_id = 0;
  int start = 300000;  // large enough to prevent conflict with others
  if (add)
  {
    visualization_msgs::Marker tmp;
    tmp.header.frame_id = "world";
    tmp.id = start + i;
    tmp.type = visualization_msgs::Marker::SPHERE;
    tmp.scale = vectorUniform(0.1);
    tmp.color = color(BLUE_LIGHT);
    tmp.pose.position = eigen2point(p);
    intersec_points_.markers.push_back(tmp);
    last_id = start + i;
    i = i + 1;
  }
  else
  {  // clear everything
    intersec_points_.markers.clear();
    /*    for (int j = start; j <= start; j++)
        {*/
    visualization_msgs::Marker tmp;
    tmp.header.frame_id = "world";
    tmp.id = start;
    tmp.type = visualization_msgs::Marker::SPHERE;
    tmp.action = visualization_msgs::Marker::DELETEALL;
    intersec_points_.markers.push_back(tmp);
    /*    }
     */
    last_id = 0;
    i = 0;
  }
  pub_intersec_points_.publish(intersec_points_);
  if (!add)
  {
    intersec_points_.markers.clear();
  }
}

std::vector<Eigen::Vector3d> simulateForward(Eigen::Vector3d& pos_init, Eigen::Vector3d& vel_init,
                                             Eigen::Vector3d& accel_init, Eigen::MatrixXd& jerk_sent);

std::vector<Eigen::Vector3d> CVX::simulateForward(Eigen::Vector3d& pos_init, Eigen::Vector3d& vel_init,
                                                  Eigen::Vector3d& accel_init, Eigen::MatrixXd& jerk_sent)
{
  double t = 0.01;

  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d accel;
  Eigen::Vector3d jerk;

  Eigen::Vector3d pos0 = pos_init;
  Eigen::Vector3d vel0 = vel_init;
  Eigen::Vector3d accel0 = accel_init;

  std::cout << "SIMULATED ONLINE!:" << std::endl;
  std::cout << pos0.transpose() << "  " << vel0.transpose() << "  " << accel0.transpose() << std::endl;
  std::cout << "going to simulate forward,jerk_sent.rows()=" << jerk_sent.rows() << std::endl;

  for (int j = 0; j < jerk_sent.rows(); j++)
  {
    // std::cout << "Simulating forward" << std::endl;
    jerk = jerk_sent.row(j).transpose();
    pos = (1 / 6.0) * jerk * t * t * t + accel0 * t * t / 2.0 + vel0 * t + pos0;
    /*    std::cout << "accel0 * t" << (accel0 * t).transpose() << std::endl;
        std::cout << "vel0" << (vel0).transpose() << std::endl;

        std::cout << "accel0 * t" << (accel0 * t).transpose() << std::endl;*/

    vel = (1 / 2.0) * jerk * t * t + accel0 * t + vel0;
    accel = jerk * t + accel0;

    pos0 = pos;
    accel0 = accel;
    vel0 = vel;

    std::cout << pos.transpose() << "  " << vel.transpose() << "  " << accel.transpose() << std::endl;
  }

  std::vector<Eigen::Vector3d> result;
  result.push_back(pos);
  result.push_back(vel);
  result.push_back(accel);
  return result;
}

Eigen::Vector3d getIntersectionJPSwithPolytope(vec_Vecf<3>& path, std::vector<LinearConstraint3D>& constraints,
                                               bool& thereIsIntersection);

// Compute the intersection of JPS with the first polytope of the vector "constraints" (each element of "constraints"
// represents a polytope)
Eigen::Vector3d CVX::getIntersectionJPSwithPolytope(vec_Vecf<3>& path, std::vector<LinearConstraint3D>& constraints,
                                                    bool& thereIsIntersection)
{
  // std::cout << "**IntersectionF:Path given=" << std::endl;
  // printElementsOfJPS(path);
  LinearConstraint3D constraint = constraints[0];
  // Each element of cs_vector is a pair (A,b) representing a polytope
  bool there_is_intersection = false;
  int last_id_inside = 0;
  // std::cout << "Inside Finding intersection" << std::endl;
  for (size_t i = 0; i < path.size(); i++)
  {
    if (constraint.inside(path[i]) == false)  // If a vertex of the path is not in JPS
    {
      there_is_intersection = true;
      break;
    }
    else
    {
      last_id_inside = i;
    }
  }

  // std::cout << "**IntersectionF: there_is_intersection= " << there_is_intersection << std::endl;

  thereIsIntersection = there_is_intersection;
  if (there_is_intersection == false)
  {  // If no intersection, return last point in the path
    return path[path.size() - 1];
  }

  // std::cout << "Out Looop 2" << std::endl;

  int n_of_faces = constraint.b().rows();
  MatDNf<3> A = constraint.A();
  VecDf b = constraint.b();

  /*  for (int m = 0; m < b.size(); m++)
    {
      std::cout << "b[m] is" << b[m] << std::endl;
    }

    for (int m = 0; m < A.rows(); m++)
    {
      std::cout << "A[m] is " << A.row(m) << std::endl;
    }*/
  // std::cout << "A directamente es\n" << A << std::endl;

  Eigen::Vector3d inters;
  /*  std::cout << "last_id_inside=" << last_id_inside << std::endl;
    std::cout << "Num of el in path " << path.size() << std::endl;
    std::cout << "Number of faces " << n_of_faces << std::endl;*/

  int j = 0;
  vec_Vecf<3> intersections;
  for (size_t j = 0; j < n_of_faces; j++)
  {
    bool intersection_with_this_face = false;
    Eigen::Vector4d coeff;
    Eigen::Vector3d normal = A.row(j);  // normal vector
    coeff << normal(0), normal(1), normal(2), -b(j);
    // std::cout << "j=" << j << std::endl;

    intersection_with_this_face =
        getIntersectionWithPlane(path[last_id_inside], path[last_id_inside + 1], coeff, inters);
    // std::cout << "j despues=" << j << std::endl;
    if (intersection_with_this_face == true)
    {
      intersections.push_back(inters);

      // break;
    }
  }

  if (intersections.size() == 0)
  {  // There is no intersection
    ROS_ERROR("This is impossible, there should be an intersection");
  }

  std::vector<double> distances;

  // And now take the nearest intersection
  for (size_t i = 0; i < intersections.size(); i++)
  {
    double distance = (intersections[i] - path[0]).norm();
    distances.push_back(distance);
  }
  int minElementIndex = std::min_element(distances.begin(), distances.end()) - distances.begin();
  inters = intersections[minElementIndex];
  // std::cout << "Plane coeff" << coeff.transpose() << std::endl;
  // std::cout << "Intersection=" << inters.transpose() << " is the correct one!" << std::endl;
  // std::cout << "Reached this point" << std::endl;

  if (par_.visual == true)
  {
    I_.header.stamp = ros::Time::now();
    I_.pose.position.x = inters(0);
    I_.pose.position.y = inters(1);
    I_.pose.position.z = inters(2);
    pub_intersectionI_.publish(I_);
  }

  // std::cout << "Going to return" << inters.transpose() << std::endl;
  return inters;
}

vec_Vecf<3> CVX::fix(vec_Vecf<3>& JPS_old, Eigen::Vector3d& start, Eigen::Vector3d& goal, bool* solved)
{
  vec_Vecf<3> fix;
  vec_Vecf<3> JPS_old_original = JPS_old;
  bool thereIsIntersection = false;

  vec_Vecf<3> path_start2fix;  // referenceFs has to be initialized
  vec_Vecf<3> path_fix2goal;
  // std::cout << "*********In fix0.6" << std::endl;
  path_start2fix.clear();
  path_fix2goal.clear();
  vec_Vecf<3> path_fixed;

  // std::cout << "*********In fix0.7" << std::endl;
  Eigen::Vector3d inters1 = getFirstCollisionJPS(JPS_old, &thereIsIntersection, MAP,
                                                 RETURN_INTERSECTION);  // intersection starting from start
  // std::cout << "Here thereIsIntersection=" << thereIsIntersection << std::endl;
  // std::cout << "*********In fix2" << std::endl;

  if (thereIsIntersection)
  {
    // std::cout << "*********In fix2.5" << std::endl;
    clearJPSPathVisualization(2);
    vec_Vecf<3> tmp = JPS_old;
    // std::cout << "*****tmp is:" << std::endl;
    // printElementsOfJPS(tmp);
    std::reverse(tmp.begin(), tmp.end());  // flip all the vector
    Eigen::Vector3d inters2 = getFirstCollisionJPS(tmp, &thereIsIntersection, MAP,
                                                   RETURN_INTERSECTION);  // intersection starting from the goal

    // std::reverse(path_fix2goal.begin(), path_fix2goal.end());
    bool solvedFix, solvedStart2Fix, solvedFix2Goal;
    // std::cout << "*********In fix3" << std::endl;

    if ((inters1 - inters2).lpNorm<1>() > 0.01)  // Hack to delete corner cases TODO
    // if (inters1.isApprox(inters2, 0.01))
    {
      fix = jps_manager_.solveJPS3D(inters1, inters2, &solvedFix, 2);
    }
    else
    {
      fix.push_back(inters1);
    }

    if ((start - inters1).lpNorm<1>() > 0.01)  // Hack to delete corner cases TODO
    // if (start.isApprox(inters1, 0.01))
    {
      path_start2fix = jps_manager_.solveJPS3D(start, inters1, &solvedStart2Fix, 2);
    }
    else
    {
      path_start2fix.push_back(start);
    }

    if ((inters2 - goal).lpNorm<1>() > 0.01)  // Hack to delete corner cases TODO
    // if (inters2.isApprox(goal, 0.01))
    {
      path_fix2goal = jps_manager_.solveJPS3D(inters2, goal, &solvedFix2Goal, 2);
    }
    else
    {
      path_fix2goal.push_back(goal);
    }

    // printf("AQUI4\n");

    // printf("After calling solveJPSD\n");
    bool solved_complete_fix = solvedFix && solvedStart2Fix && solvedFix2Goal;
    if (solved_complete_fix == false)
    {
      printf("**************Couldn't find some part of the fixed path**********\n");
      *solved = false;
    }

    else
    {
      *solved = true;

      path_fixed.clear();
      path_fixed.insert(path_fixed.end(), path_start2fix.begin(), path_start2fix.end());
      path_fixed.insert(path_fixed.end(), fix.begin() + 1, fix.end());
      path_fixed.insert(path_fixed.end(), path_fix2goal.begin() + 1, path_fix2goal.end());
      /*      printf("***************Start to fix******************\n");
            printElementsOfJPS(path_start2fix);
            printf("***************Fix***************************\n");
            printElementsOfJPS(fix);
            printf("***************Fix to Goal***************************\n");
            printElementsOfJPS(path_fix2goal);
            printf("***************Everything***************************\n");
            printElementsOfJPS(path_fixed);*/
      if (par_.visual == true)
      {
        publishJPS2handIntersection(path_fixed, inters1, inters2, solved_complete_fix);
      }
    }
  }

  else
  {
    printf("there is no intersection\n");
    /*   std::cout << "the start is " << start.transpose() << std::endl;
      std::cout << "the goal is " << goal.transpose() << std::endl;*/

    *solved = true;
    // JPS_old[0] = start;
    // JPS_old[JPS_old.size() - 1] = goal;

    /*    std::cout << "voy a copiar" << std::endl;
        printElementsOfJPS(JPS_old_original);*/

    std::copy(JPS_old_original.begin(), JPS_old_original.end(), back_inserter(path_fixed));  // Copy JPS_old into fix
    // fix = JPS_old;
    path_fixed[0] = start;
    path_fixed[path_fixed.size() - 1] = goal;

    /*    std::cout << "Lo copiado" << std::endl;
        printElementsOfJPS(path_fixed);*/

    if (par_.visual == true)
    {
      publishJPS2handIntersection(path_fixed, path_fixed[0], path_fixed[path_fixed.size() - 1], 1);
    }
  }
  // printf("finisshing fix\n");

  return path_fixed;
}

void CVX::publishJPS2handIntersection(vec_Vecf<3> JPS2_fix, Eigen::Vector3d& inter1, Eigen::Vector3d& inter2,
                                      bool solvedFix)
{
  // printf("Going to publish\n");
  /*vec_Vecf<3> traj, visualization_msgs::MarkerArray* m_array*/
  clearJPSPathVisualization(2);
  // path_jps_ = clearArrows();

  // vectorOfVectors2MarkerArray(JPS2, &path_jps2_, color(RED));
  if (solvedFix == true)
  {
    vectorOfVectors2MarkerArray(JPS2_fix, &path_jps2_, color(GREEN));
  }

  visualization_msgs::Marker m1;
  m1.header.frame_id = "world";
  m1.id = 19865165;
  m1.type = visualization_msgs::Marker::SPHERE;
  m1.scale = vectorUniform(0.3);
  m1.color = color(BLUE_TRANS);
  m1.pose.position = eigen2point(inter1);
  path_jps2_.markers.push_back(m1);

  visualization_msgs::Marker m2;
  m2.header.frame_id = "world";
  m2.id = 19865166;
  m2.type = visualization_msgs::Marker::SPHERE;
  m2.scale = vectorUniform(0.3);
  m2.color = color(RED_TRANS);
  m2.pose.position = eigen2point(inter2);
  path_jps2_.markers.push_back(m2);

  pub_path_jps2_.publish(path_jps2_);
}

if (l_constraints_whole_[0].inside(A) == false)
{
  std::cout << red << "First point of whole traj is outside" << reset << std::endl;
}

if (planner_status_ == START_REPLANNING && status_started == REPLANNED)
{
  /*      std::cout << bold << "Solved everything but the publisher already copied current matrix, exiting" << reset
                  << std::endl;*/
  return;
}