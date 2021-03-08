.. _adv-motion-planning:

Motion planning
===============

In this section, we explain how to use `The Open Motion Planning Library (OMPL) <https://ompl.kavrakilab.org/>`_ with Flightmare for advanced motion planning.

.. note::

  We followed the example of OMPL "Geometric Planning for a Rigid Body in 3D".
  `Here <https://ompl.kavrakilab.org/geometricPlanningSE3.html>`_ the link to the tutorial.
  We highly recommend first getting familiar with OMPL before trying to integrate it into your Flightmare project.


OMPL Simple Setup
-----------------

Setting up geometric planning for a rigid body in 3D requires the following steps:

* identify the space we are planning in: SE(3)

* select a corresponding state space from the available ones, or implement one. For SE(3), the ompl::base::SE3StateSpace is appropriate.

* since SE(3) contains an R^3 component, we need to define bounds.

* define the notion of state validity.

* define start states and a goal representation.


Once these steps are complete, the specification of the problem is conceptually done. The set of classes that allow the instantiation of this specification is shown below.

.. raw:: html

   <details>
   <summary>Using the ompl::geometric::SimpleSetup Class</summary>

Assuming the following namespace definitions:

.. code-block:: C++

  namespace ob = ompl::base;
  namespace og = ompl::geometric;

And a state validity checking function defined like this:

.. code-block:: C++

  bool isStateValid(const ob::State *state)

We first create an instance of the state space we are planning in.

.. code-block:: C++

  void planWithSimpleSetup()
  {
      // construct the state space we are planning in
      auto space(std::make_shared<ob::SE3StateSpace>());

We then set the bounds for the R3 component of this state space:

.. code-block:: C++

  ob::RealVectorBounds bounds(3);
  bounds.setLow(-1);
  bounds.setHigh(1);

  space->setBounds(bounds);

Create an instance of **ompl::geometric::SimpleSetup**. Instances of **ompl::base::SpaceInformation**, and **ompl::base::ProblemDefinition** are created internally.

.. code-block:: C++

  og::SimpleSetup ss(space);

Set the state validity checker

.. code-block:: C++

  ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });

Create a random start state:

.. code-block:: C++

  ob::ScopedState<> start(space);
  start.random();

And a random goal state:

.. code-block:: C++

  ob::ScopedState<> goal(space);
  goal.random();

Set these states as start and goal for SimpleSetup.

.. code-block:: C++

  ss.setStartAndGoalStates(start, goal);

We can now try to solve the problem. This will also trigger a call to **ompl::geometric::SimpleSetup::setup()** and create a default instance of a planner, since we have not specified one. Furthermore, **ompl::base::Planner::setup()** is called, which in turn calls **ompl::base::SpaceInformation::setup()**. This chain of calls will lead to computation of runtime parameters such as the state validity checking resolution. This call returns a value from **ompl::base::PlannerStatus** which describes whether a solution has been found within the specified amount of time (in seconds). If this value can be cast to true, a solution was found.

.. code-block:: C++

  ob::PlannerStatus solved = ss.solve(1.0);

If a solution has been found, we can optionally simplify it and the display it

.. code-block:: C++

  if (solved)
  {
      std::cout << "Found solution:" << std::endl;
      // print the path to screen
      ss.simplifySolution();
      ss.getSolutionPath().print(std::cout);
  }

.. raw:: html

   </details>
   <br>


State validity checking function
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A state validity function needs to be implemented. 
More details about how it needs to be defined can be found in the `OMPL documentation <https://ompl.kavrakilab.org/stateValidation.html>`_.
For our example, we implemented a KD-search tree and check if the point is within the range of a boundary point.
If not, the point is valid and otherwise invalid.

.. code-block:: C++

  bool motion_planning::isStateValid(const ob::State *state) {
  // cast the abstract state type to the type we expect
  const auto *se3state = state->as<ob::SE3StateSpace::StateType>();

  // extract the first component of the state and cast it to what we expect
  const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

  // extract the second component of the state and cast it to what we expect
  //  const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

  // check validity of state defined by pos & rot
  float x = pos->values[0];
  float y = pos->values[1];
  float z = pos->values[2];
  // return a value that is always true but uses the two variables we define, so
  // we avoid compiler warnings
  // return isInRange(x, y, z);
  Eigen::Vector3d query_pos{x, y, z};
  return searchRadius(query_pos, range);
  }

Here the helper function searchRadius.

.. code-block:: C++

  bool motion_planning::searchRadius(const Eigen::Vector3d &query_point,
                                   const double radius) {
  std::vector<int> indices;
  std::vector<double> distances_squared;
  kd_tree_.SearchRadius(query_point, radius, indices, distances_squared);

  if (indices.size() == 0) {
    return true;
  }

  for (const auto &close_point_idx : indices) {
    // get point, check if within drone body
    Eigen::Vector3d close_point = points_.col(close_point_idx);
    // project point on each body axis and check distance
    Eigen::Vector3d close_point_body = (close_point - query_point);
    if (std::abs(close_point_body.x()) <= range &&
        std::abs(close_point_body.y()) <= range &&
        std::abs(close_point_body.z()) <= range) {
      // point is in collision
      return false;
    }
  }

  return true;
  }

State space bounds
^^^^^^^^^^^^^^^^^^

.. code-block:: C++

  // set the bounds for the R^3 part of SE(3)
  ob::RealVectorBounds bounds(3);

  bounds.setLow(0, min_bounds.x);
  bounds.setLow(1, min_bounds.y);
  bounds.setLow(2, min_bounds.z);
  bounds.setHigh(0, max_bounds.x);
  bounds.setHigh(1, max_bounds.y);
  bounds.setHigh(2, max_bounds.z);

  space->setBounds(bounds);

Follow the OMPL documentation example so that the solution can be computed.


Generate Point Cloud and visualize solution path
------------------------------------------------

Create a Point Cloud
^^^^^^^^^^^^^^^^^^^^

Either use the GUI or the client to generate a point cloud of your environment and save it as a .ply file.
Check the page :ref:`PointCloud <point-cloud>` for more details. 

For the following steps, we will assume that a point cloud was successfully saved.


Read the Point Cloud
^^^^^^^^^^^^^^^^^^^^

We load the point cloud with help from `tinyply.h <https://github.com/ddiakopoulos/tinyply>`_.
We populate the KD-Search Tree for more a more efficient State Validity checker function.

.. code-block:: C++

  std::vector<motion_planning::float3> motion_planning::readPointCloud() {
  std::unique_ptr<std::istream> file_stream;
  std::vector<uint8_t> byte_buffer;
  std::string filepath =
    std::experimental::filesystem::path(__FILE__).parent_path().string() +
    "/data/point_cloud.ply";
  try {
    file_stream.reset(new std::ifstream(filepath, std::ios::binary));

    if (!file_stream || file_stream->fail())
      throw std::runtime_error("file_stream failed to open " + filepath);

    file_stream->seekg(0, std::ios::end);
    const float size_mb = file_stream->tellg() * float(1e-6);
    file_stream->seekg(0, std::ios::beg);

    PlyFile file;
    file.parse_header(*file_stream);

    std::cout << "\t[ply_header] Type: "
              << (file.is_binary_file() ? "binary" : "ascii") << std::endl;
    for (const auto &c : file.get_comments())
      std::cout << "\t[ply_header] Comment: " << c << std::endl;
    for (const auto &c : file.get_info())
      std::cout << "\t[ply_header] Info: " << c << std::endl;

    for (const auto &e : file.get_elements()) {
      std::cout << "\t[ply_header] element: " << e.name << " (" << e.size << ")"
                << std::endl;
      for (const auto &p : e.properties) {
        std::cout << "\t[ply_header] \tproperty: " << p.name
                  << " (type=" << tinyply::PropertyTable[p.propertyType].str
                  << ")";
        if (p.isList)
          std::cout << " (list_type=" << tinyply::PropertyTable[p.listType].str
                    << ")";
        std::cout << std::endl;
      }
    }

    // Because most people have their own mesh types, tinyply treats parsed data
    // as structured/typed byte buffers. See examples below on how to marry your
    // own application-specific data structures with this one.
    std::shared_ptr<PlyData> vertices, normals, colors, texcoords, faces,
      tripstrip;

    // The header information can be used to programmatically extract properties
    // on elements known to exist in the header prior to reading the data. For
    // brevity of this sample, properties like vertex position are hard-coded:
    try {
      vertices =
        file.request_properties_from_element("vertex", {"x", "y", "z"});
    } catch (const std::exception &e) {
      std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    manual_timer read_timer;

    read_timer.start();
    file.read(*file_stream);
    read_timer.stop();

    const float parsing_time = read_timer.get() / 1000.f;
    std::cout << "\tparsing " << size_mb << "mb in " << parsing_time
              << " seconds [" << (size_mb / parsing_time) << " MBps]"
              << std::endl;

    if (vertices)
      std::cout << "\tRead " << vertices->count << " total vertices "
                << std::endl;


    const size_t numVerticesBytes = vertices->buffer.size_bytes();
    std::vector<float3> verts(vertices->count);
    std::memcpy(verts.data(), vertices->buffer.get(), numVerticesBytes);

    int idx = 0;
    for (const auto &point_tinyply : verts) {
      if (idx == 0) {
        points_ = Eigen::Vector3d(static_cast<double>(point_tinyply.x),
                                  static_cast<double>(point_tinyply.y),
                                  static_cast<double>(point_tinyply.z));
      } else {
        points_.conservativeResize(points_.rows(), points_.cols() + 1);
        points_.col(points_.cols() - 1) =
          Eigen::Vector3d(static_cast<double>(point_tinyply.x),
                          static_cast<double>(point_tinyply.y),
                          static_cast<double>(point_tinyply.z));
      }
      idx += 1;
    }
    kd_tree_.SetMatrixData(points_);

    return verts;
  } catch (const std::exception &e) {
    std::cerr << "Caught tinyply exception: " << e.what() << std::endl;
  }
  return verts;
  }

Execute path
^^^^^^^^^^^^

In the plan function, we solve the path planning problem and can then get the found states of the solution path.
In our example, we just linearly interpolated between the states and followed those points, but you can also use the `rpg_quadrotor_control <https://github.com/uzh-rpg/rpg_quadrotor_control>`_ library.


.. code-block:: C++

  path_ = ss.getSolutionPath().getStates();

  for (auto const &pos : path_) {
    vecs_.push_back(stateToEigen(pos));
  }



Run example
-----------

.. code-block:: bash

  roslaunch flightros motion_planning.launch


.. highlight:: C++ 

Here the full code example
--------------------------

.. raw:: html

   <details>
   <summary>Additional script tinyply.h</summary>

.. include:: tinyply.h
  :code: C++

.. raw:: html

   </details>
   <br>

Header
^^^^^^

.. include:: motion_planning.hpp
  :code: C++


Main
^^^^

.. include:: motion_planning.cpp
  :code: C++