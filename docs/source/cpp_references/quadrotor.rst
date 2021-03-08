.. _cpp-quad-ref:

Quadrotor References
====================

.. class:: Quadrotor

  .. function:: Quadrotor(const std::string& cfg_path)

   Construct quadrotor.

   :param cfg_path: path to configuration yaml file
   :type cfg_path: const std::string&
   :rtype: Quadrotor

  .. function:: Quadrotor(const QuadrotorDynamics& dynamics = QuadrotorDynamics(1.0, 0.25))

   Construct quadrotor.

   :param dynamics: quadrotor dynamics parameters
   :type dynamics: const QuadrotorDynamics&  Default=(1.0, 0.25)
   :rtype: Quadrotor

  .. function:: ~Quadrotor();

   Deconstruct quadrotor.

   :rtype: None


  .. function:: reset();

   Reset quadrotor's quadstate.

   :rtype: bool
  
  .. function:: reset(const QuadState& state)

    Reset quadrotor's quadstate to given quadstate.

    :param state: quadrotor new QuadState
    :type state: const QuadrotorDynamics&
    :rtype: bool

  .. function:: init();

   Initialize quadrotor's quadstate to default.

   :rtype: None

  .. function:: run(const Scalar dt)

    Simulate dynamics for dt.

    :param dt: delta time
    :type dt: const Scalar
    :rtype: bool

  .. function:: run(const Command& cmd, const Scalar dt)
    :noindex:

    Simulate dynamics with command for dt.

    :param cmd: quadrotor command
    :type cmd: const Command&
    :param dt: delta time
    :type dt: const Scalar
    :rtype: bool

  .. function:: getState(QuadState* const state)

    Get the current state of the quadrotor.

    :param state: quadrotor state
    :type state: QuadState* const
    :rtype: bool

  .. function:: getMotorThrusts(Ref<Vector<4>> motor_thrusts) 

    Get the current motor thrust of the quadrotor.

    :param motor_thrusts: Motor thrusts
    :type motor_thrusts: Ref<Vector<4>> 
    :rtype: bool

  .. function:: getMotorOmega(Ref<Vector<4>> motor_omega)

    Get the current motor omega of the quadrotor.

    :param motor_omega: Motor omega
    :type motor_omega: Ref<Vector<4>> 
    :rtype: bool

  .. function:: getDynamics(QuadrotorDynamics* const dynamics)

    Get the quadrotor dynamics.

    :param dynamics: quadrotor dynamics
    :type dynamics: QuadrotorDynamics* const 
    :rtype: bool

  .. function:: getDynamics()
    :noindex:

    Get the quadrotor dynamics.

    :rtype: const QuadrotorDynamics&

  .. function:: getSize()

    Get the quadrotor size.

    :rtype: Vector<3>

  .. function:: getPosition()

    Get the quadrotor position.

    :rtype: Vector<3>

  .. function:: getQuaternion()

    Get the quadrotor orientation.

    :rtype: Quaternion

  .. function:: getCameras()

    Get all cameras assigned to the quadrotor.

    :rtype: std::vector<std::shared_ptr<RGBCamera>>

  .. function:: getCamera(const size_t cam_id, std::shared_ptr<RGBCamera> camera)

    Get the camera with the given id which is assigned to the quadrotor.

    :param cam_id: camera ID
    :type cam_id: size_t
    :param camera: pointer on camera object
    :type camera: std::shared_ptr<RGBCamera>
    :rtype: bool

  .. function:: setState(const QuadState& state)

    Set quadrotor state.

    :param state: quadrotor state
    :type state: const QuadState&
    :rtype: bool

  .. function:: setCommand(const Command& cmd)

    Set quadrotor command.

    :param cmd: quadrotor command
    :type cmd: const Command&
    :rtype: bool

  .. function:: updateDynamics(const QuadrotorDynamics& dynamics)

    Update quadrotor dynamics.

    :param dynamics: quadrotor dynamics
    :type dynamics: const QuadrotorDynamics&
    :rtype: bool

   .. function:: addRGBCamera(std::shared_ptr<RGBCamera> camera)

    Add RGBCamera to quadrotor.

    :param camera: pointer on camera
    :type camera: std::shared_ptr<RGBCamera>
    :rtype: bool   

  .. function:: runFlightCtl(const Scalar sim_dt, const Vector<3>& omega, const Command& cmd)

    Run low-level controller.

    :param sim_dt: simulation delta time
    :type sim_dt: const Scalar
    :param omega: omega
    :type omega: const Vector<3>&
    :param cmd: quadrotor command
    :type cmd: const Command&
    :rtype: Vector<4>

  .. function:: runMotors(const Scalar sim_dt, const Vector<4>& motor_thrust_des)

    Simulate motor.

    :param sim_dt: simulation delta time
    :type sim_dt: const Scalar
    :param motor_thrust_des: desired motor thrust
    :type motor_thrust_des: const Vector<4>&
    :rtype: None

    .. function:: setWorldBox(const Ref<Matrix<3, 2>> box)

    Set constraints for world.

    :param box: boundary box
    :type box: const Ref<Matrix<3, 2>>
    :rtype: bool

    .. function:: constrainInWorldBox(const QuadState& old_state)

    Check if quadstate is within the constraints of the world.

    :param box: old_state
    :type box: const QuadState& 
    :rtype: bool

    .. function:: getMass()

      Get quadrotor mass.

      :rtype: Scalar

    .. function:: setSize(const Ref<Vector<3>> size)

      Set quadrotor size.

      :param size: quadrotor size
      :type size: const Ref<Vector<3>>
      :rtype: None

    .. function:: setCollision(const bool collision)

      Set information about quadrotor collision.

      :param collision: if collided
      :type collision: const bool 
      :rtype: None

    
    .. note:: Need to implement getCollision

    .. function:: getCollision()

      Get information about quadrotor collision.

      :rtype: bool
