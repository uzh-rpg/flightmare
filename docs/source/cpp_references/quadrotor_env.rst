.. _cpp-quad-env-ref:

Quadrotor environment reference
===============================

.. class:: QuadrotorEnv

  .. function:: QuadrotorEnv()

    Constructs a QuadrotorEnv from the configuration file "/flightlib/configs/quadrotor_env.yaml".

    :rtype: QuadrotorEnv


  .. function:: QuadrotorEnv(const std::string &cfg_path)
      
      Constructs a QuadrotorEnv from the given configuration file.

      :param cfg_path: configuration file path
      :type cfg_path: std::string
      :rtype: QuadrotorEnv

  .. function:: ~QuadrotorEnv()

    Deconstructs this QuadrotorEnv.

    :rtype: None


  .. function:: reset(Ref<Vector<>> obs, const bool random)

   Reset the quadrotor state and get observations.

   | //full states of the quadrotor
   | obs = [position, orientation, linear velocity, angular velocity] 

   :param obs: Observations
   :type obs: Ref<Vector<>>
   :param random: randomly reset the quadrotor state
   :type random: const bool
   :rtype: bool

  .. function:: getObs(Ref<Vector<>> obs)

    Get observations.

   | //full states of the quadrotor
   | obs = [position, orientation, linear velocity, angular velocity]

   :param obs: Observations
   :type obs: Ref<Vector<>>
   :rtype: bool

  .. function:: step(const Ref<Vector<>> act, Ref<Vector<>> obs)

   Simulate one step with the given thrusts.

   Get observations and the total reward of this step.

   | // thrusts of each propeller
   | act = [f1, f2, f3, f4]

   | //full states of the quadrotor
   | obs = [position, orientation, linear velocity, angular velocity] 

   :param act: thrusts of each propeller
   :type act: const Ref<Vector<>>
   :param obs: Observations
   :type obs: Ref<Vector<>>
   :rtype: Total reward (Scalar)

  .. function:: isTerminalState(Scalar &reward)

   Check if terminal state is reached.

   :param reward: Reward of state
   :type reward: Scalar
   :rtype: bool

  .. function:: loadParam(const YAML::Node &cfg)

   Load the QuadrotorEnv parameters.

   :param cfg: configuration YAML
   :type cfg: const YAML::Node
   :rtype: bool

  .. function:: getAct(Ref<Vector<>> act)

   Get thrusts for commands.

   | // thrusts of each propeller
   | act = [f1, f2, f3, f4]

   :param act: thrusts of each propeller
   :type act: const Ref<Vector<>>
   :rtype: bool

  .. function:: getAct(Command *const cmd)

   Get the next command for the quadrotor.

   :param cmd: Quadrotor command
   :type cmd: Command
   :rtype: bool

  .. function:: addObjectsToUnity(std::shared_ptr<UnityBridge> bridge)

   Add the quadrotor to Flightmare.

   :param bridge: Unity bridge
   :type bridge: std::shared_ptr<UnityBridge>
   :rtype: None
