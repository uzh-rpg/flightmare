.. _py-flight-env-vec-ref:

FlightEnvVec
============

The wrapper of Flightmare.

.. class:: FlightEnvVec(impl)

   An OpenAIGym like environment of Flightmare.

   
   env = FlightEnvVec.FlightEnvVec(QuadrotorEnv_v1(
        dump(cfg, Dumper=RoundTripDumper), False))
  
  :param impl: Implementation of flightgym
  :type impl: flightgym.QuadrotorEnv_v1

  .. function:: seed(self, seed=0)

    Set a random seed for the environment.

    :param seed: Random seed
    :type seed: int
    :rtype: bool

  .. function:: step(self, action)

   Simulate one step with the given thrusts.

   Get observations and the total reward of this step.

   | // thrusts of each propeller
   | action = [f1, f2, f3, f4]

   | //full states of the quadrotor
   | obs = [position, orientation, linear velocity, angular velocity] 

   :param action: thrusts of each propeller
   :type action: np.ndarray
   :rtype: observations, reward, done, info

  
  .. error:: stepUnity() not yet implemented.

  .. function:: stepUnity(self, action, send_id)

   Simulate one step with the given thrusts and render it in Unity.

   Get observations and the total reward of this step.

   | // thrusts of each propeller
   | action = [f1, f2, f3, f4]

   | //full states of the quadrotor
   | obs = [position, orientation, linear velocity, angular velocity] 

   :param action: thrusts of each propeller
   :type action: np.ndarray
   :param send_id: frame ID
   :type send_id: int
   :rtype: observations, reward, done, info

  
  .. function:: sample_actions(self)

   Sample an action.

   | // thrusts of each propeller
   | action = [f1, f2, f3, f4]

   :rtype: action (np.ndarray)


 .. function:: reset()

   Reset the quadrotor state and get observations.

   | //full states of the quadrotor
   | obs = [position, orientation, linear velocity, angular velocity] 

   :rtype: obs

 .. function:: reset_and_update_info(self)

   Reset the quadrotor state, get observations and update info.

   | //full states of the quadrotor
   | obs = [position, orientation, linear velocity, angular velocity] 

   :rtype: obs, info

 .. error:: render() not yet implemented.

 .. function:: render(self, mode='human')

   Render in Flightmare.

   :rtype: None

  .. function:: close(self)

    Close the environment.

   :rtype: None

  .. function:: connectUnity(self)

   Connect to Flightmare.

   :rtype: None

  .. function:: disconnectUnity(self)

    Disconnect from Flightmare.

   :rtype: None