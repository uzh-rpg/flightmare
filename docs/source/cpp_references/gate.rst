.. _cpp-gate-ref:

StaticGate References
=====================

.. class:: StaticGate

  .. function:: StaticGate(std::string id, std::string prefab_id)

   Constuct Static gate.

   :param id: unique name
   :type id: std::string
   :param prefab_id: prefab name in unity
   :type prefab_id: std::string
   :rtype: StaticGate

  .. function:: ~StaticGate()

   Deconstuct Static gate.

   :rtype: None

  .. function:: setPosition(const Vector<3>& position)

   Set the position of the gate.

   :param position: position of the gate
   :type position: const Vector<3>& 
   :rtype: None

  .. function:: setRotation(const Quaternion& quaternion)

   Set the orientation of the gate.

   :param quaternion: orientation of the gate
   :type quaternion: const Quaternion& 
   :rtype: None

  .. function:: setSize(const Vector<3>& size)

   Set the size of the gate.

   :param size: size of the gate
   :type size: const Vector<3>&
   :rtype: None

  .. function:: getPos()

   Get the position of the gate.

   :rtype: Vector<3> 

  .. function:: getQuat()

   Get the orientation of the gate.

   :rtype: Quaternion

  .. function:: getSize()

   Get the size of the gate.

   :rtype: Vector<3>
