.. _faq-install:

Frequently Asked Questions
==========================

Cannot launch flightros examples?
---------------------------------

Probably the examples have not been compiled yet. Run

.. code-block:: bash

  catkin build flightros -DBUILD_SAMPLES:=ON

to properly install the examples.

OpenCV 4.2 compilation error?
-----------------------------

This issue has been resolved by a user. See the `solution in the issue <https://github.com/uzh-rpg/flightmare/issues/23>`_.
