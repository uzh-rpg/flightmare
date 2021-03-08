.. _faq-binary:

Frequently Asked Questions
==========================

In case your problem is not solved yet, please state it in 
`issues <https://github.com/uzh-rpg/flightmare/issues>`_.

How many scenes are there?
--------------------------

The binaries is released with three environments.

* `Industrial <https://assetstore.unity.com/packages/3d/environments/industrial/rpg-fps-game-assets-for-pc-mobile-industrial-set-v2-0-86679>`_

* `Warehouse <https://assetstore.unity.com/packages/3d/environments/industrial/hangar-building-modular-142104>`_

* `Garage <https://assetstore.unity.com/packages/3d/environments/urban/garage-pack-vol-02-148265>`_

* `NaturalForest <https://assetstore.unity.com/packages/3d/vegetation/forest-environment-dynamic-nature-150668>`_


The git repository version only includes the environment **Industrial**, which was created by Dmitrii Kutsenko, and is freely available in the asset store. 

Can custom scenes be added?
---------------------------

Yes, follow the steps :ref:`here <add-scene>`.

Some cool free environments:

* `City <https://assetstore.unity.com/packages/3d/environments/roadways/windridge-city-132222>`_

* `Flooded Grounds <https://assetstore.unity.com/packages/3d/environments/flooded-grounds-48529>`_

* `Dream Forest <https://assetstore.unity.com/packages/3d/vegetation/trees/dream-forest-tree-105297>`_


Why are some objects flickering?
--------------------------------

Unity optimizes the performance by not render objects that are out of sight. 
This can lead to flickering when the camera moves quickly. 
This can be prevented when the option **occlusion culling** is disabled for the camera.


The terrain does not show in the depth image?
---------------------------------------------

Unity optimizes the performance by drawing the terrain instanced. 
When the option **Terrain.drawInstanced is disabled** then the terrain is visible on the depth camera.


Why do the terrain trees only show as capsules in the pointcloud?
-----------------------------------------------------------------

Unity allows (probably for performance reasons) only capsules colliders on `terrain trees <https://docs.unity3d.com/Manual/terrain-Trees.html>`_.
We wrote a script that replaces all terrain trees with a prefab of a tree with Mesh collider on start of the scene.
The `script <https://github.com/uzh-rpg/rpg_flightmare_unity/blob/dev/flightmare-release/Assets/Flightmare/Flightmare/Scripts/HelperScripts/terrainTreeManager.cs>`_ needs to be added to the specific scene.

Is it possible to run the standalone built on a server?
-------------------------------------------------------

The latest release built is for local desktop environments. 
Technically it's possible to run it on a server, but currently there are problems with some shaders.

The depth image is not linear?
------------------------------

Be careful that the color space is set to **linear**. 
The shaders, which compute the depth image, depend on the color space. In case you change the color space in your project, you will need to adjust the UberReplacementShader. 
`Here <https://forum.unity.com/threads/gamma-space-and-linear-space-with-shader.243793/>`_ why we chose the factor 2.2 for our shader.

