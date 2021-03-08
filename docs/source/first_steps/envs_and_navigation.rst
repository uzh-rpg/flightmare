.. _environments-navigation:

Environments and navigation
===========================

Flightmare environments
-----------------------

The following environments are available in `UnityScene <https://github.com/uzh-rpg/flightmare/blob/master/flightlib/include/flightlib/bridges/unity_message_types.hpp>`_.
The environment can easily be used with

.. code-block:: C++

  // UnityScene::<SCENE_NAME>
  SceneID scene_id_{UnityScene::INDUSTRIAL};

instead of using the specific ID.

.. list-table:: 
  :widths: 10 30 60
  :header-rows: 1

  * - ID
    - Environment
    - Summary
  * - 0
    - INDUSTRIAL
    - A basic outdoor industrial environment
  * - 1
    - WAREHOUSE
    - A small indoor warehouse environment
  * - 2
    - GARAGE
    - A small indoor garage environment
  * - 3
    - NATUREFOREST
    - A high-quality outdoor forest environment

.. raw:: html

  <br>
  
  <!DOCTYPE html>
  <html>
  <head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
  * {box-sizing: border-box}
  body {font-family: Verdana, sans-serif; margin:0}
  .mySlides {display: none}
  img {vertical-align: middle;}

  /* Slideshow container */
  .slideshow-container {
    max-width: 1000px;
    position: relative;
    margin: auto;
  }

  /* Next & previous buttons */
  .prev, .next {
    cursor: pointer;
    position: absolute;
    top: 50%;
    width: auto;
    padding: 16px;
    margin-top: -22px;
    color: white;
    font-weight: bold;
    font-size: 18px;
    transition: 0.6s ease;
    border-radius: 0 3px 3px 0;
    user-select: none;
  }

  /* Position the "next button" to the right */
  .next {
    right: 0;
    border-radius: 3px 0 0 3px;
  }

  /* On hover, add a black background color with a little bit see-through */
  .prev:hover, .next:hover {
    background-color: rgba(0,0,0,0.8);
  }

  /* Caption text */
  .text {
    color: #f2f2f2;
    font-size: 15px;
    padding: 8px 12px;
    position: absolute;
    bottom: 8px;
    width: 100%;
    text-align: center;
  }

  /* Number text (1/3 etc) */
  .numbertext {
    color: #f2f2f2;
    font-size: 12px;
    padding: 8px 12px;
    position: absolute;
    top: 0;
  }

  /* The dots/bullets/indicators */
  .dot {
    cursor: pointer;
    height: 15px;
    width: 15px;
    margin: 0 2px;
    background-color: #bbb;
    border-radius: 50%;
    display: inline-block;
    transition: background-color 0.6s ease;
  }

  .active, .dot:hover {
    background-color: #717171;
  }

  /* Fading animation */
  .fade {
    -webkit-animation-name: fade;
    -webkit-animation-duration: 1.5s;
    animation-name: fade;
    animation-duration: 1.5s;
  }

  @-webkit-keyframes fade {
    from {opacity: .4} 
    to {opacity: 1}
  }

  @keyframes fade {
    from {opacity: .4} 
    to {opacity: 1}
  }

  /* On smaller screens, decrease text size */
  @media only screen and (max-width: 300px) {
    .prev, .next,.text {font-size: 11px}
  }
  </style>
  </head>
  <body>

  <div class="slideshow-container">

  <div class="mySlides fade">
    <div class="numbertext">1 / 4</div>
    <img src="https://assetstorev1-prd-cdn.unity3d.com/package-screenshot/563e298d-a799-46ba-a488-543be7cde268.webp" style="width:100%">
    <div class="text">INDUSTRIAL</div>
  </div>

  <div class="mySlides fade">
    <div class="numbertext">2 / 4</div>
    <img src="https://assetstorev1-prd-cdn.unity3d.com/package-screenshot/f3f4190f-ebeb-4d4c-88b1-21ad75199367.webp" style="width:100%">
    <div class="text">WAREHOUSE</div>
  </div>

  <div class="mySlides fade">
    <div class="numbertext">3 / 4</div>
    <img src="https://assetstorev1-prd-cdn.unity3d.com/package-screenshot/11fcda88-dbaf-40e2-8c3c-9570d88bb6da.webp" style="width:100%">
    <div class="text">GARAGE</div>
  </div>

  <div class="mySlides fade">
  <div class="numbertext">4 / 4</div>
  <img src="https://assetstorev1-prd-cdn.unity3d.com/package-screenshot/b81b0e31-7de2-40db-a806-796f37e6fb16.webp" style="width:100%">
  <div class="text">NATUREFOREST</div>
  </div>

  <a class="prev" onclick="plusSlides(-1)">&#10094;</a>
  <a class="next" onclick="plusSlides(1)">&#10095;</a>

  </div>
  <br>

  <div style="text-align:center">
    <span class="dot" onclick="currentSlide(1)"></span> 
    <span class="dot" onclick="currentSlide(2)"></span> 
    <span class="dot" onclick="currentSlide(3)"></span> 
    <span class="dot" onclick="currentSlide(4)"></span> 
  </div>

  <script>
  var slideIndex = 1;
  showSlides(slideIndex);

  function plusSlides(n) {
    showSlides(slideIndex += n);
  }

  function currentSlide(n) {
    showSlides(slideIndex = n);
  }

  function showSlides(n) {
    var i;
    var slides = document.getElementsByClassName("mySlides");
    var dots = document.getElementsByClassName("dot");
    if (n > slides.length) {slideIndex = 1}    
    if (n < 1) {slideIndex = slides.length}
    for (i = 0; i < slides.length; i++) {
        slides[i].style.display = "none";  
    }
    for (i = 0; i < dots.length; i++) {
        dots[i].className = dots[i].className.replace(" active", "");
    }
    slides[slideIndex-1].style.display = "block";  
    dots[slideIndex-1].className += " active";
  }
  </script>

  </body>
  </html> 

  <br>




Navigating through waypoints
----------------------------

.. include:: ../tutorials_general/racing.rst
  :start-after: ============================
  :end-before: Here the full code example


Here the full code example
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. raw:: html

   <details>
   <summary>racing.hpp</summary>

.. include:: ../tutorials_general/racing.hpp
  :code: C++

.. raw:: html

   </details>

.. raw:: html

   <details>
   <summary>racing.cpp</summary>

.. include:: ../tutorials_general/racing.cpp
  :code: C++

.. raw:: html

   </details>
   <br>

