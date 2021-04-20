# Assignment 4: SLAM Part 1&mdash;Mapping
![cover image](cover.jpg)
## Introduction
SLAM stands for **s**imultaneous **l**ocalization **a**nd **m**apping and describes a scenario in which a robot builts a map of an (unknown) environment that it is exploring while, at the same time, using said map to localize (/orient) itself. It is a very hard problem, not because it is comprised of **two** sub-problems/tasks, but because of their interaction: both the localizationd and the mapping task amplify each others' errors in a cyclic fashion.

Imagine a robot equipped with a depth sensor, such as the [Kinect](https://azure.microsoft.com/en-us/services/kinect-dk/) camera. Our robot records a [noisy depth map](https://graphics.stanford.edu/~mdfisher/Images/KinectSensors.png). Now it moves a few cm and records another noisy depth map. Let's assume that our robot is not equipped with any other sensors (GPS, wheel (turn) encoders, ...), meaning the only thing it has to go off of in order to localize itself is the map it built so far. How does it do that? By [registering](https://en.wikipedia.org/wiki/Iterative_closest_point) (/aligning) the current depth map with the previous one. Not only are both inputs noisy, the algorithm itself is imperfect (it does not guarantee to produce an error of 0 *even given perfect inputs*). Naturally, our new location estimate will be imprecise. This is the main challenge of SLAM: Noisy data leads to an inaccurate map which leads to an inaccurate pose estimate which in turn makes the map even worse. This effect, called drift, can be seen in the following scan obtained with [Kinect Fusion](https://www.youtube.com/watch?v=RR2fhy35oaY):

![drift in 3D scan](drift.jpg)

What is supposed to be a straight shop facade clearly bends along the reconstrution (/scanning) path.

In this assignment we want to focus on mapping alone, the pose will be provided for your. While mapping with a known pose is (almost) trivial, it is still an essential part of SLAM which you thus need to master. There are many mapping algorithms with different datastructures and trade-offs worth learning about. In later assignments we will combine both our localization and mapping knowledge into a complete SLAM algorithm.

## Mapping
SLAM is not a concrete algorithm, it's more of a meta-algorithm (/concept). You can substitute dozens of concrete algorithms for its two main components, *localization* and *mapping*. You've already learned two localization methods: (E)KF and particle filters. There are many more. Similarly, there are [dozens](http://robots.stanford.edu/papers/thrun.mapping-tr.pdf) of possible mapping algorithms. In this assignment we want to implement an algorithm known as ["depth map fusion"](https://www.microsoft.com/en-us/research/wp-content/uploads/2016/02/ismar2011.pdf). It belongs to the category of *dense, grid-based* algorithms and is in its nature **very** similar to [free space carving](https://www.cs.toronto.edu/~kyros/pubs/00.ijcv.carve.pdf) and [occupancy grids](https://ieeexplore.ieee.org/abstract/document/30720).

**Pros**:

- Produces very high-fidelity (3D) maps, suitable for the navigation of flying drones or for photo-realistic renderings
- [Embarassingly parallel](https://en.wikipedia.org/wiki/Embarrassingly_parallel)
- Simple to implement

**Cons**:

- Compute-and memory-intense (somewhat alleviated by recent advances in hardware, such as [Nvidia Jetson](https://developer.nvidia.com/buy-jetson))
- Requires additional post-processing to, for example, extract polygon meshes for rendering or graphs for path planning (i.e. using [marching cubes](https://en.wikipedia.org/wiki/Marching_cubes))

Simply put, depth map fusion (DMF), as any other grid-based algorithm, quantizes (/divides) the world into logical cells and then determines which cells are occupied (by obstacles typically) and which cells are free (to move through):

![occupancy grid visualization](grid.jpg)

The cell size determines the map resolution. A self-driving car might require 10 cm, while a domestic robot might want to use a resolution of < 1 cm so it doesn't bump into things or knock things over. There's an inherent accuracy&hArr;memory consumption/speed tradeoff. In practice, DMF does a few more things in order to handle (depth) sensor noise and improve accuracy which we will learn about shortly.

## Preliminaries
The implementation of DMF (or any other grid-based mapping algorithm) requires strong knowledge of linear algebra, especially

- matrices and the role they play in transformations (translation, rotation, projection, conversion between coordinate systems and spaces)
- spaces such as "world space", "eye/camera space", "clip space", and normalized device coordinates (NDC)
- homogeneous coordinates
- left- vs right-handed coordinate systems (and how to convert between them)
- column- vs row-vectors and their impact on matrix multiplication order

This knowledge is also quintessential for robotics in general (for example calculating the kinematics of a robotic arm), computer vision, computer graphics, ... and is beyond the scope of this readme file. If the above terms are new to you please carefully read through/watch the following tutorials:

- [OpenGL Matrices](http://www.opengl-tutorial.org/beginners-tutorials/tutorial-3-matrices/)
- [OpenGL - clip space, NDC, and screen space](https://www.youtube.com/watch?v=pThw0S8MR7w)

## Mapping without Noise
asdf