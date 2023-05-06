# Background
This repo is our final project for Carnegie Mellon University course 15-618: Parallel Computer Architecture and Programming, Spring 2023. Key contributors are Matthew Booker and Jaekyung Song.

# Overview
We parallelized Conflict Based Search (CBS) in multiple different ways, each with its own branch. The key ones are:
- HDA_SAS: implementation of Hash Distributed A-star (HDA) using Shared Address Space (SAS) model. Used to parallelize the low-level search of CBS
- parallel_cbs_naive: parallelization of the high level search of CBS. Each core is given a subtree to expand and search within
- parallel_cbs_v2.0: parallelization of high level serach of CBS. Cores will generate and share work, focusing on the best nodes in the constraints tree and balancing workload

# Website
Website for this project can be found [here](https://jsongcmu.github.io/parallel_CBS/)
