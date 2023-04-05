# Object-level Fusion

[![CI](https://github.com/icaropires/objectlevel_fusion/actions/workflows/ci.yml/badge.svg)](https://github.com/icaropires/objectlevel_fusion/actions/workflows/ci.yml)

**Summary:** This repository was firstly developed when writing a bachelor's thesis, and contributes to the fusion of data from multiple sensors (the perception ones) to get the best information from each sensor.

Object-level fusion performs fusion at a higher level of abstraction and, for this reason, contributes to modularity and reuse. This work implements a software solution to address part of this reimplementation problem. It's composed of ROS 2 packages and implements the object list preprocessing from the fusion layer of an object-level fusion architecture. This preprocessing is composed of the **spatial and temporal alignments**, plus the **objects association**. Finally, this preprocessing was validated with an experiment using [CARLA](http://carla.org/) self-driving simulator, using as main metric the number of failed associations in some test case scenarios ([check experiment](experiments/carla)).

## Bachelor's thesis document

The document version that was reviewed and approved by the thesis comittee can be found at:
* [University repository](https://bdm.unb.br/handle/10483/30630)
* [thesis/bachelors\_thesis.pdf](./thesis/bachelors_thesis.pdf)

## Architecture Layers

> _Checked boxes means implemented in this repository_

* [ ] Sensor Layer
  - ...
* [ ] Fusion Layer
  - [x] Spatial Alignment 
  - [x] Temporal Alignment
  - [x] Object Association\*
  - [ ] State and Covariance Fusion
  - [ ] Existence Fusion
  - [ ] Classification Fusion
* [ ] Application Layer
  - ...

> \*_implemented a simpler version_

## Requirements

### Dockerized execution

* Linux
* Docker
* Docker Compose

### Local execution

* Linux
* ROS 2 Foxy.
* Others: `rosdep` (from ROS) will probably install them. Check [Dockerfile](docker/Dockerfile.dev), and the packge files from [fusion layer](fusion_layer/package.xml), and [object model](object_model_msgs/package.xml).

## Using

Much of the usage is facilited by the [run](./run) script. Under the hood it just calls `docker-compose`. Feel free to customize your execution by directly calling `docker-compose` if you're more experienced.

### Executing

Execute the instructions of one of the following subsections, register your sensors, and then publish your object lists :smile:

#### Executing (Easy, dockerized way)

Execute:

```bash
$ ./run

# Or in background:
$ ./run -d
```

When the application is up, it will be waiting for messages of type [`object_model_msgs/msg/ObjectModel`](object_model_msgs/msg/ObjectModel.msg) on the topic `objectlevel_fusion/fusion_layer/fusion/submit`, and returning the list of global objects being tracked on the topic `objectlevel_fusion/fusion_layer/fusion/get`.

#### Executing in a ROS 2 workspace

Clone this project in your ROS workspace and follow the ROS 2 procedures: [ref](https://docs.ros.org/en/foxy/Tutorials/Creating-Your-First-ROS2-Package.html).

### Registering/removing sensors and publishing object lists

Check some examples in [examples](./examples) (bash and python available).

### Running (unitary) tests

With the application **up**, tests can be run with:

```bash
./run tests
```

### Development flow

After editing the source code, if the application is up, first bring it down (calling `./run down` if running in background, otherwise just `CTRL+C` in the terminal it's running), then:

```bash
./run compile
```

then, bring the application up again (`./run up`). Now, the modified should be in execution.

### Initializing a shell

To run a shell in the container where the application is running, just execute (with the application **up**):

```bash
./run shell
```

### Other commands from `run`

To see a list and descriptions, execute:

```bash
./run help
```

## How to contribute

* Creating [issues](https://github.com/icaropires/objectlevel_fusion/issues) (questions, bugs, feature requests, etc);
* Modifying the repository: [pull requests](https://github.com/icaropires/objectlevel_fusion/pulls). Just make sure to describe your changes and that everything is working.
