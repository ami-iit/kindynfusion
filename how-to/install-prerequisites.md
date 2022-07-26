### Installation Prerequisites

In order to set up the environment required for testing the `KinDynFusion` software, we install `robotology-superbuild` using the `mambaforge conda distribution`.

#### Install mambaforge

Please follow the [`mambaforge installation`](https://github.com/robotology/robotology-superbuild/blob/master/doc/install-mambaforge.md) instructions in order go through a `conda-forge` based `robotology-superbuild` installation.

#### Install robotology-superbuild

**P.S. please read the whole subsection before following the instructions through the hyperlinks.**

We will install `conda-forge` provided dependencies to compile and install `robotology-superbuild` from source. The general installation instructions can be found [here](https://github.com/robotology/robotology-superbuild/blob/master/doc/conda-forge.md#source-installation).  The basic steps include,

- creating a new `conda` environment where all the packages will be installed, example `mamba create -n robsub`
- then we activate the environment in the opened terminal `mamba activate robsub`. P.S. it is recommended to create an alias for this command for easy-use, by adding the following line in the `.bashrc` or equivalent file, `alias robsub='mamba activate robsub'`,
- followed by the installation of dependencies for `robotology-superbuild`
- and finally cloning and compiling the `robotology-superbuild`.

At the compilation step, `robotology-superbuild` needs to be configured with the following CMake options, on **Linux** or **macOS**:

```bash
cd robotology-superbuild
mkdir build
cd build
cmake -DROBOTOLOGY_ENABLE_CORE=ON \
      -DROBOTOLOGY_ENABLE_DYNAMICS=ON \
      -DROBOTOLOGY_ENABLE_HUMAN_DYNAMICS=ON \
      -DROBOTOLOGY_ENABLE_DYNAMICS_FULL_DEPS=ON ..
cmake --build . --config Release
```

and on **Windows**,

``` bash
cd robotology-superbuild
mkdir build
cd build
cmake -G"Visual Studio 16 2019" \
      -DROBOTOLOGY_ENABLE_CORE=ON \
      -DROBOTOLOGY_ENABLE_DYNAMICS=ON \
      -DROBOTOLOGY_ENABLE_HUMAN_DYNAMICS=ON \
      -DROBOTOLOGY_ENABLE_DYNAMICS_FULL_DEPS=ON ..
cmake --build . --config Release
```

**IMPORTANT: If we use Visual Studio 2022, we need to use `G"Visual Studio 17 2022"`instead of `G"Visual Studio 16 2022"` in the configuration step.**



It must be noted that by following these instructions, we create a new environment `robsub` to install all the packages within this dedicated environment. There might be cases where `robsub` environment already exists in your system. At this point, one may choose to create a new environment `kindynfusion` and follow through the same installation instructions to keep the testing of `KinDynFusion` software separate from other pre-installed packages so that we do not disturb the existing system configuration. Otherwise, one may choose to reuse the same `robsub` environment to proceed testing the `KinDynFusion` software. However, for the latter option, it is recommended that the repositories installed in the `robotology-superbuild` uses the last tested version commits specified in the [dependencies](./dependencies) page. In case of a fresh installation of `robotology-superbuild` in the `robsub` environment, instead of manually checking out the commit versions, we can pass a custom project tags file during the configuration step of compiling the superbuild as suggested [here](https://github.com/robotology/robotology-superbuild/blob/033df9308927533c925cc9a73276839adf50d66e/doc/change-project-tags.md). We can use the  [robotology.yaml](./robotology.yaml) containing all the relevant project tags as a value to the CMAKE option **`ROBOTOLOGY_PROJECT_TAGS_CUSTOM_FILE`** tag while installing the `robotology-superbuild` from scratch. It must be noted that the passing the custom project tags through this file is applicable only for a fresh installation of the superbuild.

For passing the custom project tags for installing the superbuild, we may compile it with the following commands,

``` bash
cd robotology-superbuild
mkdir build
cd build
cmake -DROBOTOLOGY_ENABLE_CORE=ON \
      -DROBOTOLOGY_ENABLE_DYNAMICS=ON \
      -DROBOTOLOGY_ENABLE_HUMAN_DYNAMICS=ON \
      -DROBOTOLOGY_ENABLE_DYNAMICS_FULL_DEPS=ON \
      -DROBOTOLOGY_PROJECT_TAGS=Custom \
      -DROBOTOLOGY_PROJECT_TAGS_CUSTOM_FILE=<absolute-path-where-you-have-cloned-kindynfusion>/how-to/robotology.yaml \..
cmake --build . --config Release
```



Once, `robotology-superbuild` is properly compiled and installed, it is convenient to add the following line to `.bashrc` file  in **Linux** or equivalent in **macOS or Windows**, for sourcing the packages installed by the superbuild and to have them accessible in the system path,

``` bash
source <where-you-installed-robotology-superbuild>/share/robotology-superbuild/setup.sh
```



Go back to the [main README](./../README.md).