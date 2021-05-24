# Texture-less 6-D Object Instance Recognition

This repository provides code to reproduce results from the paper: [RGB Gradient Based Polar-Fourier Descriptor for Texture-less 6-D Object Instance Recognition](https://github.com/davreixach/RGBpfDescriptor/blob/master/rgb-gradient-based_descriptor-REIXACH.pdf).

Here is the method summarized:

Feature Encoding                         |-
-----------------------------------------|-----------------------------------------
<img src="https://github.com/davreixach/RGBpfDescriptor/blob/master/RGBpfDescriptorLatex/images/equations1.png" width="300"> | <img src="https://github.com/davreixach/RGBpfDescriptor/blob/master/RGBpfDescriptorLatex/images/duck_input.png" width="115"> <img src="https://github.com/davreixach/RGBpfDescriptor/blob/master/RGBpfDescriptorLatex/images/duck_grad.png" width="115"> <img src="https://github.com/davreixach/RGBpfDescriptor/blob/master/RGBpfDescriptorLatex/images/duck_polarhist.png" width="115"> <img src="https://github.com/davreixach/RGBpfDescriptor/blob/master/RGBpfDescriptorLatex/images/duck_fourier.png" width="115">
<!-- -----------------------------------------|----------------------------------------- -->
Configuration Space Partitioning         |-   
-----------------------------------------|-----------------------------------------
<img src="https://github.com/davreixach/RGBpfDescriptor/blob/master/RGBpfDescriptorLatex/images/equations2.png" width="300"> | <img src="https://github.com/davreixach/RGBpfDescriptor/blob/master/RGBpfDescriptorLatex/images/clusters_duck.png" width="210"> <img src="https://github.com/davreixach/RGBpfDescriptor/blob/master/RGBpfDescriptorLatex/images/results.png" width="260">


### Preliminaries
---

1. Clone the repository
    ```shell
    $ git clone https://github.com/davreixach/RGBpfDescriptor.git
    $ cd RGBpfDescriptor/code
    ```

3. Read Readme.m and proceed. .mat files are needed to reproduce the results and are not included within the repository, contact the author for further details.
