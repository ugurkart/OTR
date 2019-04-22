# Object Tracking by Reconstruction with View-Specific Discriminative Correlation Filters

This is the Matlab side of the implementation for the paper published in the proceedings of IEEE Conference on Computer Vision and Pattern Recognition (CVPR) 2019.

## Publication
Uğur Kart, Alan Lukežič, Matej Kristan, Joni-Kristian Kämäräinen and  Jiří Matas. ''Object Tracking by Reconstruction with View-Specific Discriminative Correlation Filters.'' In Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (CVPR), 2019.</br>

<b>BibTex citation:</b><br>
@InProceedings{Kart_CVPR_2019,<br>
Title = {Object Tracking by Reconstruction with View-Specific Discriminative Correlation Filters},<br>
Author = {Kart, Uğur and Lukežič, Alan and Kristan, Matej and Kämäräinen, Joni-Kristian and Matas, Jiří},<br>
Booktitle = {CVPR},<br>
Year = {2019}<br>
}

## Contact

Uğur Kart, e-mail: ugur.kart@tuni.fi </br>

## Installation and demo
* Our method works by running the Matlab and C++ code (a modified version of Co-Fusion) simultaneously. Therefore, both of them need to be present on the same machine (can be also done on separate machines but we haven't tried that) and the necessary paths are set correctly.
* Clone git repositories: </br>
    $ git clone https://github.com/ugurkart/otr.git
    $ git clone https://github.com/ugurkart/co-fusion.git
* Install Co-Fusion and its dependencies
* Compile mex files running compile.m command </br>
	Set <i>opencv_include</i> and <i>opencv_libpath</i> to the correct OpenCV paths
* Please note that original PTB has many synchronization and registration errors. Therefore we use the depth images provided by Bibi et al. CVPR 2016.
* Build Co-Fusion into a binary
* Set necessary paths both on Matlab and Co-Fusion side.
  	Make sure to set correct calibration parameters for Co-Fusion.
* Use run_csr.m script for the visualization of the tracker and to replicate our results reported in the paper. </br>
	
## Project summary
A major limitation of the standard RGB-D trackers is
that the target is inherently considered as a 2D structure,
which makes dealing with appearance changes related even
to a simple out-of-plane rotation highly challenging. We
propose a novel long-term RGB-D tracker – Object Tracking by Reconstruction (OTR) – which is based on online
3D reconstruction of the tracked target to learn a set of
view-specific discriminative correlation filters (DCFs). The
3D reconstruction provides two important cues: (i) its 2D
projection generates an accurate spatial support for constrained DCF learning and (ii) point-cloud based estimation of 3D pose change is used to select and store DCFs associated with specific object viewpoints. OTR is extensively
evaluated on two challenging RGB-D benchmarks, Princeton Tracking Benchmark and the STC Benchmark, and sets
the new state-of-the-art by a large margin to the current top
performers on these benchmarks.
