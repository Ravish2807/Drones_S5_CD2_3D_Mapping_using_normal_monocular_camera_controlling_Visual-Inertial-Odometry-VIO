# Third-Party Notices & Attribution

This repository incorporates concepts, conventions, and references from the following third-party projects and datasets. We respect all respective licenses and academic attributions.

---

### 1. OpenVINS
* **Project**: Open Visual-Inertial Navigation System (RPNG, University of Delaware)
* **Website / Repo**: https://github.com/rpng/open_vins
* **Documentation**: https://docs.openvins.com/
* **License**: GNU General Public License v3.0 (GPL-3.0)
* **Role**: Primary theoretical reference for MSCKF formulation, continuous-discrete propagation, sliding window cloning, nullspace projection, and camera-IMU calibration conventions.

---

### 2. EuRoC MAV Dataset
* **Project**: The EuRoC micro aerial vehicle datasets
* **Authors**: Burri, M., Nikolic, J., Hurzeler, C., Caprari, G., & Siegwart, R. (ETH Zurich)
* **Website**: https://projects.asl.ethz.ch/datasets/euroc-mav/
* **Citation**: Burri et al., "The EuRoC micro aerial vehicle datasets", International Journal of Robotics Research (IJRR), 2016.
* **Role**: Primary benchmark dataset for validating VIO trajectory estimation against motion capture ground truth. *Raw datasets are not redistributed in this repository; download instructions and configuration loaders are provided.*

---

### 3. DROID-SLAM
* **Project**: Deep Visual SLAM for Monocular, Stereo, and RGB-D Cameras
* **Authors**: Zachary Teed and Jia Deng (Princeton University)
* **Website / Repo**: https://github.com/princeton-vl/DROID-SLAM
* **License**: BSD-3-Clause License
* **Role**: Visual-SLAM baseline and reference architecture as utilized in the foundational 3D reconstruction base paper.

---

### 4. CDS-MVSNet
* **Project**: Multi-View Stereo with Curved and Dense Surfaces
* **Authors**: Khang Truong et al.
* **Website / Repo**: https://github.com/TruongKhang/cds-mvsnet
* **Role**: Reference architecture for the subsequent dense 3D reconstruction stage.

---

### 5. TartanAir V2 Dataset
* **Project**: AirSim-based visual SLAM dataset for complex environments
* **Authors**: Wenshan Wang et al. (Carnegie Mellon University)
* **Website**: https://tartanair.org/
* **License**: CC BY 4.0 (Dataset) / MIT License (Toolkit)
* **Role**: Optional resource for synthetic multi-modal perception and future learning-based state estimation studies.

---

### 6. Base Paper Reference
* **Citation**: Cujó Blasco, J.; Bemposta Rosende, S.; Sánchez-Soriano, J. *Automatic Real-Time Creation of Three-Dimensional (3D) Representations of Objects, Buildings, or Scenarios Using Drones and Artificial Intelligence Techniques.* Drones 2023, 7, 516. https://doi.org/10.3390/drones7080516
