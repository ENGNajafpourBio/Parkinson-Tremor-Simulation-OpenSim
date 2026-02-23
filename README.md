# Parkinson’s Tremor Simulation: Rest to Action Transition

## 📌 Project Overview
This project presents a high-fidelity biomechanical simulation of Parkinsonian tremors using the **MOBL_ARMS_41** musculoskeletal model in **OpenSim**. 

### 📺 Simulation Preview
<p align="center">
  <video src="لینک_ویدیو_را_اینجا_رها_کن" width="100%" controls autoplay loop muted></video>
</p>

*The simulation lasts 60 seconds and accurately captures the transition from **Rest Tremor** to **Action/Intention Tremor** as the model attempts a reaching task.*

## 🛠️ Technical Implementation
- **Model:** MOBL_ARMS_41 (Upper Limb Musculoskeletal Model).
- **Duration:** 60 Seconds.
- **Simulation Logic:** - **0-30s (Rest Tremor):** Modeling low-frequency, pill-rolling tremors (4-6 Hz) in the distal joints.
  - **30-60s (Action Tremor):** Simulating the transition during a goal-oriented movement, incorporating increased amplitude and muscle co-contraction.

## 📂 Repository Structure
- `/Models`: Contains the `.osim` file.
- `/Motions`: The resulting `.mot` file (`action_tremor_motion.mot`).
- `/Scripts`: MATLAB script (`realistictremor.m`) used for frequency analysis and signal generation.

## 🚀 How to Run
1. Load the `MOBL_ARMS_41.osim` in OpenSim 4.x.
2. File -> Load Motion -> Select `action_tremor_motion.mot`.
3. Press Play to observe the transition dynamics.

## 🔬 Clinical Significance
This simulation serves as a digital twin for testing tremor-suppression algorithms and wearable assistive devices (like active orthoses) without the need for initial human trials.

---
**Contact for Collaboration:** [LinkedIn](https://www.linkedin.com/in/mohammad-najafpour-56219a222) | [X (Twitter)](https://x.com/ENGNajafpourBio)
