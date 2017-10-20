### Kidnapped Vehicle Project ###
### Particle Filter ###
Submitted by - Vishal Rangras

The goals for this project are to:

1. Design and implement a Particle filter which can localize itself in the known environment.
2. Initilizing Filter based on GPS data passed to it.
3. Prediction step to add measurements and random Gaussian noise to each particle.
4. Update step to transform vehicle coordinates to map coordinates for vehicle localization and to update weights of particles accordingly.
5. Resampling approach which dies out low probability particles and only highly probable particles survives in the end.
6. Data Association helper function for update step.

**[Rubric](https://review.udacity.com/#!/rubrics/747/view) Points**

### Building the Project and Execution ###

1. Clone this repo.
2. Execute the script build.sh to build the project. `sh build.sh`
3. Start the simulator and exectue the script run.sh to run the project. `sh run.sh` 
4. Use clean.sh to clean the build target. `sh clean.sh`

### Results ###

[image1]: ./data/Result.PNG "Result"

<h3 align="center"> Result <h3>

![alt text][image1]