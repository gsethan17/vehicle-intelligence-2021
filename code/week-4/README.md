# Week 4 - Motion Model & Particle Filters

---

[//]: # (Image References)
[empty-update]: ./empty-update.gif
[example]: ./example.gif

## Report

* `update_weights()`  
    * Get the visible landmarks with distance between ego-vehicle and each landmarks and sensor range which was defined already.  
    I refer to `distance` function which was defined on `helpers.py`.  
    Please refer to line 82 to 92 on `particle_filter.py`
    * Transform measurement coordinates from the particle's coordinate which has the origin as ego-vehicle's position to the map's coordinates.
    Please refer to line 96 to 101 on `particle_filter.py`
    * Get the associated landmarks corresponding to each measurements.
    I refer to `associate` function which was predefined on `particle_filter.py`.
    Please refer to line 109 to 110 on `particle_filter.py`
    * Calculated probability of observing the measurements if ego-vehicle is on each particle positions.
    The calculated probability is used for weights of each particles.
    I defined the `norm_pdf` function on `helpers.py` and refer the function to get the multi-variate Gaussian distribution.
    Please refer to line 119 to 128 on `particle_filter.py` 

* `resample()`
    * Draw the particle samples distribution from 0 to 1.
    Please refer to line 137 to 138 on `particle_filter.py`
    * Choose the new particles from origin particles based on each weights.
    I used method of low variance sampling and `copy.deepcopy()` method for guarantee the independence with new particles.
    Please refer to line 143 to 160 on `particle_filter.py`

## Assignment

You will complete the implementation of a simple particle filter by writing the following two methods of `class ParticleFilter` defined in `particle_filter.py`:

* `update_weights()`: For each particle in the sample set, calculate the probability of the set of observations based on a multi-variate Gaussian distribution.
* `resample()`: Reconstruct the set of particles that capture the posterior belief distribution by drawing samples according to the weights.

To run the program (which generates a 2D plot), execute the following command:

```
$ python run.py
```

Without any modification to the code, you will see a resulting plot like the one below:

![Particle Filter without Proper Update & Resample][empty-update]

while a reasonable implementation of the above mentioned methods (assignments) will give you something like

![Particle Filter Example][example]

Carefully read comments in the two method bodies and write Python code that does the job.
