# Week 2 - Markov Localization

---

[//]: # (Image References)
[plot]: ./markov.gif

## Report
* `motion_model()` : Please refer to the lines 45 - 69 lines on `markov_localizer.py`.
* `observation_model()` : Please refer to the lines 78 - 99 lines on `markov_localizer.py`.


## Assignment

You will complete the implementation of a simple Markov localizer by writing the following two functions in `markov_localizer.py`:

* `motion_model()`: For each possible prior positions, calculate the probability that the vehicle will move to the position specified by `position` given as input.
* `observation_model()`: Given the `observations`, calculate the probability of this measurement being observed using `pseudo_ranges`.

The algorithm is presented and explained in class.

All the other source files (`main.py` and `helper.py`) should be left as they are.

If you correctly implement the above functions, you expect to see a plot similar to the following:

![Expected Result of Markov Localization][plot]

If you run the program (`main.py`) without any modification to the code, it will generate only the frame of the above plot because all probabilities returned by `motion_model()` are zero by default.