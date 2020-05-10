# MECH7710 Path Planning

Project for MECH7710 - Optimal Estimation implementing trajectory replanning methods for controlling a vehicle towards a goal in the presence of other vehicles.
Vehicle Parameters are from a 2003 Infiniti G35.

## Methods to Implement
- [x] [A*](https://en.wikipedia.org/wiki/A*_search_algorithm)
- [ ] D*
- [ ] [D*-lite](http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf)
- [x] Follow-the-gap

## Vehicle Models
- [x] 2 DOF Bicycle Model

## Simulation Setup
### Features
- Class oriented approach, everything is modular
- Each vehicle is comprised of 3 things:
    1. motion model class (currently a bicycle model)
    2. Filtering class (basic kalman filter estimating remote vehicle's orientation and yaw rate)
    3. Navigation class (currently uses Follow-the-gap (FTG) or A*)
- Simulation can contain N number of vehicles, each with different components and goal destinations

### How to run
- demo.m runs the whole simulation
- buildWorld() is where the vehicles and goal locations are created
- FilterClass stores all process and sensor noise values, as well as a generic vehicle model
