# Assignment 3 Boids

##  Time Integeration
---
### Oberservation:
* Test with different step size `0.005` ,`0.02` and `0.1`.
* A simulated centripetal force is applied on each boid. the force is toward origin and its scale is associated  with distance to origin and current velocity. 
### Conclusion:
1. With larger step size, the movement of boids tends to be unstable. The oscillation is fierce.
2. The basic method shows more unstablity than two advanced methods
3. When step size is set to 0.1, the basic method is crashed at first all the boids fly always.
4. When step size is set to 0.05, the basic method comes with obvious oscillation after some time.

[step size `0.005`](https://www.google.com)

[step size `0.01`](https://www.google.com)

<pre>
the step size will use <b>0.005</b> in following subtasks 
</pre>

## Behaviors Implementations

Please Check the following videos 

[Cohesion Alignment Separation](https://www.google.com)

[Collision Avoidance](https://www.google.com)
* The velocity of boid will be forcely set to 0 when it try to move inside obstacles(collide).
* In this Collision Avoidance, I change the GUI that could add and adjust several collision obstacles. The obstacles could be dragged by mouse with right button pressed
* The boids could simlutaneously avoid collision to multi object.
* design a Tangent-Method to avoid potential collision along velocity direction and drive boids to steer around obstacles but still move to desired destination
  
## Collaborative Behavior
The leader is represented by a blue dot and its position could be dragged by mouse with left button pressed.
[Collaborative Behavior](https://www.google.com)
## Adversarial Behavior

In this task, the generation radius, in which the boids could reproduce the offsprings, is the half of the elimination radius, in which the boid from other groups will be eliminated.
The settiing is logical and realistic.

### No strategy
It could be obversed that the new boid is generated in the middle of two boids from same group with the 0 velocity of those two boids when the two boids are close enough. The single boid from other group will be eliminated. With time increasing, all the offsprings from same group will form many small cluster. The initial parent still move if they were not eliminated. 

### Strategy

It should be noticed here that strategy should increase the relative population. The strategy is also simple.
1.  Firstly, the single boid(no allies in generation radius) will seek the ally and move to it within generation radius. --**`Reproduce`**
2.  After the boid reproducing offsprings, the allied boids which has a close distance to each other will seek the nearst hostile boid and move together to its position to eliminate it --**`Elimination`**. 
3.  Meanwhile, every boids will also try to approach its mean position of all allies. --**`Uniting`**.

### Observation:

When the strategy is applied only on one group, this group is firstly going to together as a cluster. And then, they fastly move to some boids from the other group. The movement of the group with strategy is more active.
When the two groups use the strategy. the boids will tend to form two main clusters with time increasing.

