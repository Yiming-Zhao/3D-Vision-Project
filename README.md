
# Assignment 3 Boids


# **Sorry for late submission**
It seems that I only do the git commit but forget to push.

As I found it, the deadline has already be exceeded.

I understand that there will be point deduction.

But when it will be counted as two days, 20% is really heavy for me.

May ask a deduction with 10% because the submission time will not exceed 24:00 of the first day

When it is not possibl, 15% could also be kind, when the submission time will not exceed 6:00 of the second day


##  Time Integeration

---
### Oberservation:
* Test with different step size `0.005` ,`0.02` and `0.1`.
* A simulated centripetal force is applied on each boid. the force is toward origin and its scale is associated  with distance to origin and current velocity. 

1. With larger step size, the movement of boids tends to be unstable. The oscillation is fierce.
2. The basic method shows more unstablity than two advanced methods
3. When step size is set to 0.1, the basic method is crashed at first all the boids fly always.
4. When step size is set to 0.05, the basic method comes with obvious oscillation after some time.

[step size `0.005`](https://drive.google.com/file/d/1Yehi2V6oHmF-nlVMIf4rNHbzgzHvLIu8/view?usp=sharing)

[step size `0.01`](https://drive.google.com/file/d/1qibKxtNJ52BrNjCA9q8tRriKfz-CHmdw/view?usp=sharing)

<pre>
the step size will use <b>0.005</b> in following subtasks 
</pre>

## Behaviors Implementations

Please Check the following videos 

[Cohesion Alignment Separation](https://drive.google.com/file/d/1EpoYw9iaS699Ixz6akdYmS5KAw-8ON98/view?usp=sharing)

[Collision Avoidance](https://drive.google.com/file/d/1EpoYw9iaS699Ixz6akdYmS5KAw-8ON98/view?usp=sharing)
* The velocity of boid will be forcely set to 0 when it try to move inside obstacles(collide).
* In this Collision Avoidance, I change the GUI that could add and adjust several collision obstacles. The obstacles could be dragged by mouse with right button pressed
* The boids could simlutaneously avoid collision to multi object.
* design a Tangent-Method to avoid potential collision along velocity direction and drive boids to steer around obstacles but still move to desired destination
 ![tangent][tangent.png]
## Collaborative Behavior
The leader is represented by a blue dot and its position could be dragged by mouse with left button pressed.
[Collaborative Behavior](https://drive.google.com/file/d/1uHpXAR8V0MmPw5t3_alrIcF_SfHO3v6-/view?usp=sharing)
## Adversarial Behavior

In this task, the generation radius, in which the boids could reproduce the offsprings, is the half of the elimination radius, in which the boid from other groups will be eliminated.
The settiing is logical and realistic.

[Adversarial Behavior](https://drive.google.com/file/d/1SwKCka-KG-RqUHh9yZgocVjzIJzTpaKS/view?usp=sharing)

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

## Conclusion:
In this assignment, I have understand the behaviors of boids.
To control boid only with force on it is challenging but most interesting
This assignment also provides me more practice with imGUI.
If more time, the 3D boids can be implemented.
