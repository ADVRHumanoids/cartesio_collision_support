# cartesio_collision_support
Collision avoidance support for CartesIO. A the moment, the support for collision avoidance is provided by means of
 - a collision avoidance task, named `CollisionTask`
 - a collision avoidance constraint, named `CollisionConstraint`

Support for the first (task-based) version is experimental.

## How to include inside a stack file
```yaml
Collision:
    type: CollisionConstraint  # or CollisionTask
    lib_name: libcartesio_collision_support.so
    bound_scaling: 0.1  # to reduce speed before collision (< 1), only for constraint version
    lambda: 0.1  # only for task version
    max_pairs: 10  # maximum number of simultaneously active collisions
    distance_threshold: 0.01  # minimum distance between link pairs
    pairs:  # list of all link pairs to be checked
     - [ball1, ball2]
```
