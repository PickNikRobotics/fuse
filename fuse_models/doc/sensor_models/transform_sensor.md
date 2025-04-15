# Transform Sensor

## [Source Code](../../src/transform_sensor.cpp)

## Explanation

This sensor is intended to be used to estimate the position of a body relative to some sensor when only measurements of points on that body can be taken.
For example, this sensor is useful in the case of measuring april tags on some external body and using those measurements to estimate the full pose and twist of that body.
An example of this application can be seen [here](../../../fuse_tutorials/config/fuse_apriltag_tutorial.yaml) (or try `ros2 launch fuse_tutorials fuse_apriltag_tutorial.launch.py` to see the tutorial in action).

More generally, this sensor takes a transform between one frame in the set `estimation_frames` and one transform from the set `transforms`. When it receives such a transform, it will then apply a transform to this measurement to change it to be a measurement of the given `target_frame`. Then, it generates a 3D pose transaction for use by its parent optimizer.

If you want to use multiple `estimation_frames`, you should define a `base_frame`.
If this frame is non-empty, then you should set the `map_frame_id` and `world_frame_id` in the publisher to the same value.
When this is set, the sensor will transform its pose to be a measurement in `base_frame` of `target_frame`, and the odom it produces will be in `base_frame`. This can be used when e.g. you have multiple cameras, all positioned relative to `world`, and want them measuring the same object.
This allows the transform sensor to use any number of cameras to estimate the pose of some other object.

A slightly confusing aspect of the sensor is the need for multiple definitions of the target frame. Every frame in `transforms` needs a corresponding `target_frame` to be published for it to be used. This is simply because `tf` uses a tree data structure, and the target frames are leaves. These should all end up being in the same global location (discounting measurement noise) but will have different names.

Finally, an optional feature of the transform sensor is outlier filtering. This can be enabled by setting `filter_outliers` to `true`. This filters outliers based on a simple distance threshold. This distance value and the associated timeout before any distance is allowed (to avoid the situation in which measurements are lost for a period and return but are ignored) can be configured with `outlier_distance` and `outlier_time_threshold`, respectively. Mahalanobis distance thresholding was explored as well, but performs poorly in most cases with external forcing (because this is generally not accounted for in the model or measured).

## Example

For example, say we have `estimation_frame` `camera`, `transforms` {`apriltag_1`, `apriltag_2`}, and `target_frame` `robot_center_of_mass`.

Our transform sensor receives a transform (published on the standard `/tf` topic) from `camera` to `apriltag_1`.
This measurement transform has translation (-1, 0, 0) (ignore rotation in this example).
This is the transformation to apply to go from `camera` frame to `apriltag_1` frame, so the translation is the negative of the position of the april tag in camera frame.

There is a static (though it doesn't *need* to be static) transform published from `apriltag_1` to `target_frame_apriltag_1`.
This transform has translation (-0.25, -0.5, 0) (again, ignoring rotation).
It should be noted that this is the negative position of the target frame in the april tag's frame.

Our sensor will take the measurement from `camera` to `apriltag_1` and transform it to a pose from `camera` to `target_frame` by 'adding' the transform from `apriltag_1` to `target_frame_apriltag_1`.
Thus, our measurement of the actual position of `target_frame` that will be used to generate the constraint will be (1.25, 0.5, 0).
It is not (-1.25, -0.5, 0) because the translation of `T_a_to_b` is the negation of the pose of `b` in frame `a`.

Finally, this transaction is sent to the optimizer, as normal.

To see this sensor running in an example, run `ros2 launch fuse_tutorials fuse_apriltag_tutorial.launch.py`.
