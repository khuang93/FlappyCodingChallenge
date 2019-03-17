This is Kailin Huang's version of the Flappy Bird Challenge.

## Additional requirements to compile the code
* install the PCL library for point cloud handling.

## General Idea

The most important task is to find the correct opening gap reliably. To make it simple, if we only see one column of stones, we can find the largest distance between 2 stones and that would be the gap. As we can only receive laser scanned point clouds, we try to find the largest gap in the verticle direction between 2 points. In order to do this, we have to ensure that we only consider points from the next incoming column of stones. Points reflected from stones further away has to be filtered out.

When we find a gap, to pass the y coordinate of the middle of the gap further and have to control the bird to fly to the desired y coordinate. Because the x position is less relevant and we only need to get to the needed y position before we ran into a stone, we only control the velocity in x direction, while we do position control in y direction. Because the gap is not much bigger than the bird, we have to ensure the y position control is fast, but cannot overshoot. Furthermore, we have to minimize the residual position error in y to make sure the bird does not hit anything.


## How the code works

We can generally devide the code into a perception part and a control part. The perception part is mainly to build a point could map from the laser scans and then find out there the opening gap is located. The control part then needes to control the bird using acceleration inputs to let it fly accurately through the gap.
### Perception
* **Creation of point cloud**
    First we need to know the position of the bird. It is calculated by integrating its velocity over the time.

    $$pos=\Sigma v_i * \Delta t$$

    From the position of the bird we can calculate the points received from the laser scanner. We can easily calculate the position of the points using the length and angle of each laser beam.

    $$point.x= pos.x + l*cos(\alpha)$$
    $$point.y= pos.y + l*sin(\alpha)$$

* **Filter/Selection in the point cloud**
  The only interesting part of the point cloud is the next set of stones. If the bird is currently in the middle of the gap, then the current set of stones is also of interest because we cannot see backwards and have to rely on what has been observed before.

  We limit the point cloud in the X range around the current position of the bird is actually used later. With the observation that the distance between 2 columns of stones is ca. 2m, we set to filter out points more than 1.8m away. To make sure the bird does not hit the stone behind it while coming out of the previos gap, we keep pionts that are less than -0.18m behind the bird.

  To remove the influence of the wall, we set to filter out points which are close to or reflected directly by the wall. We observe that the bird spawns each time from the same height and using that, the wall is at 2.5m and -1.5m in y, which means we can easily filter them out.

* **Finding the gap**
  Basically, we sort the points and then find the largest gap between 2 neighboring points. We set a minimum size of the gap to make sure the gap we found is large enough. If we find no gap, it means the gap it very far above or below the bird so it cannot be seen by the lasers, in that case we try to move the bird up / down to see if we can see the gap then.

  The calculation of the gap is carried out each timestep. To prevent false positive gaps ruining the control of the bird, we set a rule that if the current gap position is a large change to the previos gap position, we require the current gap calculation to be consistent for at least several frames (currently set to 8) before we trust it and send it out.

  To further make it earsiert for the control, if the gap's y position change is large, it means the bird has to fly a very steep line to go to the next gap, which means the control of y can overshoot and the bird hits the stone. To prevent this, if the gap's y position change is large, we revert the gap's y posotion by 0.03m to make more room in case the y position control overshoots.


### Control
The whole control system is implemented in the callback of velocity and sends out the desired acceleration values.

* **Control of X direction**
  We set a base x velocity of 0.32m/s and increase the velocity linearly to the distance to point in the point cloud, which has the smalles x value. If the bird is far away from the obstacle is flys faster and vice versa. If the distance to the point is negative, it means the bird is currently inside of the gap, then we try to keep the bird stable in the y direction, even if the calculated y value gap changes.

* **Control of Y Direction**
  The first idea is to do a simple velocity P control. Desired velocity is proportional to the distance in y from the bird to the gap and the acceleration output is linear to the desired velocity. It works when the change of y position is moderate, but by large change of y posisiton, the control overshoots.

  We changed then into a proper PID control of the position. For this, we added variables to remember the velocity and position error of the previos timestep. We have a linear component of the position error, then a integral part which is the addition of the previos position errors and at the end, to prevent overshoot, a D part which is proportional to the changing rate of the position. In addition to this, I added a fourth term which is a velocity P control to make the controller react faster. This is implemented in line 120 of the code.

```
  acc_cmd.y = kp * distY + kp_vy * (vy_desired - msg->y) + ki * integral_y * dT + kd * diff_y * this->FPS;

```
  The PID system required some tuning. We first set the P and D values and the P part for the y velocity. Then we tuned the D part to make it not overshoot. The integral part is added at the end to make very small corrections to reduce residual position errors.


   




* Data output & analysis
  To make the analysis and tuning faster, we output the current position of the bird and the current desired y positions to a csv file and plot the curves to analyse the system's behaviours.

  For the perception of the gaps, as we treat the point clouds using the PCL library, we can easily write it out as ply files and plot the points.

## Difficulties in the task
* Consistency of the point cloud map
One key issue which I haven't solved completely is the point cloud's consistency. When the bird is moving fast, the points from different time steps are not consistent and the whole point cloud is then biased. I assume this problem comes from the discrete numerical integration of the flappy bird's position, we can become inaccurate  if the velocity changes fast. To reduce this problem I select to only keep the points from the last 5 seconds in the map and discard older points. Otherwise some points could prevent us to find the correct gap. 

* Bird changing position very fast before it completely leaves the gap. This is solve, as described above, by identifying that the x distance is negative and then override the y velocity control with 0.

* Detection of gap which is acutally not the gap
  As described above, this is solved by adding a consistency check to make sure the output of the gap position is consistent over time before we trust it and control the bird to fly there.

## Possible improvements
I have several functions implemented but not really tested and used. They are there for detection the points which are very close to the bird. If the are detected we can then add some micro-adjustments to make sure the bird can steer away from them.

Another problem is, the code contains many parameters which are fine tuned during testing. Some part of the code are done "quick and dirty", for example we could use RANSAC to detect walls above and below the bird, if the distance of the wall are not fixed. But in this case I chose to use the regularities to simplify the code and using more hardcoded parameters. It is clear that for a more general-purpose use of the algorithm we need to change the way how these hard coded parameters are estimated.

We might also use some reinforcement learning to estimate some of the parameters or even make it end-to-end for the navigation. However I have little experience in reinforcement learning and in this rather simple game, it was sufficient to tune the code manually and add some hard coded rules.