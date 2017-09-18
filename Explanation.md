# CarND Controls MPC Project

## Discussion of Rubric points

### 1. Student describes their model in detail. This includes the state, actuators and update equations.

Kinematic model is used in this project to define Vehicle Model. Kinematic models are basically ignores the dynamic forces. This makes the model more tracable and non-linear.

Code can be seen implemented in MPC.cpp in class FG_eval.

Following are the state values - px (Position in X), py (Position in Y), psi (Car Direction of Heading), cte (Cross Track Error), epsi (PSI Error), v (Velocity)

Following are the actuators - steering angle, throttling

Following are the Model update equations used:

* x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
* y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
* psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
* v_[t+1] = v[t] + a[t] * dt
* cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
* epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

### 2. Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

The values N & dt were used from the video lectures of the course.

For N and dt, I started with the values provided from the lectures. I tried tweaking the values to get better results. Mainly increasing the values but the speed of simulation gets impacted if we increase the values. Higher the value, slower the simulation speed.

For N I tried 10, 15, 20, 25 and for dt 0.1, 0.05 and 0.001. But the values from the video lecture were working better.

Finally I decided to use 20 and 0.05

I tried increasing the ref_v 30, 40, 50, 60, 100 so I decided to finalize the ref_v at 40.

Final values -

          size_t N = 20 ;
          double dt = 0.05 ;

### 3. A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

* MPC Preprocessing

The waypoints of Car reference trajectory are given in the MAP co-ordinate system. These waypoints are transformed into the vehicle co-ordinate system.
This transformation is required to perform the calculations consistently in vehicle co-ordinate system.
Below is the code snippet from main.cpp

          for (unsigned int i = 0; i < ptsx.size(); i++)
          {
            double shift_x = ptsx[i] - px;
            double shift_y = ptsy[i] - py;
            ptsx[i] = shift_x * cos(0-psi) - shift_y * sin(0-psi);
            ptsy[i] = shift_x * sin(0-psi) + shift_y * cos(0-psi);
          }

* Polynomial Fitting

As the waypoints are in vehicle co-ordinate system, a third order polynomial is then fitted.
Below is the code snippet from main.cpp

          Eigen::VectorXd x_temp = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsx.data(), ptsx.size());
          Eigen::VectorXd y_temp = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsy.data(), ptsy.size());

          // Measure Polynomial Coefficients.
          auto coeffs = polyfit(x_temp, y_temp, 3);

Complete code for this step can be found in main.cpp


### 4. The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

Using 100 millisecond of latency. Since this is an additional latency added we had to incorporate it otherwise trajectory of path would have been compromised. Also the car movement will be affected due to the osscilations caused. Hence to tackle this issue I predicted the state forward at the latency time itself before giving that input to the solver.

Steps used:

* Calculate initial latency time for state values (px, py & psi).
* Calculate the errors (cte & epsi) at the beginning of timestep.
* Update the error values to incorporate latency considerations.
* Feed the state values and coeffs to the solver.

Following is the code snippet from main.cpp showing the latency consideration.

    double steering;
    double throttling;

    v *= 1.6094;
    steering = j[1]["steering_angle"];
    throttling = j[1]["throttle"];
    const double Lf = 2.67;

    // Predict the state
    const double lat = 0.1;
    px += v * cos(psi) * lat;
    py += v * sin(psi) * lat;
    psi += - v * steering/Lf * lat;
    v += throttling * lat;

    // Convert reference angle
    for (unsigned int i = 0; i < ptsx.size(); i++)
    {
      // Shift Car reference Angle to 90 degrees
      double shift_x = ptsx[i] - px;
      double shift_y = ptsy[i] - py;

      ptsx[i] = shift_x * cos(0-psi) - shift_y * sin(0-psi);
      ptsy[i] = shift_x * sin(0-psi) + shift_y * cos(0-psi);
    }

    Eigen::VectorXd ptsx_t = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsx.data(), ptsx.size());
    Eigen::VectorXd ptsy_t = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsy.data(), ptsy.size());

    // Measure Polynomial Coefficients.
    auto coeffs = polyfit(ptsx_t, ptsy_t, 3);

    cout << "Coeffs Size: " << coeffs.size() << endl;
    for (unsigned int i = 0; i < coeffs.size(); i++)
    {
      cout << coeffs[i] << endl;
    }

    // Calculate cte & epsi
    double cte = polyeval(coeffs, 0);
    //double epsi = psi - atan(coeffs[1]);
    double epsi = -atan(coeffs[1]);    // Since psi becomes zero we can remove that from the equation.

    // Incorporate latency considerations
    cte += v * sin(epsi) * lat;
    epsi += v * steering/Lf * lat;

    // Define the state vector
    Eigen::VectorXd state(6);
    state << 0, 0, 0, v, cte, epsi;

    // Call MPC Solver
    auto vars = mpc.Solve(state, coeffs);
