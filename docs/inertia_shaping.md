# Leader-side inertia shaping

The Cartesian controller provides two opt-in, measured-wrench paths for experiments with a
force-sensing leader robot. Both paths are disabled by default. They are intended for the leader
(`gravity_compensation` in the IRIS bilateral stack), not for the follower Cartesian impedance
controller.

## Control law

Let `f_e[k]` be the environment wrench reflected from the follower, `w_h[k]` the compensated local
leader F/T measurement, and `J[k]` the leader end-effector Jacobian. The configured static sign
`s_h` is either `-1` or `+1` and aligns the sensor convention with the controller convention. When
activation-local zero capture is enabled, `b_h` is the mean of the first configured number of
samples after controller activation and the active path uses `w_h[k] - b_h`. Its output remains
zero while the mean is being captured.

The active-measurement path is the behavior that was active in the original IRIS local Panda
driver:

\[
\bar w_h[k] = \alpha_h \bar w_h[k-1] + (1-\alpha_h)(s_h w_h[k]-b_h),
\]

\[
\tau_h[k] = J[k]^T\operatorname{diag}(s_f,s_f,s_f,s_\mu,s_\mu,s_\mu)\bar w_h[k].
\]

The one-sample dynamics-compensation path follows the intended equation documented beside the
original, commented experiment:

\[
u[k] = f_e[k] + \gamma \bar d[k-1], \qquad 0 < \gamma < 1,
\]

\[
\bar d[k] = \alpha_d \bar d[k-1]
           + (1-\alpha_d)(f_e[k]-s_h w_h[k]).
\]

The resulting wrench torque is `J[k]^T u[k]`. The residual is updated only after `u[k]` is formed,
so the compensation is exactly one controller cycle old. The original source mixed 6-D wrench and
7-D torque quantities and used an unexplained plus sign; this implementation keeps the intended
operation in one 6-D frame and exposes `s_h` explicitly.

When both paths are enabled, the commanded torque contains `J^T u + tau_h`. Evaluate the two paths
independently before combining them.

## Configuration

The measured wrench must already be bias-, payload-, and gravity-compensated. Its six numerical
components must use the same frame, axis order, torque reference point, and force/torque units as
the controller Jacobian. The controller deliberately performs no TF lookup in its real-time loop.
For the IRIS FR3 leader, use `use_local_jacobian: true` and the compensated leader-TCP wrench topic.

Configure the topic before the controller enters the configured state; changing the topic string
at runtime does not recreate the subscription.

```yaml
gravity_compensation:
  ros__parameters:
    use_local_jacobian: true
    topics:
      measured_wrench: netft_data_unbiased_tcp
    inertia_shaping:
      measurement_sign: 1       # determine using the sign test below
      measurement_timeout_s: 0.05
      active_measurement:
        enabled: false
        force_scale: 0.05       # begin well below the historical 2.0
        torque_scale: 0.0       # identify translation first
        filter_alpha: 0.99
        zero_on_enable: true    # keep the endpoint untouched while capturing
        zero_samples: 200       # 0.2 seconds at a 1 kHz controller rate
      one_sample:
        enabled: false
        gamma: 0.02             # begin with a small value
        filter_alpha: 0.99
```

At a 1 kHz controller rate, `alpha=0.99` has an approximate -3 dB cutoff of 1.6 Hz. At other rates,
use `f_c ~= -f_s ln(alpha)/(2 pi)` when matching the filter bandwidth.

If the wrench is missing or older than `measurement_timeout_s`, both measured-wrench paths are
suppressed and their filter state is reset. Ordinary pose control and reflected `target_wrench`
remain active. Zero capture affects only the local active-assistance term: it does not alter the
published F/T topics or the one-sample residual. A stale-wrench reset deliberately starts a new
zero capture before active assistance resumes.

## Static sign test

Do this before enabling one-sample compensation. Keep the robot away from contacts and joint
limits, retain the normal Franka collision/reflex protection, and keep an operator at the stop.

1. Confirm the compensated wrench is near zero while stationary, changes smoothly, and its TCP
   axes match the local Jacobian axes.
2. Disable reflected environment wrench, the one-sample path, and torque assistance. Set force
   scale to `0.05` and test one translational axis at a time with `measurement_sign: 1`.
3. Apply a small, slow hand force. The correct sign assists that motion; the wrong sign resists it or
   amplifies motion opposite to the hand force. Stop immediately if the robot accelerates or
   oscillates.
4. Repeat with `measurement_sign: -1`. Select the single sign that is consistently assistive on all
   three axes. Then repeat at very low torque scale for the rotational axes.
5. Record the selected sign with the sensor mounting and TCP configuration. Re-run the test after
   either changes.

## Evaluation protocol

Use the same start pose, payload, controller gains, operator, movement script, and contact object in
every trial. Randomize trial order where practical. Run these conditions in sequence:

| ID | Active measurement | One-sample | Initial sweep |
|---|---:|---:|---|
| B0 | off | off | baseline |
| A1 | on | off | force scale `0.05, 0.1, 0.2, 0.4`; torque remains `0` |
| A2 | on | off | torque scale `0.02, 0.05, 0.1`; retain accepted force scale |
| D1 | off | on | gamma `0.02, 0.05, 0.1, 0.2` |
| C1 | on | on | only the best stable A and D settings |

For each point, perform at least three 20-second free-space trials and three gentle-contact trials.
Start each sweep from its smallest value; never increase a gain after a reflex, sustained
oscillation, or degraded free-space behavior.

Record at least these leader topics: compensated local wrench, `target_wrench`, current TCP pose,
current TCP twist, and joint states. Report:

- operator force RMS and peak during the same free-space trajectory;
- apparent mass from force versus Cartesian acceleration over a defined frequency band;
- pose/velocity tracking and follower-to-leader cross-correlation delay;
- wrench and velocity spectral energy in 5--50 Hz as a chatter indicator;
- peak commanded/reflected wrench, torque-rate saturation, stale-wrench events, and reflexes;
- contact task time, peak contact force, and number of stable completions.

Accept a setting only when it reduces free-space effort or apparent mass without increasing chatter,
contact peak force, stale events, or safety interventions relative to B0.
