class Pid(object):
    """PID controller implementation

    Very detailed documentation of the implementation @:
    http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
    Features:
    - handles integral windup by clamping the I term
    - changes to sample time

    - derivative kick is eliminated (using negative derivation of input instead
      of derivation of error)

    - changes to tuning parameters (mainly Ki) are handled correctly
      -> use ITerm instead
    """
    def __init__(self, sample_time_ms=200):
        self.curr_input = self.last_input = 0
        self.output = 0
        self.output_min = self.output_max = 0
        self.setpoint = 0
        # P-term and D-term are only for debugging purposes to
        # investigate the state of the controller
        self.p_term = 0
        self.i_term = 0
        self.d_term = 0

        self.kp = self.ki = self.kd = 0

        self.sample_time_ms = sample_time_ms
        self.last_compute_time = 0
        self.auto_mode = False

        # By default, assume a normal process -> positive error
        # requires positive change of the output
        self.output_reverse_direction = False

    def compute(self, now):
        if not self.auto_mode:
            return
        # TODO: implement time subtraction that accounts for overflow
        # in a clean way
        time_change = (now - self.last_compute_time) & ((1 << 32) - 1)
        if time_change >= self.sample_time_ms:
            # Calculate all error related variables
            error = self.setpoint - self.curr_input
            # I term
            self.i_term += self.ki * error
            # handle integral windup
            self.clamp_i_term()

            # D term - derivative on measure (not on error)
            d_input = self.curr_input - self.last_input

            # Calculate the PID output
            self.p_term = self.kp * error
            self.d_term = self.kd * d_input
            self.output = self.p_term + self.i_term - self.d_term
            self.clamp_output()

            # store input variables and sample time stamp for the next iteration
            self.last_input = self.curr_input
            self.last_compute_time = now

            print('Recalculating PID: error %s iterm %s dinput: %s' %
                  (error, self.i_term, d_input))

    def set_pid_params(self, kp, ki, kd):
        # all params have to be positive to handle direction of the
        # output properly
        if (kp < 0) or (ki < 0) or (kd < 0):
            return

        sample_time_sec = self.sample_time_ms / 1000.0

        self.kp = kp
        self.ki = ki * sample_time_sec
        self.kd = kd / sample_time_sec

        if self.output_reverse_direction:
            self.kp = -self.kp
            self.ki = -self.ki
            self.kd = -self.kd

    def set_sample_time(self, new_sample_time_ms):
        if new_sample_time_ms > 0:
            ratio = new_sample_time_ms / float(self.sample_time_ms)
            self.ki *= ratio;
            self.kd /= ratio;
            self.sample_time_ms = new_sample_time_ms

    def clamp_output(self):
        if self.output > self.output_max:
            self.output = self.output_max
        elif self.output < self.output_min:
            self.output = self.output_min

    def clamp_i_term(self):
        if self.i_term > self.output_max:
            self.i_term = self.output_max
        elif self.i_term < self.output_min:
            self.i_term = self.output_min

    def set_output_limits(self, output_min, output_max):
        if output_min > output_max:
            return
        self.output_min = output_min
        self.output_max = output_max
        # adjust the current output
        self.clamp_output()
        self.clamp_i_term()

    def enable_auto_mode(self):
        if not self.auto_mode:
            self.init()
            print('Enabling PID auto mode')
            self.auto_mode = True

    def disable_auto_mode(self):
        self.auto_mode = False

    def set_output_direction(self, reverse_direction):
        self.output_reverse_direction = reverse_direction

    def init(self):
        self.last_input = self.curr_input
        self.i_term = self.output
        self.clamp_i_term()
