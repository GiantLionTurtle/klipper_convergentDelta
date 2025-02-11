
import math, logging
import stepper, mathutil, chelper

class Actuator:
    def __init__(self, start, end):
        self.start = start
        self.end = end
        self.length = mathutil.matrix_mag(self.direction())
    
    def at_lerp(self, lerp):
        return mathutil.matrix_add(mathutil.matrix_mul(self.direction(), lerp), self.start)
    def at_dist(self, dist):
        return self.at_lerp(dist / self.length) # is this right within the context of clipper?

    def direction(self):
        # return self.end.minus(self.start)
        return mathutil.matrix_sub(self.end, self.start)

    def direction_normal(self):
        return mathutil.matrix_normalize(self.direction())

    def offset(self, by):
        return Actuator(mathutil.matrix_add(self.start, by), mathutil.matrix_add(self.end, by))

    def trim(self, start_trim: float, end_trim: float):
        dir_norm = self.direction_normal()
        return Actuator(mathutil.matrix_add(self.start, mathutil.matrix_mul(dir_norm, start_trim)), 
                        mathutil.matrix_sub(self.end, mathutil.matrix_mul(dir_norm, end_trim)))


class ConvergentDeltaKinematics:
    def __init__(self):

        self.rails = []

        # Read config
        self.max_velocity = 0.0
        self.max_accel = 0.0
        
        self.max_z_velocity = 0.0
        self.max_z_accel = 0.0

        self.actuators = []

        self.arm_lengths = arm_lengths = []
        self.arm2 = []

        self.need_home = True
        self.home_position = None

    @staticmethod
    def make_for_klipper(toolhead, config):
        out = ConvergentDeltaKinematics()

        stepper_configs = [config.getsection('stepper_' + a) for a in 'abc']
        rail_a = stepper.PrinterRail(stepper_configs[0], need_position_minmax=True, units_in_radians=False)
        a_endstop = rail_a.get_homing_info().position_endstop
        rail_b = stepper.PrinterRail(stepper_configs[1], need_position_minmax=True, units_in_radians=False, default_position_endstop=a_endstop)
        rail_c = stepper.PrinterRail(stepper_configs[2], need_position_minmax=True, units_in_radians=False, default_position_endstop=a_endstop)

        out.rails = [rail_a, rail_b, rail_c]

        # Read config
        out.max_velocity, out.max_accel = toolhead.get_max_velocity()
        out.max_z_velocity = config.getfloat('max_z_velocity', out.max_velocity, above=0., maxval=out.max_velocity)
        out.max_z_accel = config.getfloat('max_z_accel', out.max_accel, above=0., maxval=out.max_accel)

        out.actuators = [Actuator( [sconfig.getfloat('min_x'), sconfig.getfloat('min_y'), sconfig.getfloat('min_z')],
                                    [sconfig.getfloat('max_x'), sconfig.getfloat('max_y'), sconfig.getfloat('max_z')])
                                    for sconfig in stepper_configs]

        arm_length_a = stepper_configs[0].getfloat('arm_length', above=0)
        out.arm_lengths = arm_lengths = [sconfig.getfloat('arm_length', arm_length_a, above=0)
                                            for sconfig in stepper_configs]
        out.arm2 = [arm**2 for arm in arm_lengths]

        out.need_home = True
        out.home_position = tuple(out._actuators_to_cartesian([0.0, 0.0, 0.0]))
        

        for rail, arm2, act in zip(out.rails, out.arm2, out.actuators):
            rail.setup_itersolve('convergent_delta_stepper_alloc', arm2, 
                act.start[0], act.start[1], act.start[2], act.end[0], act.end[1], act.end[2])
        for s in out.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)

        out.set_position([0.0, 0.0, 0.0], "")
        return out

    @staticmethod
    def make_headless(actuators, arm2):
        out = ConvergentDeltaKinematics()
        out.actuators = actuators
        out.arm2 = [arm2, arm2, arm2]
        return out

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]
    
    def _actuators_to_cartesian(self, spos):
        sphere_coords = [act.at_dist(sp) for act, sp in zip(self.actuators, spos)]
        return mathutil.trilateration(sphere_coords, self.arm2)
    
    def calc_position(self, stepper_positions):
        spos = [stepper_positions[rail.get_name()] for rail in self.rails]
        return self._actuators_to_cartesian(spos)

    def set_position(self, newpos, homing_axes):
        for rail in self.rails:
            rail.set_position(newpos)
        
        if homing_axes == "xyz":
            self.need_home = False
    def clear_homing_state(self, clear_axes):
        if clear_axes:
            self.need_home = True
    def home(self, homing_state):
        # Home all axes at the same time
        homing_state.set_axes([0, 1, 2])
        forcepos = list(self.home_position)
        forcepos[2] = forcepos[2]-100
        homing_state.home_rails(self.rails, forcepos, self.home_position)

    def check_move(self, move):

        if self.need_home:
            raise move.move_error("Must home first")
        
    def get_status(self, eventtime):
        return {
            'homed_axes': '' if self.need_home else 'xyz',
        }

def load_kinematics(toolhead, config):
    return ConvergentDeltaKinematics.make_for_klipper(toolhead, config)
