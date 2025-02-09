
import math, logging
import stepper, mathutil, chelper

class Actuator:
    def __init__(self, start, end):
        self.start = start
        self.end = end
        self.length = mathutil.matrix_mag(self.direction())
    
    def at_lerp(self, lerp):
        return self.direction().times(lerp).plus(self.start)
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
    def __init__(self, toolhead, config):
        stepper_configs = [config.getsection('stepper_' + a) for a in 'abc']
        rail_a = stepper.PrinterRail(stepper_configs[0], need_position_minmax=True, units_in_radians=False)
        a_endstop = rail_a.get_homing_info().position_endstop
        rail_b = stepper.PrinterRail(stepper_configs[1], need_position_minmax=True, units_in_radians=False, default_position_endstop=a_endstop)
        rail_c = stepper.PrinterRail(stepper_configs[2], need_position_minmax=True, units_in_radians=False, default_position_endstop=a_endstop)

        self.rails = [rail_a, rail_b, rail_c]

        # Read config
        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat('max_z_velocity', self.max_velocity, above=0., maxval=self.max_velocity)
        self.max_z_velocity = config.getfloat('max_z_accel', self.max_accel, above=0., maxval=self.max_accel)
        
        self.need_home = True
        self.home_position = tuple(self._actuators_to_cartesian())

        self.actuators = []
        self.arm2 = 0.0

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

    def check_move(self, move):

        if self.need_home:
            raise move.move_error("Must home first")
        
    def get_status(self, eventtime):
        return {
            'homed_axes': '' if self.need_home else 'xyz',
        }